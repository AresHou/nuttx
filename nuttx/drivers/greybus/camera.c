/**
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/device.h>
#include <nuttx/device_camera.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/debug.h>
#include <apps/greybus-utils/utils.h>
#include <arch/byteorder.h>

#include "camera-gb.h"

#define GB_CAMERA_VERSION_MAJOR 0
#define GB_CAMERA_VERSION_MINOR 1

/* Camera protocol operational model */
#define STATE_REMOVED       0
#define STATE_INSERTED      1
#define STATE_CONNECTED     2
#define STATE_UNCONFIGURED  3
#define STATE_CONFIGURED    4
#define STATE_STREAMING     5

/**
 * Capture request information for queue
 */
struct capture_req {
    /** queue entry */
    sq_entry_t  entry;
    /** setting for capture to driver */
    struct capture_info capt_info;
};

/**
 * Camera protocol private information.
 */
struct gb_camera_info {
    /** CPort from greybus */
    unsigned int    cport;
    /** Camera driver handle */
    struct device   *dev;
    /** Camera operational model */
    uint8_t         state;
    /** Capture queue */
    sq_queue_t      capt_queue;
    /** Semaphore for capture */
    sem_t           capt_sem;
    /** Thread for capture */
    pthread_t       capt_thread;
    /** Thread stop flag */
    uint32_t        thread_stop;
    /** Semaphore for flush */
    sem_t           flush_sem;
    /** keep the last capture request id */
    uint32_t        last_id;
    /** perform flush */
    uint32_t        flush;
};

static struct gb_camera_info *info = NULL;

/**
 * @brief Set configuration to driver and get accepted data
 *
 * @param num_streams Number of streams
 * @param flags Returns status for the input parameters
 * @param cfg_sup_reqest Data from host request
 * @param cfg_answer Buffer for getting driver accepted data
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static int set_streams_cfg(uint16_t num_streams, uint16_t *flags,
                       struct gb_stream_config_req *cfg_sup_reqest,
                       struct streams_cfg_ans *cfg_answer)
{
    struct streams_cfg_sup *cfg_support;
    int i, ret;

    if (info->state != STATE_UNCONFIGURED) {
        return GB_OP_INVALID;
    }

    cfg_support = malloc(num_streams *sizeof(cfg_support));
    if (!cfg_support) {
        ret = GB_OP_NO_MEMORY;
        goto err_free_mem;
    }

    for (i = 0; i < num_streams; i++) {
        cfg_support[i].width = le16_to_cpu(cfg_sup_reqest[i].width);
        cfg_support[i].height = le16_to_cpu(cfg_sup_reqest[i].height);
        cfg_support[i].format = le16_to_cpu(cfg_sup_reqest[i].format);
        cfg_support[i].padding = le16_to_cpu(cfg_sup_reqest[i].padding);
    }

    /* driver shall check the num_streams, it can't exceed its capability */
    ret = device_camera_set_streams_cfg(info->dev, num_streams, flags,
                                        cfg_support, cfg_answer);
    if (ret) {
        ret = gb_errno_to_op_result(ret);
        goto err_free_mem;
    }

    /* if driver not accpet the settings, it stay on unconfigured */
    if (!flags) {
        ret = device_camera_power_up(info->dev);
        if (ret) {
            ret = gb_errno_to_op_result(ret);
            goto err_free_mem;
        }
        info->state = STATE_CONFIGURED;
    }

    free(cfg_support);
    return 0;

err_free_mem:
    free(cfg_support);
    return ret;
}

/**
 * @brief Get camera support configuration
 *
 * @param num_streams Number of streams
 * @param cfg_answer Buffer for getting driver configuration
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static int get_supported_cfg(uint16_t num_streams,
                         struct streams_cfg_ans *cfg_answer)
{
    int ret;

    if (info->state != STATE_CONFIGURED) {
        return GB_OP_INVALID;
    }

    ret = device_camera_get_support_strm_cfg(info->dev, num_streams,
                                             cfg_answer);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    ret = device_camera_power_down(info->dev);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }
    info->state = STATE_UNCONFIGURED;

    return 0;
}

/**
 * @brief Capture thread for handling host capture command.
 *
 * When host send the capture into device, it queues the capture command and its
 * parameter into a queue, it dequeues after driver finishes its operation by
 * itself or by stop function.
 *
 * The queue will be flushed when the flush funciton called by host.
 *
 * @param data The regular thread data.
 * @return None.
 */
static void *capt_thread(void *data)
{
    struct capture_req *capt_req;
    int ret;

    while (1) {
        if (sq_empty(&info->capt_queue)) {
            sem_wait(&info->capt_sem);
        }

        if (info->thread_stop) {
            break;
        }

        /** dequeue from capture queue */
        if (!sq_empty(&info->capt_queue)) {
            capt_req = (struct capture_req *)sq_remfirst(&info->capt_queue);
            info->state = STATE_STREAMING;
            /* start capture in blocking mode */
            ret = device_camera_start_capture(info->dev, &capt_req->capt_info);
            if (ret) {
                gb_error("error in camera capture thread. \n");
            }
            /* wait for capture stop, by device or flush function */
            sem_wait(&info->capt_sem);

            info->state = STATE_CONFIGURED;
            free(capt_req);
        }

        if (info->flush) { /* clean queue */
            while (!sq_empty(&info->capt_queue)) {
                capt_req = (struct capture_req *)sq_remfirst(&info->capt_queue);
                free(capt_req);
            }
            info->flush = 0;
        }
    }
    return NULL;
}

/**
 * @brief Returns the major and minor Greybus Camera Protocol version number
 *
 * This operation returns the major and minor version number supported by
 * Greybus Camera Protocol
 *
 * @param operation Pointer to structure of Greybus operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_camera_protocol_version(struct gb_operation *operation)
{
    struct gb_camera_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_CAMERA_VERSION_MAJOR;
    response->minor = GB_CAMERA_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Get Camera capabilities
 *
 * This operation retrieves the list of capabilities of the Camera Module and
 * then returns to host.
 *
 * @param operation Pointer to structure of Greybus operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_camera_capabilities(struct gb_operation *operation)
{
    struct gb_camera_capabilities_response *response;
    uint8_t capabilities[GB_CAP_MAX_DATA_SIZE];
    uint16_t size = GB_CAP_MAX_DATA_SIZE;
    int ret;

    if (info->state != STATE_CONNECTED) {
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response) + size);
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    /* camera module capabilities */
    ret = device_camera_capabilities(info->dev, &size, capabilities);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    response->size = cpu_to_le16(size);
    memcpy(response->capabilities, &capabilities, size);

    return GB_OP_SUCCESS;
}

/**
 * @brief Configure camera module streams
 *
 * The Configure Streams operation configures or unconfigures the Camera Module
 * to prepare or stop video capture.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_configure_streams(struct gb_operation *operation)
{
    struct gb_camera_configure_streams_request *request;
    struct gb_camera_configure_streams_response *response;
    struct gb_stream_config_req  *cfg_sup_reqest;
    struct gb_stream_config_resp *cfg_ans_resp;
    struct streams_cfg_ans *cfg_answer = NULL;
    uint16_t num_streams;
    uint16_t flags;
    uint32_t size;
    int i, ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    num_streams = le16_to_cpu(request->num_streams);
    if (num_streams) {
        /**
         * if num_stream is not 0, set configuration to camera and then get
         * final setting from camera.
         */
        cfg_sup_reqest = request->config;

        ret = device_camera_get_required_size(info->dev, SIZE_CONFIG_ANSWER,
                                              &size);
        if (ret) {
            return gb_errno_to_op_result(ret);
        }

        cfg_answer = malloc(size);
        if (!cfg_answer) {
            return GB_OP_NO_MEMORY;
        }
        ret = set_streams_cfg(num_streams, &flags, cfg_sup_reqest, cfg_answer);
        if (ret) {
            goto err_free_mem;
        }

    } else {
        /**
         *  if num_streams is 0, it means the host queries camera supported
         *  configurations or set camer to unconfigured state
         */
        ret = device_camera_get_required_size(info->dev, SIZE_CONFIG_SUPPORT,
                                              &size);
        if (ret) {
            return gb_errno_to_op_result(ret);
        }
        num_streams = size / sizeof(cfg_answer);

        cfg_answer = malloc(size);
        if (!cfg_answer) {
            return GB_OP_NO_MEMORY;
        }

        ret = get_supported_cfg(num_streams, cfg_answer);
        if (ret) {
            goto err_free_mem;
        }
    }

    /* make response data */
    response = gb_operation_alloc_response(operation,
                                           sizeof(*response) + size);
    response->num_streams = cpu_to_le16(num_streams);
    response->flags = flags;
    cfg_ans_resp = response->config;
    for (i = 0; i < num_streams; i++) {
        cfg_ans_resp[i].width = cpu_to_le16(cfg_answer[i].width);
        cfg_ans_resp[i].height = cpu_to_le16(cfg_answer[i].height);
        cfg_ans_resp[i].format = cpu_to_le16(cfg_answer[i].format);
        cfg_ans_resp[i].virtual_channel = cfg_answer[i].virtual_channel;
        cfg_ans_resp[i].data_type = cfg_answer[i].data_type;
        cfg_ans_resp[i].max_size = cpu_to_le32(cfg_answer[i].max_size);
    }

    free(cfg_answer);
    /* send back the response to host by return */
    return GB_OP_SUCCESS;

err_free_mem:
    free(cfg_answer);
    return ret;
}

/**
 * @brief Engage camera capture operation
 *
 * It enqueues a capture request and leaves.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_capture(struct gb_operation *operation)
{
    struct gb_camera_capture_request *request;
    struct capture_info *capt_info;
    struct capture_req *capt_req; /* into queue */
    irqstate_t flags;

    if (info->state != STATE_CONFIGURED && info->state != STATE_STREAMING) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    if (le32_to_cpu(request->request_id) <= info->last_id) {
        return GB_OP_INVALID;
    }

    capt_req = malloc(sizeof(capt_req));
    if(!capt_req) {
        return GB_OP_NO_MEMORY;
    }

    capt_info = &capt_req->capt_info;
    capt_info->request_id = le32_to_cpu(request->request_id);
    capt_info->streams = request->streams;
    capt_info->padding = request->padding;
    capt_info->num_frames = le32_to_cpu(request->num_frames);
    capt_info->callback = NULL; /* block mode */

    info->last_id = capt_info->request_id;

    flags = irqsave();
    if (sq_empty(&info->capt_queue)) {
        sq_addlast(&capt_req->entry, &info->capt_queue);
        irqrestore(flags);

        sem_post(&info->capt_sem);
    } else {
        sq_addlast(&capt_req->entry, &info->capt_queue);
        irqrestore(flags);
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Stop and flush the camera capture
 *
 * The Flush operation removes all Capture requests from the queue and stops
 * frame transmission as soon as possible.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_flush(struct gb_operation *operation)
{
    struct gb_camera_flush_response *response;
    uint32_t request_id = 0;
    int ret;

    if (info->state != STATE_STREAMING && info->state != STATE_CONNECTED) {
        return GB_OP_INVALID;
    }

    if (!sq_empty(&info->capt_queue)) {
        info->flush = 1;
    }

    ret = device_camera_stop_capture(info->dev, &request_id);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    if (info->state == STATE_STREAMING) {
        sem_post(&info->flush_sem);
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->request_id = cpu_to_le32(request_id);

    return GB_OP_SUCCESS;
}

/**
 * @brief Request meta-data from camera module
 *
 * Allows the Camera to provide meta-data associated with a frame to the AP over
 * Greybus.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_meta_data(struct gb_operation *operation)
{
    struct gb_camera_meta_data_request *request;
    struct gb_camera_meta_data_response *response;
    struct meta_data_info meta_data;
    uint32_t size;
    uint8_t *data;
    int ret;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    ret = device_camera_get_required_size(info->dev, SIZE_META_DATA, &size);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    data = malloc(size);
    if (!data) {
        return GB_OP_NO_MEMORY;
    }

    meta_data.request_id = le32_to_cpu(request->request_id);
    meta_data.frame_number = le16_to_cpu(request->frame_number);
    meta_data.stream = request->stream;
    meta_data.padding = request->padding;
    meta_data.data = data;

    ret = device_camera_get_meta_data(info->dev, &meta_data);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    response = gb_operation_alloc_response(operation,
                                           sizeof(*response) + size);

    response->request_id = cpu_to_le32(meta_data.request_id);
    response->frame_number = cpu_to_le16(meta_data.frame_number);
    response->stream = meta_data.stream;
    response->padding = meta_data.padding;
    memcpy(response->data, data, size);
    free(data);

    return GB_OP_SUCCESS;
}

/**
 * @brief Greybus Camera Protocol initialize function
 *
 * This function performs the protocol inintilization, such as open the
 * cooperation device driver and create buffers etc.
 *
 * @param cport CPort number
 * @return 0 on success, negative errno on error
 */
static int gb_camera_init(unsigned int cport)
{
    int ret;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    info->cport = cport;

    info->last_id = 0;
    info->state = STATE_REMOVED; /* STATE_INSERTED ? */

    info->dev = device_open(DEVICE_TYPE_CAMERA_HW, 0);
    if (!info->dev) {
        return -EIO;
        goto err_free_info;
    }

    info->state = STATE_CONNECTED; /* STATE_UNCONFIGURED ? */

    sq_init(&info->capt_queue);

    ret = sem_init(&info->capt_sem, 0, 0);
    if (ret) {
        goto err_close_device;
    }

    ret = sem_init(&info->flush_sem, 0, 0);
    if (ret) {
        goto err_destroy_capt_sem;
    }

    ret = pthread_create(&info->capt_thread, NULL, capt_thread, info);
    if (ret) {
        goto err_destroy_flush_sem;
    }

    return 0;

err_destroy_flush_sem:
    sem_destroy(&info->capt_sem);
err_destroy_capt_sem:
    sem_destroy(&info->capt_sem);
err_close_device:
    device_close(info->dev);
err_free_info:
    free(info);

    return ret;
}

/**
 * @brief Greybus Camera Protocol deinitialize function
 *
 * This function is called when protocol terminated.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return None.
 */
static void gb_camera_exit(unsigned int cport)
{
    struct capture_info *capt_info;

    DEBUGASSERT(cport == info->cport);

    if (info->capt_thread != (pthread_t)0) {
        info->thread_stop = 1;
        sem_post(&info->capt_sem);
        sem_post(&info->flush_sem);
        pthread_join(info->capt_thread, NULL);
    }

    sem_destroy(&info->capt_sem);
    sem_destroy(&info->flush_sem);

    while (!sq_empty(&info->capt_queue)) {
        capt_info = (struct capture_info *) sq_remfirst(&info->capt_queue);
        free(capt_info);
    }

    device_close(info->dev);

    free(info);
    info = NULL;
}

/**
 * @brief Greybus Camera Protocol operation handler
 */
static struct gb_operation_handler gb_camera_handlers[] = {
    GB_HANDLER(GB_CAMERA_TYPE_PROTOCOL_VERSION, gb_camera_protocol_version),
    GB_HANDLER(GB_CAMERA_TYPE_CAPABILITIES, gb_camera_capabilities),
    GB_HANDLER(GB_CAMERA_TYPE_CONFIGURE_STREAMS, gb_camera_configure_streams),
    GB_HANDLER(GB_CAMERA_TYPE_CAPTURE, gb_camera_capture),
    GB_HANDLER(GB_CAMERA_TYPE_FLUSH, gb_camera_flush),
    GB_HANDLER(GB_CAMERA_TYPE_META_DATA, gb_camera_meta_data),
};

/**
 * @brief Greybus Camera Protocol driver ops
 */
static struct gb_driver gb_camera_driver = {
    .init = gb_camera_init,
    .exit = gb_camera_exit,
    .op_handlers = gb_camera_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_camera_handlers),
};

/**
 * @brief Register Greybus Camera Protocol
 *
 * @param cport CPort number.
 */
void gb_camera_register(int cport)
{
    gb_register_driver(cport, &gb_camera_driver);
}
