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
 * Camera protocol private information.
 */
struct gb_camera_info {
    /** CPort from greybus */
    unsigned int    cport;
    /** Camera driver handle */
    struct device   *dev;
    /** Camera operational model */
    uint8_t         state;
};

static struct gb_camera_info *info = NULL;

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

    lldbg("gb_camera_protocol_version() + \n");

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_CAMERA_VERSION_MAJOR;
    response->minor = GB_CAMERA_VERSION_MINOR;

    lldbg("gb_camera_protocol_version() - \n");

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
    uint8_t *capabilities;
    uint16_t size;
    int ret;

    lldbg("gb_camera_capabilities() + \n");

    if (info->state != STATE_CONNECTED) {
        lldbg("state error %d \n", info->state);
        return GB_OP_INVALID;
    }

    ret = device_camera_get_required_size(info->dev, SIZE_CAPABILITIES, &size);
    if (ret) {
        return gb_errno_to_op_result(ret);
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

    lldbg("gb_camera_capabilities() - \n");

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
    struct gb_stream_config_req  *cfg_set_req;  /* in request */
    struct gb_stream_config_resp *cfg_ans_resp; /* in response */
    struct streams_cfg_req *cfg_request;    /* for put into driver */
    struct streams_cfg_ans *cfg_answer;     /* for get from driver */
    uint16_t num_streams;
    uint16_t flags;
    int i, ret;

    lldbg("gb_camera_configure_streams() + \n");

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message \n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    num_streams = le16_to_cpu(request->num_streams);
    if (num_streams) {
        /**
         * if num_stream is not 0, set configuration to camera and then get
         * accepted configuration from camera sensor.
         */
        if (info->state != STATE_UNCONFIGURED ||
            num_streams > MAX_STREAMS_NUM) {
            return GB_OP_INVALID;
        }

        /* get from request package */
        cfg_set_req = request->config;

        /* alloc for sending to the driver */
        cfg_request = malloc(num_streams * sizeof(*cfg_request));
        if (!cfg_request) {
            return GB_OP_NO_MEMORY;
        }
        
        lldbg("num_streams = %d \n", num_streams);
        
        /* todo: check is there performance issue on below conversion*/
        /* convert data for driver */
        for (i = 0; i < num_streams; i++) {
            lldbg("\n");
            
            cfg_request[i].width = le16_to_cpu(cfg_set_req[i].width);
            cfg_request[i].height = le16_to_cpu(cfg_set_req[i].height);
            cfg_request[i].format = le16_to_cpu(cfg_set_req[i].format);
            cfg_request[i].padding = le16_to_cpu(cfg_set_req[i].padding);

            lldbg("    width = %d \n", cfg_request[i].width);
            lldbg("    height = %d \n", cfg_request[i].height);
            lldbg("    format = %d \n", cfg_request[i].format);
            lldbg("    padding = %d \n", cfg_request[i].padding);
        }

        /* alloc for getting answer from driver */
        cfg_answer = malloc(MAX_STREAMS_NUM * sizeof(*cfg_answer));
        if (!cfg_answer) {
            ret = GB_OP_NO_MEMORY;
            goto err_free_req_mem;
        }

        /* driver shall check the num_streams, it can't exceed its capability */
        ret = device_camera_set_streams_cfg(info->dev, &num_streams, &flags,
                                            cfg_request, cfg_answer);
        if (ret) {
            ret = GB_OP_NO_MEMORY;
            goto err_free_ans_mem;
        }

        /* check driver returned num_streams must be under MAX_STREAMS_NUM */
        if (num_streams > MAX_STREAMS_NUM) {
            ret = GB_OP_INVALID;
            goto err_free_ans_mem;
        }

        /**
         * flags is 0 for driver accept the settings, change state to
         * configured
         */
        if (!flags) {
            ret = device_camera_power_up(info->dev);
            if (ret) {
                ret = gb_errno_to_op_result(ret);
                goto err_free_ans_mem;
            }
            info->state = STATE_CONFIGURED;
        }
        /**
         * if flags is not 0, it keeps in uncinfigured but still return
         * the modification for negotiation
         */

        /* make response data */
        response = gb_operation_alloc_response(operation, sizeof(*response) +
                                  num_streams * sizeof(*cfg_ans_resp));
        response->num_streams = cpu_to_le16(num_streams);
        response->flags = flags;
        lldbg("Resp: \n");
        lldbg("flags = 0x%2x: \n", response->flags);
        cfg_ans_resp = response->config;
        for (i = 0; i < num_streams; i++) {
            lldbg("\n");
            lldbg("    width = %d \n", cfg_answer[i].width);
            lldbg("    height = %d \n", cfg_answer[i].height);
            lldbg("    format = %d \n", cfg_answer[i].format);
            lldbg("    virtual_channel = %d \n", cfg_answer[i].virtual_channel);
            lldbg("    data_type = %d \n", cfg_answer[i].data_type);
            lldbg("    max_size = %d \n", cfg_answer[i].max_size);
            
            cfg_ans_resp[i].width = cpu_to_le16(cfg_answer[i].width);
            cfg_ans_resp[i].height = cpu_to_le16(cfg_answer[i].height);
            cfg_ans_resp[i].format = cpu_to_le16(cfg_answer[i].format);
            cfg_ans_resp[i].virtual_channel = cfg_answer[i].virtual_channel;
            cfg_ans_resp[i].data_type = cfg_answer[i].data_type;
            cfg_ans_resp[i].max_size = cpu_to_le32(cfg_answer[i].max_size);
        }

        free(cfg_request);
        free(cfg_answer);
    } else {
        /**
         *  if num_streams is 0, set camer to unconfigured state
         */
        if (info->state == STATE_CONFIGURED) {
            info->state = STATE_UNCONFIGURED;
            ret = device_camera_power_down(info->dev);
            if (ret) {
                return gb_errno_to_op_result(ret);
            }
        }
        else {
            return GB_OP_INVALID;
        }
    }

    lldbg("gb_camera_configure_streams() - \n");

    /* send back the response to host by return */
    return GB_OP_SUCCESS;

err_free_ans_mem:
    free(cfg_answer);
err_free_req_mem:
    free(cfg_request);
    return ret;
}

/**
 * @brief Engage camera capture operation
 *
 * It tell camera module to start capture.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_capture(struct gb_operation *operation)
{
    struct gb_camera_capture_request *request;
    struct capture_info *capt_req;
    int ret;

    lldbg("gb_camera_capture() + \n");

    if (info->state != STATE_CONFIGURED && info->state != STATE_STREAMING) {
        return GB_OP_INVALID;
    }

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    capt_req = malloc(sizeof(*capt_req));
    if(!capt_req) {
        return GB_OP_NO_MEMORY;
    }

    capt_req->request_id = le32_to_cpu(request->request_id);
    capt_req->streams = request->streams;
    capt_req->padding = request->padding;
    capt_req->num_frames = le32_to_cpu(request->num_frames);

    lldbg("    request_id = %d \n", capt_req->request_id);
    lldbg("    streams = %d \n", capt_req->streams);
    lldbg("    padding = %d \n", capt_req->padding);
    lldbg("    num_frames = %d \n", capt_req->num_frames);

    ret = device_camera_capture(info->dev, capt_req);
    if (ret) {
        gb_error("error in camera capture thread. \n");
        ret = gb_errno_to_op_result(ret);
        goto err_free_mem;
    }

    free(capt_req);

    lldbg("gb_camera_capture() - \n");

    return GB_OP_SUCCESS;

err_free_mem:
    free(capt_req);
    return ret;
}

/**
 * @brief Flush the camera capture
 *
 * The Flush operation calls camera driver to flush capture.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_camera_flush(struct gb_operation *operation)
{
    struct gb_camera_flush_response *response;
    uint32_t request_id = 0;
    int ret;

    lldbg("gb_camera_flush() + \n");

    if (info->state != STATE_STREAMING && info->state != STATE_CONNECTED) {
        return GB_OP_INVALID;
    }

    ret = device_camera_flush(info->dev, &request_id);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->request_id = cpu_to_le32(request_id);
    lldbg("    request_id = %d + \n", request_id);
    
    lldbg("gb_camera_flush() + \n");

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
static uint8_t gb_camera_metadata(struct gb_operation *operation)
{
    struct gb_camera_meta_data_request *request;
    struct metadata_info meta_data;
    int ret;

    lldbg("gb_camera_metadata() + \n");

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);

    meta_data.request_id = le32_to_cpu(request->request_id);
    meta_data.frame_number = le16_to_cpu(request->frame_number);
    meta_data.stream = request->stream;
    meta_data.padding = request->padding;
    meta_data.data = request->data;

    lldbg("    request_id = %d \n", request->request_id);
    lldbg("    frame_number = %d \n", request->frame_number);
    lldbg("    stream = %d \n", request->stream);

    ret = device_camera_trans_metadata(info->dev, &meta_data);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    lldbg("gb_camera_metadata() - \n");

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

    lldbg("gb_camera_init + \n");
    
    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    info->cport = cport;

    info->state = STATE_REMOVED; /* STATE_INSERTED ? */

    info->dev = device_open(DEVICE_TYPE_CAMERA_HW, 0);
    if (!info->dev) {
        return -EIO;
        goto err_free_info;
    }

    info->state = STATE_CONNECTED; /* STATE_UNCONFIGURED ? */

    lldbg("gb_camera_init - \n");
    
    return 0;

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
    DEBUGASSERT(cport == info->cport);

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
    GB_HANDLER(GB_CAMERA_TYPE_METADATA, gb_camera_metadata),
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
