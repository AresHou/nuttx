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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/device_camera.h>

#define FAKE_SET_SIZE     2
#define FAKE_SIZE         1
#define FAKE_WIDTH        640
#define FAKE_HEIGHT       480
#define FAKE_FORMAT       0x3007
#define PADDING           0
#define FAKE_REQUEST_ID   1
#define FAKE_NUM_FRAMES   16
#define FAKE_STREAM       8
#define REQUEST_SIZE_FAIL 0

enum ov5645_pixel_format {
    _8bit_RGB_RAW,
    _10bit_RGB_RAW,
    RGB565,
    RGB555,
    RGB444,
    YUV422,
    YUV420,
    YCbCr422,
    format_max = 0xff,
};

uint32_t num_of_support_mode;

static int show_capabilities(struct device *dev)
{
    int result = 0;
    uint8_t capabilities = 0;
    uint16_t size = 0;

    result = device_camera_capabilities(dev, &size, &capabilities);
    if(!result) {
        printf("=== show capabilities ===\n");
        printf("capabilities: 0x%X \n", capabilities);
        printf("size: %d\n", size);
        printf("\n");
    } else {
        printf("ERROR! ov5645_capabilities failed: %d\n", result);
    }

    return result;
}

static int show_get_required_size(struct device *dev, uint8_t operation)
{
    int result = 0;
    uint32_t size = 0;

    result = device_camera_get_required_size(dev, operation, &size);
     if(!result) {
        num_of_support_mode = size;

        printf("=== show required size ===\n");
        printf("support type: %d,size: %d\n", operation, size);
    } else {
        printf("ERROR! ov5645_get_required_size failed: %d\n", result);
        num_of_support_mode = REQUEST_SIZE_FAIL;
    }

    return result;
}

static int show_get_support_strm_cfg(struct device *dev, uint8_t operation)
{
    int result = 0, i;
    uint16_t num_streams = 0;
    struct streams_cfg_ans *config;

    /* Get requirement size */
    show_get_required_size(dev, operation);
    if (!num_of_support_mode)
        return -EINVAL;

    config = zalloc(num_of_support_mode * sizeof(struct streams_cfg_ans));
    if (!config) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    /* get support stream config */
    result = device_camera_get_support_strm_cfg(dev, num_streams, config);
    if(!result) {
        printf("=== show support stream config ===\n");
        printf("The number of support modes: %d\n", num_of_support_mode);

        /* print all of ov5645 modes */
        for(i=0; i<num_of_support_mode; i++) {
           printf("--- mode%d ---\n", i);
           printf("width           : %d\n", config[i].width);
           printf("height          : %d\n", config[i].height);
           printf("format          : 0x%x\n", config[i].format);
           printf("virtual_channel : %d\n", config[i].virtual_channel);
           printf("data_type       : %d\n", config[i].data_type);
           printf("max_size        : %d\n", config[i].max_size);
           printf("\n");
        }
    } else
        printf("ERROR! ov5645_get_support_strm_cfg failed: %d\n", result);

    free(config);
    return result;
}

static int show_set_streams_cfg(struct device *dev)
{
    int result = 0;
    uint16_t num_streams = 0, flags = 0;
    struct streams_cfg_sup *config = NULL;
    struct streams_cfg_ans *answer = NULL;

    config = zalloc(sizeof(*config));
    if (!config) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    config->width = FAKE_WIDTH;
    config->height = FAKE_HEIGHT;
    config->format = YCbCr422;
    config->padding = PADDING;

    answer = zalloc(sizeof(*answer));
    if (!answer) {
        printf("ERROR! Fail to alloc memory. \n");
        result = -ENOMEM;
        goto err_free_config;
    }

    result = device_camera_set_streams_cfg(dev, num_streams, &flags, config,
                                           answer);
    if(!result) {
        printf("=== show streams config ===\n");
        printf("width           : %d\n",   answer->width);
        printf("height          : %d\n",   answer->height);
        printf("format          : 0x%x\n", answer->format);
        printf("virtual_channel : %d\n",   answer->virtual_channel);
        printf("data_type       : %d\n",   answer->data_type);
        printf("max_size        : %d\n",   answer->max_size);
        printf("flags           : %d\n",   flags);
        printf("\n");
    } else {
        printf("ERROR! ov5645_get_support_strm_cfg failed: %d\n", result);
   }

    free(answer);

err_free_config:
    free(config);

    return result;
}

static int show_get_current_strm_cfg(struct device *dev, uint8_t operation)
{
    int result = 0;
    uint16_t flags = 0;
    struct streams_cfg_ans *answer = NULL;

    /* Get requirement size */
    show_get_required_size(dev, operation);
    if (!num_of_support_mode)
        return -EINVAL;

    answer = zalloc(sizeof(*answer));
    if (!answer) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    result = device_camera_get_current_strm_cfg(dev, &flags, answer);
    if(!result) {
        printf("=== show current streams config ===\n");
        printf("width           : %d\n",   answer->width);
        printf("height          : %d\n",   answer->height);
        printf("format          : 0x%x\n", answer->format);
        printf("virtual_channel : %d\n",   answer->virtual_channel);
        printf("data_type       : %d\n",   answer->data_type);
        printf("max_size        : %d\n",   answer->max_size);
        printf("flags           : %d\n",   flags);
        printf("\n");
    } else {
        printf("ERROR! show_get_current_strm_cfg failed: %d\n", result);
    }

    free(answer);

    return result;
}

static int show_start_capture(struct device *dev)
{
    int result = 0;
    uint8_t cap_setting[FAKE_SET_SIZE] = {0};
    struct capture_info *capt_info = NULL;

    capt_info = zalloc(sizeof(*capt_info));
    if (!capt_info) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    capt_info->request_id =  FAKE_REQUEST_ID;
    capt_info->streams    =  FAKE_STREAM;
    capt_info->padding    =  PADDING;
    capt_info->num_frames =  FAKE_NUM_FRAMES;
    capt_info->settings   =  cap_setting;

    result = device_camera_start_capture(dev, capt_info);
    if(!result) {
        printf("=== Start capture ===\n");
        printf("\n");
    } else {
        printf("ERROR! show_start_capture failed: %d\n", result);
    }

    free(capt_info);

    return result;
}

static int show_stop_capture(struct device *dev)
{
    int result = 0;
    uint32_t request_id = 0;

    result = device_camera_stop_capture(dev, &request_id);
    if(!result) {
        printf("=== Stop_capture ===\n");
        printf("request_id: %d\n",request_id);
        printf("\n");
    } else {
        printf("ERROR! show_stop_capture failed: %d\n", result);
    }

    return result;
}

static int show_get_meta_data(struct device *dev, uint8_t operation)
{
    int result = 0;
    struct meta_data_info *meta_data;

    /* Get requirement size */
    show_get_required_size(dev, operation);
    if (!num_of_support_mode)
        return -EINVAL;

    meta_data = zalloc(num_of_support_mode * sizeof(struct meta_data_info));
    if (!meta_data) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    /* obtain meta data */
    result = device_camera_get_meta_data(dev, meta_data);
    if(!result) {
        printf("=== Show meta data ===\n");

        /* get meta data */
        printf("request_id   : %d\n", meta_data->request_id);
        printf("frame_number : %d\n", meta_data->frame_number);
        printf("stream       : %d\n", meta_data->stream);
        printf("padding      : %d\n", meta_data->padding);
        printf("data         : 0x%x\n", meta_data->data);
        printf("\n");
    } else
       printf("ERROR! ov5645_get_support_strm_cfg failed: %d\n", result);

    free(meta_data);
    return result;
}

static void print_usage(void) {
    printf("Usage: camera_test\n");
    printf("Parameters < c | s | t | p | u | r | m | e > \n");
    printf("          c : Get camera capabilities\n");
    printf("          s : Set streams config\n");
    printf("          t : Start stream\n");
    printf("          p : Stop stream\n");
    printf("          u : Get support stream config\n");
    printf("          r : Get current stream config\n");
    printf("          m : Meta data request\n");
    printf("          e : Exit test application\n");
    printf("\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
int camera_test_main(int argc, char *argv[]) {
#endif
    struct device *dev = NULL;
    char cmd = '\0';
    int ret = 0;

    printf("Power on camera sensor and get sendor ID... \n");

    /* open device */
    dev = device_open(DEVICE_TYPE_CAMERA_HW, 0);
    if (!dev) {
        printf("Failed to open camera driver \n");
        return EXIT_FAILURE;
    }

    print_usage();

    while((cmd = getchar()) != 'e')
    {
        switch (cmd) {
        case 'c':
            /** Camera capabilities */
            ret = show_capabilities(dev);
            break;
        case 'u':
            /** Get camera module supported configure */
            ret = show_get_support_strm_cfg(dev, SIZE_CONFIG_SUPPORT);
            break;
        case 's':
            /** Set configures to camera module */
            ret = show_set_streams_cfg(dev);
            break;
        case 'r':
            /** Get current configures from camera module */
            ret = show_get_current_strm_cfg(dev, SIZE_CONFIG_ANSWER);
           break;
        case 't':
            /** Start Capture */
            ret = show_start_capture(dev);
           break;
        case 'p':
            /** Stop Capture */
            ret = show_stop_capture(dev);
           break;
        case 'm':
            /** Get camera module supported configure */
            ret = show_get_meta_data(dev, SIZE_META_DATA);
           break;
        default:
            print_usage();
            return EXIT_FAILURE;
        }
    }
    device_close(dev);
    dev = NULL;

    return ret;
}
