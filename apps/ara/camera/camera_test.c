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

/* Image sizes */
/* VGA */
#define VGA_WIDTH           640
#define VGA_HEIGHT          480
/* QVGA */
#define QVGA_WIDTH          320
#define QVGA_HEIGHT         240
/* 720P */
#define _720P_WIDTH         1280
#define _720P_HEIGHT        720
/* 1080P */
#define _1080P_WIDTH        1920
#define _1080P_HEIGHT       1080
/* QSXGA */
#define QSXGA_WIDTH         2592
#define QSXGA_HEIGHT        1944
/* SXGA */
#define SXGA_WIDTH          1280
#define SXGA_HEIGHT         960
/* XGA */
#define XGA_WIDTH           1024
#define XGA_HEIGHT          768
/* MAX */

#define MODE_MIN          0
#define MODE_MAX          7

enum cam_pixel_format {
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

struct image_size {
    /** Image width in pixels */
    uint16_t    width;
    /** Image height in pixels */
    uint16_t    height;
};

uint32_t num_of_support_mode;

static int get_capabilities(struct device *dev)
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
        printf("ERROR! get_capabilities failed: %d\n", result);
    }

    return result;
}

static int test_get_required_size(struct device *dev, uint8_t operation)
{
    int result = 0;
    uint16_t size = 0;

    result = device_camera_get_required_size(dev, operation, &size);
     if(!result) {
        num_of_support_mode = size;

        printf("=== show required size ===\n");
        printf("support type: %d,size: %d\n", operation, size);
    } else {
        printf("ERROR! test_get_required_size failed: %d\n", result);
        num_of_support_mode = REQUEST_SIZE_FAIL;
    }

    return result;
}

static int set_streams_cfg(struct device *dev, struct image_size img_size)
{
    int result = 0;
    uint16_t num_streams = 0, flags = 0;
    struct streams_cfg_req *config = NULL;
    struct streams_cfg_ans *answer = NULL;

    config = zalloc(sizeof(*config));
    if (!config) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    printf("[%s]img_size.width = %d \n", __func__, img_size.width);
    printf("[%s]img_size.height = %d \n", __func__, img_size.height);

    config->width = img_size.width;
    config->height = img_size.height;
    config->format = YCbCr422;
    config->padding = PADDING;

    answer = zalloc(sizeof(*answer));
    if (!answer) {
        printf("ERROR! Fail to alloc memory. \n");
        result = -ENOMEM;
        goto err_free_config;
    }

    result = device_camera_set_streams_cfg(dev, &num_streams, &flags, config,
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
        printf("ERROR! set_streams_cfg failed: %d\n", result);
   }

    free(answer);

err_free_config:
    free(config);

    return result;
}

static int capture(struct device *dev)
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

    result = device_camera_capture(dev, capt_info);
    if(!result) {
        printf("=== Start capture ===\n");
        printf("Check if preview is working.\n");
        printf("\n");
    } else {
        printf("ERROR! apture failed: %d\n", result);
    }

    free(capt_info);

    return result;
}

static int flush(struct device *dev)
{
    int result = 0;
    uint32_t request_id = 0;

    result = device_camera_flush(dev, &request_id);
    if(!result) {
        printf("=== Stop_capture ===\n");
        printf("request_id: %d\n",request_id);
        printf("Check if preview is terminated.\n");
        printf("\n");
    } else {
        printf("ERROR! flush failed: %d\n", result);
    }

    return result;
}

static int metadata_transmit(struct device *dev, uint8_t operation)
{
    int result = 0;
    struct metadata_info *meta_data;

    /* Get requirement size */
    test_get_required_size(dev, operation);
    if (!num_of_support_mode)
        return -EINVAL;

    meta_data = zalloc(num_of_support_mode * sizeof(struct metadata_info));
    if (!meta_data) {
        printf("ERROR! Fail to alloc memory. \n");
        return -ENOMEM;
    }

    /* obtain meta data */
    result = device_camera_trans_metadata(dev, meta_data);
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
       printf("ERROR! show_get_meta_data failed: %d\n", result);

    free(meta_data);
    return result;
}

static void print_usage(void) {
    printf("Usage: camera_test < c | s | t | p | m | e >\n");
    printf("camera operation tests:\n");
    printf("          c : Get camera capabilities\n");
    printf("          s : Set streams configuration\n");
    printf("          t : Start to capture\n");
    printf("          p : Stop to capture\n");
    printf("          m : Meta-Data requests\n");
    printf("          e : Exit camera test application\n"); 
    printf("\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[]) {
#else
int camera_test_main(int argc, char *argv[]) {
#endif
    struct device *dev = NULL;
    struct image_size img_size;

    char cmd = '\0';
    int ret = 0, val = 0;

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
            /** Get Camera capabilities */
            ret = get_capabilities(dev);
            break;
        case 's':
            printf("1.VGA   - 640*480 \n");
            printf("2.QVGA  - 320*240 \n");
            printf("3.720P  - 1280*720 \n");
            printf("4.1080P - 1920*1080 \n");
            printf("5.QSXGA - 2592*1944 \n");
            printf("6.SXGA  - 1280*960 \n");
            printf("7.XGA   - 1024*768 \n");

            /* Check if input value is out of range */
            while((cmd = getchar()))
            {
                val = atoi(&cmd);
                if ((val > MODE_MIN) || (val < (MODE_MAX+1))) {
                  break;
                }
                else {
                  /* Out of range value */
                  printf("No Such Mode! \n");
                }
            }

            switch (cmd) {
            case '1':
                img_size.width   = VGA_WIDTH;
                img_size.height  = VGA_HEIGHT;
               break;
            case '2':
                img_size.width   = QVGA_WIDTH;
                img_size.height  = QVGA_HEIGHT;
               break;
            case '3':
                img_size.width   = _720P_WIDTH;
                img_size.height  = _720P_HEIGHT;
               break;
            case '4':
                img_size.width   = _1080P_WIDTH;
                img_size.height  = _1080P_HEIGHT;
               break;
            case '5':
                img_size.width   = QSXGA_WIDTH;
                img_size.height  = QSXGA_HEIGHT;
               break;
            case '6':
                img_size.width   = SXGA_WIDTH;
                img_size.height  = SXGA_HEIGHT;
               break;
            case '7':
                img_size.width   = XGA_WIDTH;
                img_size.height  = XGA_HEIGHT;
               break;
            default:
                img_size.width   = VGA_WIDTH;
                img_size.height  = VGA_HEIGHT;
            }

            /** Set configures to camera module */
            ret = set_streams_cfg(dev, img_size);
            break;
        case 't':
            /** Start Capture */
            ret = capture(dev);
           break;
        case 'p':
            /** Stop Capture */
            ret = flush(dev);
           break;
        case 'm':
            /** Get camera module supported configure */
            ret = metadata_transmit(dev, SIZE_CAPABILITIES);
           break;
        default:
            print_usage();
        }
    }

    device_close(dev);
    dev = NULL;

    return ret;
}
