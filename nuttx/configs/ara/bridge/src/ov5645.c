/*
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

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <debug.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_camera.h>
#include <nuttx/list.h>
#include <nuttx/wqueue.h>
#include <nuttx/gpio.h>
#include <nuttx/i2c.h>
#include <nuttx/util.h>
#include <arch/board/cdsi0_offs_def.h>
#include <arch/board/cdsi0_reg_def.h>
#include <arch/board/mipi_csi2.h>

/* OV5645 ID registers */
#define OV5645_ID_HIGH      0x300a
#define OV5645_ID_LOW       0x300b

#define REG_STREAM_ONOFF    0x4202
#define stream_on           0x00
#define stream_off          0x0f

#define OV5645_ID_H         (0x56)
#define OV5645_ID_L         (0x45)

#define L_SHIFT_BIT         8
#define R_SHIFT_BIT         8

/* Delay Time */
#define DELAY_10            10
#define DELAY_50            50
#define DELAY_5000          5000
#define DELAY_1000          1000

/* OV5645 power */
#define OV5645_RESET        7
#define OV5645_PWDN         8

/* OV5645 I2C address */
#define OV5645_I2C_ADDR     0x3c

#define OV5645_APB3_I2C0    0

/*************************************************
 * The difinitions of the NOT_SUPPORT and
 * SUPPORT were used by flag in the
 * function op_set_streams_cfg().
 *
 * NOT_SUPPORT means that requested configuration
 * isn’t supported and has been adjusted.
 *************************************************/
#define NOT_SUPPORT         0
#define SUPPORT             1

#define PADDING             0

#define HI_BYTE             1
#define LOW_BYTE            0
#define OV5645_REG_END      0xffff
#define ID_SIZE             2
#define MATA_DATA_SIZE      8
#define CAPB_SIZE           16

#define FRAME_NUMBER        6
#define STREAM              8

#define VIRTUAL_CHANNEL     0

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
#define MAX_WIDTH           2592
#define MAX_HEIGHT          1944

enum ov5645_mode_enum {
    ov5645_mode_MIN = 0,
    ov5645_init_mode_SXGA_1280_960 = 0,
    ov5645_mode_VGA_640_480 = 1,
    ov5645_mode_QVGA_320_240 = 2,
    ov5645_mode_720P_1280_720 = 3,
    ov5645_mode_1080P_1920_1080 = 4,
    ov5645_mode_QSXGA_2592_1944 = 5,
    ov5645_mode_XGA_1024_768 = 6,
    ov5645_mode_SXGA_1280_960 = 7,
    ov5645_mode_MAX = 8,
    ov5645_mode_INIT = 0xff, /*only for sensor init*/
};

/**
 * @brief camera device state
 */
enum ov5645_state {
    OV5645_STATE_OPEN,
    OV5645_STATE_CLOSED,
};

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

struct pixel_format {
    uint32_t    width;
    uint32_t    height;
    uint32_t    pixelformat;
};

/**
 * @brief private camera device information
 */
struct sensor_info {
    struct device *dev;
    struct i2c_dev_s *cam_i2c;
    enum ov5645_state state;
    struct streams_cfg_req *str_cfg_sup;
    struct streams_cfg_req *str_cfg_req;
    struct streams_cfg_ans *str_cfg_ans;
    struct capture_info *cap_info;
    struct metadata_info *mdata_info;
    struct pixel_format pix_fmt;
    struct cdsi_dev *cdsidev;
    uint8_t current_mode;
    uint8_t new_mode;

};

static uint8_t req_id;

/**
 * @brief Struct to store register and value for sensor read/write
 */
struct reg_val_tbl {
    uint16_t reg_num;
    uint8_t value;
};

/**
 * @brief ov5645 sensor init registers for SXGA
 */
struct reg_val_tbl ov5645_init_setting_SXGA_1280_960[] = {
    /* SVGA 1280*960 */
    /* initial setting, Sysclk = 56Mhz, MIPI 2 lane 224MBps */
    {0x3103, 0x11}, /* select PLL input clock */
    {0x3008, 0x82}, /* software reset */
    {0x3008, 0x42}, /* software standby */
    {0x3103, 0x03}, /* clo0xfrom, 0xpl,L */
    {0x3503, 0x07}, /* AGC manual, AEC manual */
    {0x3002, 0x1c}, /* system reset */
    {0x3006, 0xc3}, /* clock enable */
    {0x300e, 0x45}, /* MIPI 2 lane */
    {0x3017, 0x40}, /* Frex, CSK input, Vsync output */
    {0x3018, 0x00}, /* GPIO input */
    {0x302e, 0x0b},
    {0x3037, 0x13}, /* PLL */
    {0x3108, 0x01}, /* PLL */
    {0x3611, 0x06},
    {0x3612, 0xab},
    {0x3614, 0x50},
    {0x3618, 0x04},
    {0x3034, 0x18}, /* PLL, MIPI 8-bit mode */
    {0x3035, 0x21}, /* PLL */
    {0x3036, 0x70}, /* PLL */
    {0x3500, 0x00}, /* exposure = 0x100 */
    {0x3501, 0x01}, /* exposure */
    {0x3502, 0x00}, /* exposure */
    {0x350a, 0x00}, /* gain = 0x3f */
    {0x350b, 0x3f}, /* gain */
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3620, 0x33},
    {0x3621, 0xe0},
    {0x3622, 0x01},
    {0x3630, 0x2d},
    {0x3631, 0x00},
    {0x3632, 0x32},
    {0x3633, 0x52},
    {0x3634, 0x70},
    {0x3635, 0x13},
    {0x3636, 0x03},
    {0x3702, 0x6e},
    {0x3703, 0x52},
    {0x3704, 0xa0},
    {0x3705, 0x33},
    {0x3708, 0x66},
    {0x3709, 0x12},
    {0x370b, 0x61},
    {0x370c, 0xc3},
    {0x370f, 0x10},
    {0x3715, 0x08},
    {0x3717, 0x01},
    {0x371b, 0x20},
    {0x3731, 0x22},
    {0x3739, 0x70},
    {0x3901, 0x0a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3719, 0x86},
    {0x3800, 0x00}, /* HS = 0 */
    {0x3801, 0x00}, /* HS */
    {0x3802, 0x00}, /* VS = 6 */
    {0x3803, 0x06}, /* VS */
    {0x3804, 0x0a}, /* HW = 2623 */
    {0x3805, 0x3f}, /* HW */
    {0x3806, 0x07}, /* VH = 1949 */
    {0x3807, 0x9d}, /* VH */
    {0x3808, 0x05}, /* DVPHO = 1280 */
    {0x3809, 0x00}, /* DVPHO */
    {0x380a, 0x03}, /* DVPVO = 960 */
    {0x380b, 0xc0}, /* DVPVO */
    {0x380c, 0x07}, /* HTS = 1896 */
    {0x380d, 0x68}, /* HTS */
    {0x380e, 0x03}, /* VTS = 984 */
    {0x380f, 0xd8}, /* VTS */
    {0x3810, 0x00}, /* H OFF = 16 */
    {0x3811, 0x10}, /* H OFF */
    {0x3812, 0x00}, /* V OFF = 6 */
    {0x3813, 0x06}, /* V OFF */
    {0x3814, 0x31}, /* X INC */
    {0x3815, 0x31}, /* Y INC */
    {0x3820, 0x47}, /* flip on, V bin on */
    {0x3821, 0x07}, /* mirror on, H bin on */
    {0x3824, 0x01}, /* PLL */
    {0x3826, 0x03},
    {0x3828, 0x08},
    {0x3a02, 0x03}, /* nigt mode ceiling = 984 */
    {0x3a03, 0xd8}, /* nigt mode ceiling */
    {0x3a08, 0x01}, /* B50 */
    {0x3a09, 0xf8}, /* B50 */
    {0x3a0a, 0x01}, /* B60 */
    {0x3a0b, 0xa4}, /* B60 */
    {0x3a0e, 0x02}, /* max 50 */
    {0x3a0d, 0x02}, /* max 60 */
    {0x3a14, 0x03}, /* 50Hz max exposure = 984 */
    {0x3a15, 0xd8}, /* 50Hz max exposure */
    {0x3a18, 0x01}, /* gain ceiling = 31.5x */
    {0x3a19, 0xf8}, /* gain ceiling */
    /* 50Hz/60Hz auto detect */
    {0x3c01, 0x34},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c07, 0x07},
    {0x3c09, 0xc2},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},
    {0x3c01, 0x34},
    {0x4001, 0x02}, /* BLC start line */
    {0x4004, 0x02}, /* B0xline, 0xnu,mber */
    {0x4005, 0x18}, /* BLC update by gain change */
    {0x4300, 0x30}, /* YUV 422, YUYV */
    {0x4514, 0x00},
    {0x4520, 0xb0},
    {0x460b, 0x37},
    {0x460c, 0x20},
    /* MIPI timing */
    {0x4818, 0x01},
    {0x481d, 0xf0},
    {0x481f, 0x50},
    {0x4823, 0x70},
    {0x4831, 0x14},
    {0x4837, 0x10}, /* global timing */
    {0x5000, 0xa7}, /* Lenc/raw gamma/BPC/WPC/color interpolation on */
    {0x5001, 0x83}, /* SDE on, scale off, UV adjust off, color matrix/AWB on */
    {0x501d, 0x00},
    {0x501f, 0x00}, /* select ISP YUV 422 */
    {0x503d, 0x00},
    {0x505c, 0x30},
    /* AWB control */
    {0x5181, 0x59},
    {0x5183, 0x00},
    {0x5191, 0xf0},
    {0x5192, 0x03},
    /* AVG control */
    {0x5684, 0x10},
    {0x5685, 0xa0},
    {0x5686, 0x0c},
    {0x5687, 0x78},
    {0x5a00, 0x08},
    {0x5a21, 0x00},
    {0x5a24, 0x00},
    {0x3008, 0x02}, /* wake 0xfrom, 0xso,ftware standby */
    {0x3503, 0x00}, /* AGC auto, AEC auto */
    /* AWB control */
    {0x5180, 0xff},
    {0x5181, 0xf2},
    {0x5182, 0x00},
    {0x5183, 0x14},
    {0x5184, 0x25},
    {0x5185, 0x24},
    {0x5186, 0x09},
    {0x5187, 0x09},
    {0x5188, 0x0a},
    {0x5189, 0x75},
    {0x518a, 0x52},
    {0x518b, 0xea},
    {0x518c, 0xa8},
    {0x518d, 0x42},
    {0x518e, 0x38},
    {0x518f, 0x56},
    {0x5190, 0x42},
    {0x5191, 0xf8},
    {0x5192, 0x04},
    {0x5193, 0x70},
    {0x5194, 0xf0},
    {0x5195, 0xf0},
    {0x5196, 0x03},
    {0x5197, 0x01},
    {0x5198, 0x04},
    {0x5199, 0x12},
    {0x519a, 0x04},
    {0x519b, 0x00},
    {0x519c, 0x06},
    {0x519d, 0x82},
    {0x519e, 0x38},
    /* matrix */
    {0x5381, 0x1e},
    {0x5382, 0x5b},
    {0x5383, 0x08},
    {0x5384, 0x0b},
    {0x5385, 0x84},
    {0x5386, 0x8f},
    {0x5387, 0x82},
    {0x5388, 0x71},
    {0x5389, 0x11},
    {0x538a, 0x01},
    {0x538b, 0x98},
    /* CIP */
    {0x5300, 0x08}, /* sharpen MT th1 */
    {0x5301, 0x30}, /* sharpen MT th2 */
    {0x5302, 0x10}, /* sharpen MT off1 */
    {0x5303, 0x00}, /* sharpen MT off2 */
    {0x5304, 0x08}, /* DNS th1 */
    {0x5305, 0x30}, /* DNS th2 */
    {0x5306, 0x08}, /* DNS off1 */
    {0x5307, 0x16}, /* DNS off2 */
    {0x5309, 0x08}, /* sharpen TH th1 */
    {0x530a, 0x30}, /* sharpen TH th2 */
    {0x530b, 0x04}, /* sharpen TH off1 */
    {0x530c, 0x06}, /* sharpen TH off2 */
    /* Gamma */
    {0x5480, 0x01}, /* bias on */
    {0x5481, 0x0e}, /* Y yst 00 */
    {0x5482, 0x18},
    {0x5483, 0x2b},
    {0x5484, 0x52},
    {0x5485, 0x65},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x87},
    {0x5489, 0x91},
    {0x548a, 0x9a},
    {0x548b, 0xaa},
    {0x548c, 0xb8},
    {0x548d, 0xcd},
    {0x548e, 0xdd},
    {0x548f, 0xea}, /* Y yst 0E */
    {0x5490, 0x1d}, /* Y yst 0F */
    /* SDE */
    {0x5580, 0x06},
    {0x5583, 0x40},
    {0x5584, 0x30},
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},
    /* LENC */
    {0x5800, 0x3f},
    {0x5801, 0x16},
    {0x5802, 0x0e},
    {0x5803, 0x0d},
    {0x5804, 0x17},
    {0x5805, 0x3f},
    {0x5806, 0x0b},
    {0x5807, 0x06},
    {0x5808, 0x04},
    {0x5809, 0x04},
    {0x580a, 0x06},
    {0x580b, 0x0b},
    {0x580c, 0x09},
    {0x580d, 0x03},
    {0x580e, 0x00},
    {0x580f, 0x00},
    {0x5810, 0x03},
    {0x5811, 0x08},
    {0x5812, 0x0a},
    {0x5813, 0x03},
    {0x5814, 0x00},
    {0x5815, 0x00},
    {0x5816, 0x04},
    {0x5817, 0x09},
    {0x5818, 0x0f},
    {0x5819, 0x08},
    {0x581a, 0x06},
    {0x581b, 0x06},
    {0x581c, 0x08},
    {0x581d, 0x0c},
    {0x581e, 0x3f},
    {0x581f, 0x1e},
    {0x5820, 0x12},
    {0x5821, 0x13},
    {0x5822, 0x21},
    {0x5823, 0x3f},
    {0x5824, 0x68},
    {0x5825, 0x28},
    {0x5826, 0x2c},
    {0x5827, 0x28},
    {0x5828, 0x08},
    {0x5829, 0x48},
    {0x582a, 0x64},
    {0x582b, 0x62},
    {0x582c, 0x64},
    {0x582d, 0x28},
    {0x582e, 0x46},
    {0x582f, 0x62},
    {0x5830, 0x60},
    {0x5831, 0x62},
    {0x5832, 0x26},
    {0x5833, 0x48},
    {0x5834, 0x66},
    {0x5835, 0x44},
    {0x5836, 0x64},
    {0x5837, 0x28},
    {0x5838, 0x66},
    {0x5839, 0x48},
    {0x583a, 0x2c},
    {0x583b, 0x28},
    {0x583c, 0x26},
    {0x583d, 0xae},
    {0x5025, 0x00},
    {0x3a0f, 0x38}, /* AEC in H */
    {0x3a10, 0x30}, /* AEC in L */
    {0x3a1b, 0x38}, /* AEC out H */
    {0x3a1e, 0x30}, /* AEC out L */
    {0x3a11, 0x70}, /* control zone H */
    {0x3a1f, 0x18}, /* control zone L */
    {0x3008, 0x02}, /* software enable */

    {OV5645_REG_END, 0x00}, /* END MARKER */
};

/**
 * @brief ov5645 sensor registers for 30fps VGA
 */
struct reg_val_tbl ov5645_setting_30fps_VGA_640_480[] = {
    {0x3618, 0x00},
    {0x3035, 0x11},
    {0x3036, 0x46},
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3708, 0x64},
    {0x370c, 0xc3},
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x04},
    {0x3804, 0x0a},
    {0x3805, 0x3f},
    {0x3806, 0x07},
    {0x3807, 0x9b},
    {0x3808, 0x02},
    {0x3809, 0x80},
    {0x380a, 0x01},
    {0x380b, 0xe0},
    {0x380c, 0x07},
    {0x380d, 0x68},
    {0x380e, 0x04},
    {0x380f, 0x38},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x06},
    {0x3820, 0x41},
    {0x3821, 0x07},
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0x0e},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0e, 0x03},
    {0x3a0d, 0x04},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},
    {0x4004, 0x02},
    {0x4005, 0x18},
    {0x4837, 0x16},
    {0x3503, 0x00},

    {OV5645_REG_END, 0x00}, /* END MARKER */
};

/**
 * @brief ov5645 sensor registers for 30fps QVGA
 */
struct reg_val_tbl ov5645_setting_30fps_QVGA_320_240[] = {

    /* TBD */

    {OV5645_REG_END, 0x00}, /* END MARKER */
};
#if 1
/* video moide size: 1280*720. Below table is form ov5645 sample code */
struct reg_val_tbl ov5645_setting_30fps_720p_1280_720[] = {
    //Sysclk = 42Mhz, MIPI 2 lane 168MBps
    //0x3612, 0xa9,
    {0x3618, 0x00},
    {0x3035, 0x21},
    {0x3036, 0x54},
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3708, 0x66},
    {0x370c, 0xc3},
    {0x3803, 0xfa}, // VS L
    {0x3806, 0x06}, // VH = 1705
    {0x3807, 0xa9}, // VH
    {0x3808, 0x05}, // DVPHO = 1280
    {0x3809, 0x00}, // DVPHO
    {0x380a, 0x02}, // DVPVO = 720
    {0x380b, 0xd0}, // DVPVO
    {0x380c, 0x07}, // HTS = 1892
    {0x380d, 0x64}, // HTS
    {0x380e, 0x02}, // VTS = 740
    {0x380f, 0xe4}, // VTS
    {0x3814, 0x31}, // X INC
    {0x3815, 0x31}, // X INC
    #ifdef OV5645_mirror
    #ifdef OV5645_flip
    {0x3820, 0x47}, // flip on, V bin on
    {0x3821, 0x07}, // mirror on, H bin on
    #else
    {0x3820, 0x41}, // flip off, V bin on
    {0x3821, 0x07}, // mirror on, H bin on
    #endif
    #else
    #ifdef OV5645_flip
    {0x3820, 0x47}, // flip on, V bin on
    {0x3821, 0x01}, // mirror off, H bin on
    #else
    {0x3820, 0x41}, // flip off, V bin on
    {0x3821, 0x01}, // mirror off, H bin on
    #endif
    #endif
    {0x3a02, 0x02}, // night mode ceiling = 740
    {0x3a03, 0xe4}, // night mode ceiling
    {0x3a08, 0x00}, // B50 = 222
    {0x3a09, 0xde}, // B50
    {0x3a0a, 0x00}, // B60 = 185
    {0x3a0b, 0xb9}, // B60
    {0x3a0e, 0x03}, // max 50
    {0x3a0d, 0x04}, // max 60
    {0x3a14, 0x02}, // max 50hz exposure = 3/100
    {0x3a15, 0x9a}, // max 50hz exposure
    {0x3a18, 0x01}, // max gain = 31.5x
    {0x3a19, 0xf8}, // max gain
    {0x4004, 0x02}, // BLC line number
    {0x4005, 0x18}, // BLC update by gain change
    {0x4837, 0x16}, // MIPI global timing
    {0x3503, 0x00}, // AGC/AEC on
 };
#else
/**
 * @brief ov5645 sensor registers for 30fps 720p
 */
struct reg_val_tbl ov5645_setting_30fps_720p_1280_720[] = {
    {0x3618, 0x00},
    {0x3035, 0x11},
    {0x3036, 0x54},
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3708, 0x64},
    {0x370c, 0xc3},
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0xfa},
    {0x3804, 0x0a},
    {0x3805, 0x3f},
    {0x3806, 0x06},
    {0x3807, 0xa9},
    {0x3808, 0x05},
    {0x3809, 0x00},
    {0x380a, 0x02},
    {0x380b, 0xd0},
    {0x380c, 0x07},
    {0x380d, 0x64},
    {0x380e, 0x02},
    {0x380f, 0xe4},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x04},
    {0x3820, 0x41},
    {0x3821, 0x07},
    {0x3a02, 0x02},
    {0x3a03, 0xe4},
    {0x3a08, 0x01},
    {0x3a09, 0xbc},
    {0x3a0a, 0x01},
    {0x3a0b, 0x72},
    {0x3a0e, 0x01},
    {0x3a0d, 0x02},
    {0x3a14, 0x02},
    {0x3a15, 0xe4},
    {0x4004, 0x02},
    {0x4005, 0x18},
    {0x4837, 0x16},
    {0x3503, 0x00},
    {0x3008, 0x02},

    {OV5645_REG_END, 0x00}, /* END MARKER */
};
#endif
/**
 * @brief ov5645 sensor registers for 30fps 1080p
 */
struct reg_val_tbl ov5645_setting_30fps_1080p_1920_1080[] = {
    {0x3612, 0xab},
    {0x3614, 0x50},
    {0x3618, 0x04},
    {0x3035, 0x21},
    {0x3036, 0x70},
    {0x3600, 0x08},
    {0x3601, 0x33},
    {0x3708, 0x63},
    {0x370c, 0xc0},
    {0x3800, 0x01},
    {0x3801, 0x50},
    {0x3802, 0x01},
    {0x3803, 0xb2},
    {0x3804, 0x08},
    {0x3805, 0xef},
    {0x3806, 0x05},
    {0x3807, 0xf1},
    {0x3808, 0x07},
    {0x3809, 0x80},
    {0x380a, 0x04},
    {0x380b, 0x38},
    {0x380c, 0x09},
    {0x380d, 0xc4},
    {0x380e, 0x04},
    {0x380f, 0x60},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x04},
    {0x3814, 0x11},
    {0x3815, 0x11},
    {0x3820, 0x41},
    {0x3821, 0x07},
    {0x3a02, 0x04},
    {0x3a03, 0x90},
    {0x3a08, 0x01},
    {0x3a09, 0xf8},
    {0x3a0a, 0x01},
    {0x3a0b, 0xf8},
    {0x3a0e, 0x02},
    {0x3a0d, 0x02},
    {0x3a14, 0x04},
    {0x3a15, 0x90},
    {0x3a18, 0x00},
    {0x4004, 0x02},
    {0x4005, 0x18},
    {0x4837, 0x10},
    {0x3503, 0x00},

    {OV5645_REG_END, 0x00}, /* END MARKER */
};

/**
 * @brief ov5645 sensor registers for 15fps QSXGA
 */
struct reg_val_tbl ov5645_setting_15fps_QSXGA_2592_1944[] = {
    {0x3820, 0x40},
    {0x3821, 0x06}, /*disable flip*/
    {0x3035, 0x21},
    {0x3036, 0x54},
    {0x3c07, 0x07},
    {0x3c09, 0xc2},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},
    {0x3820, 0x40},
    {0x3821, 0x06},
    {0x3814, 0x11},
    {0x3815, 0x11},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x00},
    {0x3804, 0x0a},
    {0x3805, 0x3f},
    {0x3806, 0x07},
    {0x3807, 0x9f},
    {0x3808, 0x0a},
    {0x3809, 0x20},
    {0x380a, 0x07},
    {0x380b, 0x98},
    {0x380c, 0x0b},
    {0x380d, 0x1c},
    {0x380e, 0x07},
    {0x380f, 0xb0},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x04},
    {0x3618, 0x04},
    {0x3612, 0xab},
    {0x3708, 0x21},
    {0x3709, 0x12},
    {0x370c, 0x00},
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0e, 0x03},
    {0x3a0d, 0x04},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},
    {0x4001, 0x02},
    {0x4004, 0x06},
    {0x4713, 0x03},
    {0x4407, 0x04},
    {0x460b, 0x35},
    {0x460c, 0x22},
    {0x3824, 0x02},
    {0x5001, 0x83},

    {OV5645_REG_END, 0x00}, /* END MARKER */
};

/**
 * @brief ov5645 sensor registers for 30fps XGA
 */
struct reg_val_tbl ov5645_setting_30fps_XGA_1024_768[] = {
    {0x3618, 0x00},
    {0x3035, 0x11},
    {0x3036, 0x70},
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3708, 0x64},
    {0x370c, 0xc3},
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x06},
    {0x3804, 0x0a},
    {0x3805, 0x3f},
    {0x3806, 0x07},
    {0x3807, 0x9d},
    {0x3808, 0x04},
    {0x3809, 0x00},
    {0x380a, 0x03},
    {0x380b, 0x00},
    {0x380c, 0x07},
    {0x380d, 0x68},
    {0x380e, 0x03},
    {0x380f, 0xd8},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x06},
    {0x3820, 0x41},
    {0x3821, 0x07},
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0xf8},
    {0x3a0a, 0x01},
    {0x3a0b, 0xa4},
    {0x3a0e, 0x02},
    {0x3a0d, 0x02},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},
    {0x4004, 0x02},
    {0x4005, 0x18},
    {0x4837, 0x16},
    {0x3503, 0x00},

    {OV5645_REG_END, 0x00}, /* END MARKER */
};

#if 1
/* preview moide size: 1280*960. Below table is form ov5645 sample code */
struct reg_val_tbl ov5645_setting_30fps_SXGA_1280_960[] = {
    // Sysclk = 56Mhz, MIPI 2 lane 224MBps
    //0x3612, 0xa9,
    {0x3618, 0x00},
    {0x3035, 0x21}, // PLL
    {0x3036, 0x70}, // PLL
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3708, 0x66},
    {0x370c, 0xc3},
    {0x3803, 0x06}, // VS L
    {0x3806, 0x07}, // VH = 1949
    {0x3807, 0x9d}, // VH
    {0x3808, 0x05}, // DVPHO = 1280
    {0x3809, 0x00}, // DVPHO
    {0x380a, 0x03}, // DVPVO = 960
    {0x380b, 0xc0}, // DVPVO
    {0x380c, 0x07}, // HTS = 1896
    {0x380d, 0x68}, // HTS
    {0x380e, 0x03}, // VTS = 984
    {0x380f, 0xd8}, // VTS
    {0x3814, 0x31}, // X INC
    {0x3815, 0x31}, // Y INC
    #ifdef OV5645_mirror
    #ifdef OV5645_flip
    {0x3820, 0x47}, // flip on, V bin on
    {0x3821, 0x07}, // mirror on, H bin on
    #else
    {0x3820, 0x41}, // flip off, V bin on
    {0x3821, 0x07}, // mirror on, H bin on
    #endif
    #else
    #ifdef OV5645_flip
    {0x3820, 0x47}, // flip on, V bin on
    {0x3821, 0x01}, // mirror off, H bin on
    #else
    {0x3820, 0x41}, // flip off, V bin on
    {0x3821, 0x01}, // mirror off, H bin on
    #endif
    #endif
    {0x3a02, 0x07}, // night mode ceiling = 8/120
    {0x3a03, 0xb0}, // night mode ceiling
    {0x3a08, 0x01}, // B50
    {0x3a09, 0x27}, // B50
    {0x3a0a, 0x00}, // B60
    {0x3a0b, 0xf6}, // B60
    {0x3a0e, 0x03}, // max 50
    {0x3a0d, 0x04}, // max 60
    {0x3a14, 0x08}, // 50Hz max exposure = 7/100
    {0x3a15, 0x11}, // 50Hz max exposure
    {0x3a18, 0x01}, // max gain = 31.5x
    {0x3a19, 0xf8}, // max gain
    {0x4004, 0x02}, // BLC line number
    {0x4005, 0x18}, // BLC update by gain change
    {0x4837, 0x10}, // MIPI global timing
    {0x3503, 0x00}, // AGC/AEC on

    {OV5645_REG_END, 0x00}, /* END MARKER */
};
#else
/**
 * @brief ov5645 sensor registers for 30fps SXGA
 */
struct reg_val_tbl ov5645_setting_30fps_SXGA_1280_960[] = {
    {0x3618, 0x00},
    {0x3035, 0x21},
    {0x3036, 0x70},
    {0x3600, 0x09},
    {0x3601, 0x43},
    {0x3708, 0x66},
    {0x370c, 0xc3},
    {0x3803, 0x06},
    {0x3806, 0x07},
    {0x3807, 0x9d},
    {0x3808, 0x05},
    {0x3809, 0x00},
    {0x380a, 0x03},
    {0x380b, 0xc0},
    {0x380c, 0x07},
    {0x380d, 0x68},
    {0x380e, 0x03},
    {0x380f, 0xd8},
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3820, 0x41},
    {0x3821, 0x07},
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0xf8},
    {0x3a0a, 0x01},
    {0x3a0b, 0xa4},
    {0x3a0e, 0x02},
    {0x3a0d, 0x02},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x4004, 0x02},
    {0x4005, 0x18},
    {0x4837, 0x10},
    {0x3503, 0x00},
    {0x4300, 0x32},
    {0x4202, 0x00},

    {OV5645_REG_END, 0x00}, /* END MARKER */
};
#endif

/**
 * @brief ov5645 sensor mode
 */
struct ov5645_mode_info {
    enum ov5645_mode_enum mode_enum;
    int width;
    int height;
    enum ov5645_pixel_format img_fmt;
    struct reg_val_tbl *regs;
};

struct ov5645_mode_info ov5645_mode_settings[] = {
    /* VGA - init 1280*960 */
    {
        .mode_enum  = ov5645_init_mode_SXGA_1280_960,
        .width      = VGA_WIDTH,
        .height     = VGA_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_init_setting_SXGA_1280_960,
    },
    /* VGA - 640*480 */
    {
        .mode_enum  = ov5645_mode_VGA_640_480,
        .width      = VGA_WIDTH,
        .height     = VGA_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_setting_30fps_VGA_640_480,
    },
    /* QVGA - 320*240 */
    {
        .mode_enum  = ov5645_mode_QVGA_320_240,
        .width      = QVGA_WIDTH,
        .height     = QVGA_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_init_setting_SXGA_1280_960,
    },
    /* 720p - 1280*720 */
    {
        .mode_enum  = ov5645_mode_720P_1280_720,
        .width      = _720P_WIDTH,
        .height     = _720P_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_setting_30fps_720p_1280_720,
    },
    /* 1080p - 1920*1080 */
    {
        .mode_enum  = ov5645_mode_1080P_1920_1080,
        .width      = _1080P_WIDTH,
        .height     = _1080P_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_setting_30fps_1080p_1920_1080,
    },
    /* QSXGA - 2592*1944 */
    {
        .mode_enum  = ov5645_mode_QSXGA_2592_1944,
        .width      = QSXGA_WIDTH,
        .height     = QSXGA_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_setting_15fps_QSXGA_2592_1944,
    },
    /* SXGA - 1280*960 */
    {
        .mode_enum  = ov5645_mode_SXGA_1280_960,
        .width      = SXGA_WIDTH,
        .height     = SXGA_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_setting_30fps_SXGA_1280_960,
    },
    /* XGA - 1024*768 */
    {
        .mode_enum  = ov5645_mode_XGA_1024_768,
        .width      = XGA_WIDTH,
        .height     = XGA_HEIGHT,
        .img_fmt    = YCbCr422,
        .regs       = ov5645_setting_30fps_XGA_1024_768,
    }
};
#define N_WIN_SIZES (ARRAY_SIZE(ov5645_mode_settings))

/**
 * @brief i2c read for camera sensor (It reads a single byte)
 * @param dev Pointer to structure of i2c device data
 * @param addr Address of i2c to read
 * @param data Pointer of data to read in
 * @return zero for success or non-zero on any faillure
 */
static int data_read(struct i2c_dev_s *dev, uint16_t addr, uint8_t *data)
{
    uint8_t cmd[2] = {0x00, 0x00};
    uint8_t buf = 0x00;
    int ret = 0;
    struct i2c_msg_s msg[] = {
        {
            .addr = OV5645_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 2,
        }, {
            .addr = OV5645_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = &buf,
            .length = 1,
        }
    };

    cmd[0] = (addr >> R_SHIFT_BIT) & 0xFF;
    cmd[1] = addr & 0xFF;

    ret = I2C_TRANSFER(dev, msg, 2);
    if (ret != OK) {
        return -EIO;
    }

    *data = buf;
    return 0;
}

/**
 * @brief i2c write for camera sensor (It writes a single byte)
 * @param dev Pointer to structure of i2c device data
 * @param addr Address of i2c to write
 * @param data Data to write
 * @return zero for success or non-zero on any faillure
 */
int data_write(struct i2c_dev_s *dev, uint16_t addr, uint8_t data)
{
    uint8_t cmd[3] = {0x00, 0x00, 0x00};
    int ret = 0;
    struct i2c_msg_s msg[] = {
        {
            .addr = OV5645_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 3,
        },
    };

    cmd[0] = (addr >> R_SHIFT_BIT) & 0xFF;
    cmd[1] = addr & 0xFF;
    cmd[2] = data;

    ret = I2C_TRANSFER(dev, msg, 1);
    if (ret != OK) {
        return -EIO;
    }

    return 0;
}

/**
 * @brief i2c write for camera sensor (It writes array)
 * @param dev Pointer to structure of i2c device data
 * @param vals Address and values of i2c to write
 * @return zero for success or non-zero on any faillure
 */
static int data_write_array(struct i2c_dev_s *dev, struct reg_val_tbl *vals)
{
    int ret = 0;

    while (vals->reg_num < OV5645_REG_END) {
        ret = data_write(dev, vals->reg_num, vals->value);
        if (ret) {
           printf("[%s]ERROR! Fails to fill sensor registers.\n", __func__);
           return ret;
        }
        usleep(DELAY_50);
        vals++;
    }

    return ret;
}

/**
 * @brief ov5645 sensor initialization function
 * @param vals Pointer to mode of sensor to initialize
 * @return zero for success or non-zero on any faillure
 */
static int set_mode(struct cdsi_dev *cdsidev, struct reg_val_tbl *vals,
                    struct i2c_dev_s *cam_i2c,
                    enum ov5645_mode_enum current_mode,
                    enum ov5645_mode_enum new_mode)
{
    int ret = 0;
    uint32_t rdata1 = 0;

    printf("[%s]+\n", __func__);

    if (!cdsidev) {
        printf("[%s]ERROR! cdsidev is invaild.\n", __func__);
        return -EINVAL;
    }

    if (!cam_i2c) {
        printf("[%s]ERROR! cam_i2c is invaild.\n", __func__);
        return -EIO;
    }

    printf("[%s]current_mode: %d, new_mode: %d\n",
           __func__, current_mode, new_mode);

    if ((current_mode == ov5645_init_mode_SXGA_1280_960) &&
       (new_mode == ov5645_init_mode_SXGA_1280_960))
    {
        printf("[%s]init mode... \n", __func__);

        /* Do ov5645 init mode settings */
#if 0
        /* setup CDSIRX internal state */
        //cdsi_write(cdsidev, CDSI0_CDSIRX_SYSTEM_INIT_OFFS,
        //           CDSI0_CDSIRX_SYSTEM_CLEAR_VAL);

        /* Enable CDSIRX */
        cdsi_write(cdsidev, CDSI0_CDSIRX_CLKEN_OFFS, CDSIRX_CLKEN_VAL);

        /* Enable RX Bridge */
        cdsi_write(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS, AL_RX_BRG_MODE_VAL);
#endif
        /* Start CDSIRX */
        cdsi_write(cdsidev, CDSI0_CDSIRX_START_OFFS, CDSI0_CDSIRX_START_VAL);

        /* Wait Line Initialization finish */
        rdata1 = cdsi_read(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);

        /* setup sensor registers */
        ret = data_write_array(cam_i2c, vals);
        if (ret){
            printf("[%s]ERROR! Fails to fill sensor registers.\n", __func__);
            ret = -EIO;
            return ret;
        }

        printf("[%s]Wait Line Initialization...\n", __func__);
        while ((rdata1 &
        CDSI0_CDSIRX_LPRX_STATE_INT_STAT_LINEINITDONE_MASK) == 0x0) {
            rdata1 = cdsi_read(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
            usleep(DELAY_10);
        }
        printf("[%s]Second LPRX_STATE_INT: %d\n", __func__, rdata1);

        cdsi_write(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS,
                   CDSI0_CDSIRX_LPRX_STATE_INT_STAT_VAL);

        cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS,
                   CDSIRX_DSI_LPTX_MODE_VAL);

        cdsi_write(cdsidev, CDSI0_CDSIRX_ADDRESS_CONFIG_OFFS,
                   CDSI0_CDSIRX_ADDRESS_CONFIG_VAL);
    }
    else if (current_mode == new_mode)
    {
        printf("[%s]the same mode... \n", __func__);
    }
    else
    {
        printf("[%s]switch mode... \n", __func__);

//        deinit_csi_rx(cdsidev);// disable clk
//        cdsidev = init_csi_rx(CDSI0, CDSI_RX); //enable clk

        /* Start CDSIRX */
        cdsi_write(cdsidev, CDSI0_CDSIRX_START_OFFS, CDSI0_CDSIRX_START_VAL);

        /* Wait Line Initialization finish */
        rdata1 = cdsi_read(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);

        /* setup sensor registers */
        ret = data_write_array(cam_i2c, vals);
        if (ret){
            printf("[%s]ERROR! Fails to fill sensor registers.\n", __func__);
            ret = -EIO;
            return ret;
        }

#if 1 //bsq test +
        usleep(DELAY_5000);
        /* Start stream */
        ret = data_write(cam_i2c, REG_STREAM_ONOFF, stream_on);
        if (ret) {
            ret = -EIO;
            return ret;
        }
        usleep(DELAY_10);
#endif //bsq test -

        printf("[%s]Wait Line Initialization...\n", __func__);

        //mipi_csi2_get_error(cdsidev);// MIPI CSI-2 debug...

        while ((rdata1 & CDSI0_CDSIRX_LPRX_STATE_INT_STAT_LINEINITDONE_MASK)
               == 0x0)
        {
            rdata1 = cdsi_read(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
            usleep(DELAY_10);
        }
        printf("[%s]Second LPRX_STATE_INT: %d\n", __func__, rdata1);

        cdsi_write(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS,
                   CDSI0_CDSIRX_LPRX_STATE_INT_STAT_VAL);

        cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS,
                   CDSIRX_DSI_LPTX_MODE_VAL);

        cdsi_write(cdsidev, CDSI0_CDSIRX_ADDRESS_CONFIG_OFFS,
                   CDSI0_CDSIRX_ADDRESS_CONFIG_VAL);

        /* update current mode */
        current_mode = new_mode;
    }

    printf("[%s]-\n", __func__);
    return ret;
}

/**
 * @brief Power up camera module
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int op_power_up(struct device *dev)
{
    struct sensor_info *info = NULL;

    if (!dev) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state == OV5645_STATE_OPEN) {
        return -EPERM;
    }

    /* sensor power-on */
    gpio_direction_out(OV5645_PWDN, 0); /* shutdown -> L */
    gpio_direction_out(OV5645_RESET, 0); /* reset -> L */
    usleep(DELAY_5000);

    gpio_direction_out(OV5645_PWDN, 1); /* shutdown -> H */
    usleep(DELAY_1000);

    gpio_direction_out(OV5645_RESET, 1); /* reset -> H */
    usleep(DELAY_1000);

    return 0;
}

/**
 * @brief Power down camera module
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int op_power_down(struct device *dev)
{
    struct sensor_info *info = NULL;

    if (!dev) {
       return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    /* sensor power off */
    gpio_direction_out(OV5645_PWDN, 0); /* shutdown -> L */
    usleep(DELAY_1000);

    gpio_direction_out(OV5645_RESET, 0); /* reset -> L */
    usleep(DELAY_1000);

    return 0;
}

/**
 * @brief Get capabilities of camera module
 * @param dev Pointer to structure of device data
 * @param capabilities Pointer that will be stored Camera Module capabilities.
 * @return 0 on success, negative errno on error
 */
static int op_capabilities(struct device *dev, uint32_t *size,
                               uint8_t *capabilities)
{
    struct sensor_info *info = NULL;

    if (!dev || !capabilities || !size) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    /* init capabilities [Fill in fake value]*/
    capabilities[0] = CAP_METADATA_GREYBUS;
    capabilities[0] |= CAP_METADATA_MIPI;
    capabilities[0] |= CAP_STILL_IMAGE;
    capabilities[0] |= CAP_JPEG;

    *size = sizeof(uint32_t);
    return 0;
}

/**
 * @brief Get required data size of camera module information
 * @param dev Pointer to structure of device data
 * @param capabilities Pointer that will be stored Camera Module capabilities.
 * @return 0 on success, negative errno on error
 */
static int op_get_required_size(struct device *dev, uint8_t operation,
                                uint16_t *size)
{
    struct sensor_info *info = NULL;
    int ret = 0;

    if (!dev || !size) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    switch (operation) {
    case SIZE_CAPABILITIES:
         *size = CAPB_SIZE;
       break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

/**
 * @brief Get camera module support stream cofigurations
 * @param dev Pointer to structure of device data
 * @param num_streams Number of streams
 * @param config Pointer to structure of streams configuration
 * @return 0 on success, negative errno on error
 */
static int get_support_mode(struct cdsi_dev *cdsidev,
                            struct streams_cfg_req *sup_modes)
{
    uint8_t i;

    /* get supported modes */
    for(i = 1; i < (N_WIN_SIZES-1); i++)
    {
        sup_modes[i].width = (uint16_t)ov5645_mode_settings[i].width;
        sup_modes[i].height = (uint16_t)ov5645_mode_settings[i].height;
        sup_modes[i].format = (uint16_t)ov5645_mode_settings[i].img_fmt;
        sup_modes[i].padding = PADDING;
    }

    return 0;
}

/**
 * @brief Set streams configuration to camera module
 * @param dev Pointer to structure of device data
 * @param num_streams Number of streams
 * @param config Pointer to structure of streams configuration
 * @return 0 on success, negative errno on error
 */
static int op_set_streams_cfg(struct device *dev, uint16_t *num_streams,
                           uint16_t *flags, struct streams_cfg_req *config,
                           struct streams_cfg_ans *answer)
{
    struct sensor_info *info = NULL;
    uint8_t i;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    /* Filter out the data from host request */
    for(i=1; i < (N_WIN_SIZES-1); i++) {
        if(((config->width) == (info->str_cfg_sup[i].width)) &&
            ((config->height) == (info->str_cfg_sup[i].height)) &&
            ((config->format) == (info->str_cfg_sup[i].format))) {

            answer->width = info->str_cfg_sup[i].width;
            answer->height = info->str_cfg_sup[i].height;
            answer->format = info->str_cfg_sup[i].format;

            info->cdsidev->v_channel = mipi_csi2_get_virtual_channel(info->cdsidev);
            info->cdsidev->datatype = mipi_csi2_get_datatype(info->cdsidev);
            //info->max_size = ;

            answer->virtual_channel = info->cdsidev->v_channel;
            answer->data_type = info->cdsidev->datatype;
            //answer->max_size = info->max_size;

            info->new_mode = i;

#if 1 //For debugging
            printf("[%s]info->cdsidev: 0x%x\n",
                   __func__, info->cdsidev);

            printf("[%s]answer->virtual_channel: 0x%x\n",
                   __func__, answer->virtual_channel);

            printf("[%s]answer->data_type: 0x%x\n",
                   __func__, answer->data_type);

            printf("[%s]index:%d, (w,h)=(%d,%d)\n", __func__, i,
                   ov5645_mode_settings[i].width,
                   ov5645_mode_settings[i].height);

            printf("[%s]info->current_mode: %d, info->new_mode: %d\n",
                   __func__, info->current_mode, info->new_mode);
 #endif

            if (!set_mode(info->cdsidev, ov5645_mode_settings[i].regs,
                 info->cam_i2c, info->current_mode, info->new_mode)) {
                *flags = SUPPORT;
                break;
            } else {
                /*
                * NOT_SUPPORT means that requested configuration isn’t supported
                * and has been adjusted.
                */
                *flags = NOT_SUPPORT;
                return -EINVAL;
            }
        }
        *flags = SUPPORT;
    }
    return 0;
}

/**
 * @brief Start the camera capture
 * @param dev Pointer to structure of device data
 * @param capt_info Capture parameters
 * @return 0 on success, negative errno on error
 */
static int op_capture(struct device *dev, struct capture_info *capt_info)
{
    struct sensor_info *info = NULL;
    int ret = 0;

    if (!dev || !device_get_private(dev) ) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    if (!(info->cam_i2c)) {
        return -EIO;
    }

    /* Start stream */
    ret = data_write(info->cam_i2c, REG_STREAM_ONOFF, stream_on);

    req_id++;
    return ret;
}

/**
 * @brief stop stream
 * @param dev The pointer to structure of device data
 * @param request_id The request id set by capture
 * @return 0 for success, negative errno on error.
 */
static int op_flush(struct device *dev, uint32_t *request_id)
{
    struct sensor_info *info = NULL;
    int ret = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    if (!(info->cam_i2c)) {
        return -EIO;
    }

    /* Stop stream */
    ret = data_write(info->cam_i2c, REG_STREAM_ONOFF, stream_off);

    req_id = 0;
    *request_id = req_id;
    return ret;
}

/**
 * @brief Get camera meta data
 * @param dev The pointer to structure of device data
 * @param meta_data Pointer to Meta-data block
 * @return 0 for success, negative errno on error.
 */
static int op_get_meta_data(struct device *dev, struct metadata_info *meta_data)
{
    struct sensor_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != OV5645_STATE_OPEN) {
        return -EPERM;
    }

    meta_data->request_id   =  info->mdata_info->request_id;
    meta_data->frame_number =  info->mdata_info->frame_number;
    meta_data->stream       =  info->mdata_info->stream;
    meta_data->padding      =  info->mdata_info->padding;
    meta_data->data         =  info->mdata_info->data;

    return 0;
}


/**
 * @brief Open camera device
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int ov5645_dev_open(struct device *dev)
{
    struct sensor_info *info = NULL;
    enum ov5645_pixel_format ov5645_out_fmt = YUV422;
    uint8_t id[ID_SIZE] = {0, 0};
    uint8_t m_data[MATA_DATA_SIZE];
    uint8_t ov5645_datatype = 0;
    int ret = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if(info->state == OV5645_STATE_OPEN) {
        return -EBUSY;
    }

    /* === get support modes === */
    info->str_cfg_sup = zalloc(N_WIN_SIZES * sizeof(struct streams_cfg_req));
    if (info->str_cfg_sup == NULL) {
        ret = -ENOMEM;
        goto err_free_info;
    }

    ret = get_support_mode(info->cdsidev, info->str_cfg_sup);
    if (ret) {
        ret = -ENOMEM;
        goto err_free_support;
    }

    /* === power on ov5645 sensor and get sensor ID === */

    /* power on sensor */
    ret = op_power_up(info->dev);
    if (ret) {
        ret = -EIO;
        goto err_free_support;
    }

    /* initialize I2C */
    info->cam_i2c = NULL;
    info->cam_i2c = up_i2cinitialize(OV5645_APB3_I2C0);
    if (!(info->cam_i2c)) {
        ret = -EIO;
        goto err_power_down;
    }

    /* get sensor id (high) */
    ret = data_read(info->cam_i2c, OV5645_ID_HIGH, &id[HI_BYTE]);
    if (ret){
        goto err_free_i2c;
    }

    if (id[HI_BYTE] != OV5645_ID_H) {
        ret = -ENODEV;
        goto err_free_i2c;
    }

    /* get sensor id (low) */
    ret = data_read(info->cam_i2c, OV5645_ID_LOW, &id[LOW_BYTE]);
    if (ret) {
        goto err_free_i2c;
    }

    if (id[LOW_BYTE] != OV5645_ID_L){
        ret = -ENODEV;
        goto err_free_i2c;
    }

    printf("[%s]Sensor ID : 0x%04X\n", __func__, (id[1] << 8) | id[0]);

    info->current_mode = ov5645_init_mode_SXGA_1280_960;
    info->new_mode = ov5645_init_mode_SXGA_1280_960;

    /* === init MIPI CSI-2, allocate CSI-2 memory and initialize CSI-2 Rx === */
    info->cdsidev = zalloc(sizeof(struct cdsi_dev));
    if (info->cdsidev == NULL) {
        ret = -ENOMEM;
        goto err_free_i2c;
    }

    info->cdsidev = init_csi_rx(CDSI0, CDSI_RX);
    if (info->cdsidev == NULL) {
        ret = -EINVAL;
        goto err_free_cdsi;
    }

    /* Execute CSI-2 RX start sequence */
    mipi_csi2_start(info->cdsidev);

    /* get virtual channel */
    info->cdsidev->v_channel = mipi_csi2_get_virtual_channel(info->cdsidev);

    /* set data and clk lane */
    info->cdsidev->lanes = CDSIRX_LANE_ENABLE_VAL;
    mipi_csi2_set_lane(info->cdsidev);

    /* get data and clk lane */
    info->cdsidev->lanes = mipi_csi2_get_lane(info->cdsidev);

    /* get ov5645 sensor data type */
    ret = data_read(info->cam_i2c, 0x4300, &ov5645_datatype);
    if (ret) {
        goto err_free_cdsi;
    }

    /* set data type */
    mipi_csi2_set_datatype(info->cdsidev, ov5645_datatype);

 #if 1 //for debugging
    printf("[%s]info->cdsidev: 0x%x\n",
           __func__, info->cdsidev);

    printf("[%s]info->cdsidev->v_channel: 0x%x\n",
           __func__, info->cdsidev->v_channel);

    printf("[%s]info->cdsidev->datatype: 0x%x\n",
           __func__, info->cdsidev->datatype);

    printf("[%s]info->cdsidev->lanes: 0x%x\n",
           __func__, info->cdsidev->lanes);

    printf("[%s]info->cdsidev->base: 0x%x\n",
           __func__, info->cdsidev->base);
#endif

    /* === setup sensor mode - 1280*960 is default setting === */
    if (set_mode(info->cdsidev, ov5645_mode_settings[info->current_mode].regs,
        info->cam_i2c, info->current_mode, info->new_mode)) {
        ret = -EINVAL;
        goto err_free_cdsi;
    }

    /* === initialize meta data === */
    info->mdata_info = zalloc(MATA_DATA_SIZE * sizeof(struct metadata_info));
    if (info->mdata_info == NULL) {
        ret = -ENOMEM;
        goto err_free_metadata;
    }

    info->mdata_info->request_id = req_id;
    info->mdata_info->frame_number = FRAME_NUMBER;
    info->mdata_info->stream = STREAM;
    info->mdata_info->padding = PADDING;
    info->mdata_info->data = m_data;

    info->pix_fmt.pixelformat = ov5645_out_fmt;
    info->pix_fmt.width = SXGA_WIDTH;
    info->pix_fmt.height = SXGA_HEIGHT;

    info->state = OV5645_STATE_OPEN;

    return ret;

/* Error Handle */
err_free_metadata:
    free(info->mdata_info);
err_free_cdsi:
    free(info->cdsidev);
err_free_i2c:
    up_i2cuninitialize(info->cam_i2c);
err_power_down:
    op_power_down(dev);
err_free_support:
    free(info->str_cfg_sup);
err_free_info:
    free(info);
    info = NULL;

    printf("[%s]***ERROR*** Fails to open driver!\n", __func__);
    info->state = OV5645_STATE_CLOSED;

    return ret;
}

/**
 * @brief Close camera device
 * @param dev pointer to structure of device data
 */
static void ov5645_dev_close(struct device *dev)
{
    struct sensor_info *info = NULL;

    printf("[%s]+\n", __func__);

    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    /* free all of the resources */
    free(info->cdsidev);
    up_i2cuninitialize(info->cam_i2c);
    op_power_down(dev);
    free(info->mdata_info);
    free(info->str_cfg_sup);

#if 1
    /* Stop stream */
    data_write(info->cam_i2c, REG_STREAM_ONOFF, stream_off);
    usleep(DELAY_10);
#endif

    mipi_csi2_stop(info->cdsidev);

    /* deinitialize CSI-2 Rx */
    //deinit_csi_rx(info->cdsidev);// disable clk

    info->state = OV5645_STATE_CLOSED;

    printf("[%s]-\n", __func__);

}

/**
 * @brief Probe camera device
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int ov5645_dev_probe(struct device *dev)
{
    struct sensor_info *info = NULL;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->state = OV5645_STATE_CLOSED;
    info->dev = dev;
    device_set_private(dev, info);

    return 0;
}

/**
 * @brief Remove camera device
 * @param dev pointer to structure of device data
 */
static void ov5645_dev_remove(struct device *dev)
{
    struct sensor_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    device_set_private(dev, NULL);
    free(info);
    info = NULL;
}

static struct device_camera_type_ops ov5645_type_ops = {

     /** power up camera module */
    .power_up = op_power_up,

    /** power down camera module */
    .power_down = op_power_down,

    /** Camera capabilities */
    .capabilities = op_capabilities,

    /** Get required size of various data  */
    .get_required_size = op_get_required_size,

    /** Set configures to camera module */
    .set_streams_cfg = op_set_streams_cfg,

    /** Start Capture */
    .capture = op_capture,

    /** stop capture */
    .flush = op_flush,

    /** Meta data request */
    .trans_metadata = op_get_meta_data,
};

static struct device_driver_ops ov5645_driver_ops = {
    .probe              = ov5645_dev_probe,
    .remove             = ov5645_dev_remove,
    .open               = ov5645_dev_open,
    .close              = ov5645_dev_close,
    .type_ops           = &ov5645_type_ops,
};

struct device_driver camera_driver = {
    .type       = DEVICE_TYPE_CAMERA_HW,
    .name       = "camera",
    .desc       = "ov5645 5M Camera Driver",
    .ops        = &ov5645_driver_ops,
};
