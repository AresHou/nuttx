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

#ifndef __INCLUDE_NUTTX_DEVICE_CAMERA_H
#define __INCLUDE_NUTTX_DEVICE_CAMERA_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/greybus/types.h>

#define DEVICE_TYPE_CAMERA_HW    "camera"

/**
 * Camera module capabilities
 */
#define CAP_METADATA_GREYBUS     BIT(0)
#define CAP_METADATA_MIPI        BIT(1)
#define CAP_STILL_IMAGE          BIT(2)
#define CAP_JPEG                 BIT(3)

/**
 * Query required data size to report
 */
#define SIZE_CONFIG_SUPPORT     0
#define SIZE_CONFIG_CURRENT     1
#define SIZE_META_DATA          2

/**
 * @brief Capture finish callback function
 */
typedef void (*capture_callback)(void);

/**
 *  Supported Stream Configuration
 */
struct streams_cfg_sup {
    /** Image width in pixels */
    uint16_t    width;
    /** Image height in pixels */
    uint16_t    height;
    /** Image format */
    uint16_t    format;
    /** Must be set to zero */
    uint16_t    padding;
};

/**
 * Answer of Stream Configurations from Camera
 */
struct streams_cfg_ans {
    /** Image width in pixels */
    uint16_t    width;
    /** Image height in pixels */
    uint16_t    height;
    /** Image format */
    uint16_t    format;
    /** Virtual channel number for the stream */
    uint8_t     virtual_channel;
    /** Data type for the stream */
    uint8_t     data_type;
    /** Maximum frame size in bytes */
    uint32_t    max_size;
};

/**
 *  Parameters for Camera Capture
 */
struct capture_info {
    /** An incrementing integer to uniquely identify the capture request */
    uint32_t    request_id;
    /** Bitmask of the streams included in the capture request */
    uint8_t     streams;
    /** Must be set to zero */
    uint8_t     padding;
    /** Number of frames to capture */
    uint16_t    num_frames;
    /** Capture  request settings */
    uint8_t     *settings;
    /** callback function for capture finish */
    capture_callback callback;
};

/**
 *  Parameters for Getting Meta Data
 */
struct meta_data_info {
    /** The ID of the corresponding capture request */
    uint32_t    request_id;
    /** CSI-2 frame number */
    uint16_t    frame_number;
    /** The stream number */
    uint8_t     stream;
    /** Must be set to zero */
    uint8_t     padding;
    /** Meta-data block */
    uint8_t     *data;
};

/**
 * Camera device driver operations
 */
struct device_camera_type_ops {
    /** power up camera module */
    int (*power_up)(struct device *dev);
    /** power down camera module */
    int (*power_down)(struct device *dev);
    /** Camera capabilities */
    int (*capabilities)(struct device *dev, uint32_t *size,
                        uint8_t *capabilities);
    /** Get required size of various data  */
    int (*get_required_size)(struct device *dev, uint8_t operation,
                             uint32_t *size);
    /** Get camera module supported configure */
    int (*get_support_strm_cfg)(struct device *dev, uint16_t num_streams,
                                   struct streams_cfg_ans *config);
    /** Set configures to camera module */
    int (*set_streams_cfg)(struct device *dev, uint16_t num_streams,
                           struct streams_cfg_sup *config);
    /** Get current configures from camera module */
    int (*get_current_strm_cfg)(struct device *dev, uint16_t *flags,
                                   struct streams_cfg_ans *config);
    /** Start Capture */
    int (*start_capture)(struct device *dev, struct capture_info *capt_info);
    /** stop capture */
    int (*stop_capture)(struct device *dev, uint32_t *request_id);
    /** Meta data request */
    int (*get_meta_data)(struct device *dev, struct meta_data_info *meta_data);
};

/**
 * @brief Power up camera module
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_power_up(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->power_up) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->power_up(dev);
    }
    return -ENOSYS;
}

/**
 * @brief Power down camera module
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_power_down(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->power_down) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->power_down(dev);
    }
    return -ENOSYS;
}

/**
 * @brief Get capabilities of camera module
 *
 * @param dev Pointer to structure of device data
 * @param capabilities Pointer that will be stored Camera Module capabilities.
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_capabilities(struct device *dev, uint16_t *size,
                                             uint8_t *capabilities)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->capabilities) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->capabilities(dev, size,
                                                                capabilities);
    }
    return -ENOSYS;
}

/**
 * @brief Get required data size of camera module information
 *
 * @param dev Pointer to structure of device data
 * @param capabilities Pointer that will be stored Camera Module capabilities.
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_get_required_size(struct device *dev,
                                              uint8_t operation, uint32_t *size)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->get_required_size) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->get_required_size(dev,
                                                               operation, size);
    }
    return -ENOSYS;
}

/**
 * @brief Get camera module support stream cofigurations
 *
 * @param dev Pointer to structure of device data
 * @param num_streams Number of streams
 * @param config Pointer to structure of streams configuration
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_get_support_strm_cfg(struct device *dev,
                           uint16_t num_streams, struct streams_cfg_ans *config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->get_support_strm_cfg) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->get_support_strm_cfg(dev,
                                                           num_streams, config);
    }
    return -ENOSYS;
}

/**
 * @brief Set streams configuration to camera module
 *
 * @param dev Pointer to structure of device data
 * @param num_streams Number of streams
 * @param config Pointer to structure of streams configuration
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_set_streams_cfg(struct device *dev,
                           uint16_t num_streams, struct streams_cfg_sup *config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->set_streams_cfg) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->set_streams_cfg(dev,
                                                           num_streams, config);
    }
    return -ENOSYS;
}

/**
 * @brief Get applied settings of streams configuration from camera channel
 *
 * @param dev Pointer to structure of device data
 * @param flags Flags of configuration
 * @param config Pointer to structure of streams configuration
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_get_current_strm_cfg(struct device *dev,
                                uint16_t *flags, struct streams_cfg_ans *config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->get_current_strm_cfg) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->get_current_strm_cfg(dev,
                                                                 flags, config);
    }
    return -ENOSYS;
}

/**
 * @brief Start the camera capture
 *
 * @param dev Pointer to structure of device data
 * @param capt_info Capture parameters
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_start_capture(struct device *dev,
                                              struct capture_info *capt_info)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->start_capture) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->start_capture(dev,
                                                                 capt_info);
    }
    return -ENOSYS;
}

/**
 * @brief Stop the camera capture
 *
 * @param dev Pointer to structure of device data
 * @param request_id The request id set by capture
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_stop_capture(struct device *dev,
                                             uint32_t *request_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->stop_capture) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->stop_capture(dev,
                                                                request_id);
    }
    return -ENOSYS;
}

/**
 * @brief Get meta_data from camera module
 *
 * @param dev Pointer to structure of device data
 * @param meta_data Pointer to Meta-data block
 * @return 0 on success, negative errno on error
 */
static inline int device_camera_get_meta_data(struct device *dev,
                                               struct meta_data_info *meta_data)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, camera)->get_meta_data) {
        return DEVICE_DRIVER_GET_OPS(dev, camera)->get_meta_data(dev, meta_data);
    }
    return -ENOSYS;
}

#endif
