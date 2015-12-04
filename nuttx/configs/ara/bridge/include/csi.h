/**
 * Copyright (c) 2015 Google Inc.
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
 *
 * @brief OV5645 Camera Sensor Driver
 */

#ifndef __OV_CSI_H__
#define __OV_CSI_H__

/**
 * CSI control for camera stream start
 */
struct csi_control {
    /** cdsi control id */
    uint8_t csi_id;
    /** clocking mode for continuous or non-continuous */
    uint8_t clock_mode;
    /** lane number for camera stream */
    uint8_t lane_num;
    /** reserve */
    uint8_t padding;
    /** csi frequence */
    uint32_t bus_freq;
};

/**
 * @brief APB-A csi control initial process
 *
 * @return 0 on success, negative errno on error.
 */
int csi_tx_init(void);

/**
 * @brief Start the CSI for data streaming
 *
 * @param csi_ctrl Pointer to structure of CSI control settings.
 * @return 0 on success, negative errno on error.
 */
int csi_tx_start(struct csi_control *csi_ctrl);

/**
 * @brief Stop the CSI for data streaming
 *
 * @return 0 on success, negative errno on error.
 */
int csi_tx_stop(void);


int camera_init(void);


#endif /* __OV_CSI_H__ */

