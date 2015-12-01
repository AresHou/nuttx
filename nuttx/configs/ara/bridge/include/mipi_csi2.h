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
 
#ifndef _MIPI_CSI2_H_
#define _MIPI_CSI2_H_

#include <arch/tsb/cdsi.h>

/* CDSI deifne */
#define CDSI0   0
#define CDSI1   1
#define CDSI_RX 0
#define CDSI_TX 1

/* Register status */
#define INTERNAL_STAT_BUSY  0x00000000
#define HS_LANE_STATUS      0x00000000
#define LP_LANE_STATUS      0x0040008f
#define AL_RX_BRG_MODE_BUSY 0x04

/* CSI-2 register values */
#define AL_RX_BRG_MODE_VAL                              0x00000003
#define AL_RX_BRG_CSI_INFO_VAL                          0x00000000
#define AL_RX_BRG_CSI_DT0_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT1_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT2_VAL                           0x00000000
#define AL_RX_BRG_CSI_DT3_VAL                           0x00000000

#define CDSIRX_CLKEN_VAL                                0x00000001
#define CDSIRX_CLKSEL_VAL                               0x00000101
#define CDSIRX_MODE_CONFIG_VAL                          0x00000001
#define CDSIRX_LANE_ENABLE_VAL                          0x00000012
#define CDSIRX_VC_ENABLE_VAL                            0x0000000F
#define CDSIRX_LINE_INIT_COUNT_VAL                      0x000012C0
#define CDSIRX_HSRXTO_COUNT_VAL                         0xFFFFFFFF
#define CDSIRX_FUNC_ENABLE_VAL                          0x00070701
#define CDSIRX_DSI_LPTX_MODE_VAL                        0x00000001
#define CDSIRX_DSI_TATO_COUNT_VAL                       0xFFFFFFFF
#define CDSIRX_DSI_LPTXTO_COUNT_VAL                     0xFFFFFFFF
#define CDSIRX_FUNC_MODE_VAL                            0x00000000
#define CDSIRX_PPI_HSRX_CNTRL_VAL                       0x40000000
#define CDSIRX_PPI_HSRX_COUNT_VAL                       0x0400000A
#define CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_VAL            0x00000003
#define CDSIRX_PPI_DPHY_DLYCNTRL_VAL                    0x00000000
#define CDSIRX_PPI_DPHY_LPRX_THSLD_VAL                  0x000002AA
#define CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_VAL         0x00000001
#define CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL                 0x00000FFF
#define CDSIRX_PPI_DSI_BTA_COUNT_VAL                    0x000407FF
#define CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL                0x00000002
#define CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL                 0x000002AA
#define CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL                0x00190040
#define CDSIRX_LPRX_STATE_INT_MASK_VAL                  0x1F1F1F1D

#define CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_RXERR_INT_MASK_VAL                 0x00000000
#define CDSI0_CDSIRX_TXERR_INT_MASK_VAL                 0x00000000
#define CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_VAL            0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_VAL           0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_VAL          0x00000000
#define CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_VAL          0x00000000

#define CDSI0_CDSIRX_DSI_WAITBTA_COUNT_VAL              0x10000010
#define CDSI0_CDSIRX_START_VAL                          0x00000001
#define CDSI0_CDSIRX_LPRX_STATE_INT_STAT_VAL            0x00000001
#define CDSI0_CDSIRX_ADDRESS_CONFIG_VAL                 0x00000000

#define CDSI0_CDSIRX_START_VAL                          0x00000001
#define CDSI0_CDSIRX_SYSTEM_INIT_VAL                    0x10000010
#define CDSI0_CDSIRX_CLKEN_VAL                          0x10000010

#define CDSI0_CDSIRX_STOP                               0x00000000
#define CDSI0_CDSIRX_SYSTEM_CLEAR_VAL                   0x00000001
#define CDSI0_CDSIRX_CLKDISABLE_VAL                     0x00000000
#define CDSI0_AL_RX_BRG_DISABLE_VAL                     0x00000000

/* mipi data type */
#define MIPI_DT_YUV420_8bit        0x18
#define MIPI_DT_YUV420_10bit       0x19
#define MIPI_DT_YUV420_LEGACY_8bit 0x1a
#define MIPI_DT_YUV422_8bit        0x1e
#define MIPI_DT_YUV422_10bit       0x1f
#define MIPI_DT_RGB444             0x20
#define MIPI_DT_RGB555             0x21
#define MIPI_DT_RGB565             0x22
#define MIPI_DT_RGB666             0x23
#define MIPI_DT_RGB888             0x24
#define MIPI_DT_RAW6               0x28
#define MIPI_DT_RAW7               0x29
#define MIPI_DT_RAW8               0x2a
#define MIPI_DT_RAW10              0x2b
#define MIPI_DT_RAW12              0x2c
#define MIPI_DT_RAW14              0x2d

/* mipi csi2 API */
#if 1
struct cdsi_dev *init_csi_rx(int cdsi, int tx);
void deinit_csi_rx(struct cdsi_dev *dev);
#endif

int mipi_csi2_stop(struct cdsi_dev *cdsidev);
int mipi_csi2_start(struct cdsi_dev *cdsidev);

uint8_t mipi_csi2_get_datatype(struct cdsi_dev *cdsidev);
int mipi_csi2_set_datatype(struct cdsi_dev *cdsidev, uint8_t data_type);
uint8_t mipi_csi2_get_virtual_channel(struct cdsi_dev *cdsidev);
int mipi_csi2_set_virtual_channel(struct cdsi_dev *cdsidev, uint8_t);
int mipi_csi2_set_lane(struct cdsi_dev *cdsidev);
uint8_t mipi_csi2_get_lane(struct cdsi_dev *cdsidev);
void mipi_csi2_get_error(struct cdsi_dev *cdsidev);

#endif  /* _MIPI_CSI2_H_ */
