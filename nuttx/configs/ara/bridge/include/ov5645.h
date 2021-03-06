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

#ifndef _OV5645_H_
#define _OV5645_H_

#include <stdint.h>

int ov5645_init(int mode);

/* CDSI deifne */
#define CDSI0 0
#define CDSI1 1
#define CDSI_RX 0
#define CDSI_TX 1

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


#endif  /* _OV5645_H_ */

