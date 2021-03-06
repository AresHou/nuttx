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
#include <arch/board/cdsi0_offs_def.h>
#include <arch/board/cdsi0_reg_def.h>
#include <arch/tsb/cdsi.h>
#include <arch/board/mipi_csi2.h>

/* Delay Time */
#define CSI2_DELAY_10            10

struct cdsi_dev *init_csi_rx(int cdsi, int tx)
{
    struct cdsi_dev *cdsidev;

    cdsidev = cdsi_initialize(cdsi, tx);
    if (!cdsidev) {
        printf("[%s]csdi_init fails. cdsidev: 0x%x\n",__func__, cdsidev);
        return NULL;
    }

#if 0 //For debugging
    printf("[%s]cdsidev: 0x%x\n",__func__, cdsidev);
    printf("[%s]cdsidev->base: 0x%x\n",__func__, cdsidev->base);
    printf("[%s]tx: %d\n",__func__, tx);
    printf("[%s]cdsi: 0x%x\n",__func__, cdsi);
#endif

    return cdsidev;
}

void deinit_csi_rx(struct cdsi_dev *dev)
{

#if 0 //For debugging
    printf("[%s]cdsidev: 0x%x\n",__func__, dev);
    printf("[%s]cdsidev->base: 0x%x\n",__func__, dev->base);
    printf("[%s]tx: %d\n",__func__, dev->tx);
#endif

    cdsi_uninitialize(dev);

}


int mipi_csi2_start(struct cdsi_dev *cdsidev)
{
    /* Enable the Rx bridge and set to CSI mode */
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS, AL_RX_BRG_MODE_VAL);
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_CSI_INFO_OFFS, AL_RX_BRG_CSI_INFO_VAL);
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_CSI_DT0_OFFS, AL_RX_BRG_CSI_DT0_VAL);
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_CSI_DT1_OFFS, AL_RX_BRG_CSI_DT1_VAL);
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_CSI_DT2_OFFS, AL_RX_BRG_CSI_DT2_VAL);
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_CSI_DT3_OFFS, AL_RX_BRG_CSI_DT3_VAL);

    /* Enable CDSIRX */
    cdsi_write(cdsidev, CDSI0_CDSIRX_CLKEN_OFFS, CDSIRX_CLKEN_VAL);
    /* Set CDSIRX functions enable */
    cdsi_write(cdsidev, CDSI0_CDSIRX_FUNC_ENABLE_OFFS, CDSIRX_FUNC_ENABLE_VAL);
    /* Set LPRX calibration */
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_LPRXCALCNTRL_OFFS,
               CDSIRX_PPI_DPHY_LPRXCALCNTRL_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_LPRX_THSLD_OFFS,
               CDSIRX_PPI_DPHY_LPRX_THSLD_VAL);
    /* Start LPRX calibration */
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_OFFS,
               CDSI0_CDSIRX_PPI_DPHY_LPRXAUTOCALST_VAL);
    /* CDSIRX configuration */
    cdsi_write(cdsidev, CDSI0_CDSIRX_CLKSEL_OFFS, CDSIRX_CLKSEL_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_MODE_CONFIG_OFFS, CDSIRX_MODE_CONFIG_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_LANE_ENABLE_OFFS, CDSIRX_LANE_ENABLE_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_VC_ENABLE_OFFS, CDSIRX_VC_ENABLE_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_LINE_INIT_COUNT_OFFS,
               CDSIRX_LINE_INIT_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_HSRXTO_COUNT_OFFS,
               CDSIRX_HSRXTO_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_FUNC_MODE_OFFS, CDSIRX_FUNC_MODE_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_LPTXTIMECNT_OFFS,
               CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS,
               CDSIRX_DSI_LPTX_MODE_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DSI_BTA_COUNT_OFFS,
               CDSIRX_PPI_DSI_BTA_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_HSRX_CNTRL_OFFS,
               CDSIRX_PPI_HSRX_CNTRL_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_HSRX_COUNT_OFFS,
               CDSIRX_PPI_HSRX_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_OFFS,
               CDSI0_CDSIRX_PPI_DPHY_POWERCNTRL_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DSI_DPHYTX_ADJUST_OFFS,
               CDSIRX_PPI_DSI_DPHYTX_ADJUST_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_HSRX_ADJUST_OFFS,
               CDSIRX_PPI_DPHY_HSRX_ADJUST_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_DLYCNTRL_OFFS,
               CDSIRX_PPI_DPHY_DLYCNTRL_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_MASK_OFFS,
               CDSIRX_LPRX_STATE_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_RXTRIG_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_RXERR_INT_MASK_OFFS,
               CDSI0_CDSIRX_RXERR_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_TXERR_INT_MASK_OFFS,
               CDSI0_CDSIRX_TXERR_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC0_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC1_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC2_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC3_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC0_LN_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC1_LN_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC2_LN_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_OFFS,
               CDSI0_CDSIRX_DSI_VC3_LN_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_SH_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_LN0_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC0_LN1_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_LN0_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC1_LN1_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_LN0_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC2_LN1_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_LN0_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_OFFS,
               CDSI0_CDSIRX_CSI2_VC3_LN1_INT_MASK_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_LPTXTO_COUNT_OFFS,
               CDSIRX_DSI_LPTXTO_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_TATO_COUNT_OFFS,
               CDSIRX_DSI_TATO_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_WAITBTA_COUNT_OFFS,
               CDSI0_CDSIRX_DSI_WAITBTA_COUNT_VAL);

    return 0;
}

int mipi_csi2_stop(struct cdsi_dev *cdsidev)
{
    uint32_t rdata3;
    uint32_t lane_status_hs, lane_status_lp, internal_stat;  

#if 0
    /* Check Lane status - HS */
    lane_status_hs = cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_STATUS_HS_OFFS);
    printf("[%s] lane_status_hs = 0x%08x \n", __func__, lane_status_hs);
    while (lane_status_hs != HS_LANE_STATUS) {
        lane_status_hs = cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_STATUS_HS_OFFS);
    }    
#endif

#if 0
    /* Check Lane status - LP */
    lane_status_lp = cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_STATUS_LP_OFFS);
    printf("[%s] lane_status_lp = 0x%08x \n", __func__, lane_status_lp);
    while (lane_status_lp != LP_LANE_STATUS) {
        lane_status_lp = cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_STATUS_LP_OFFS);
    }
#endif

#if 0
    /* Check CDSIRX Internal state. */
    internal_stat = cdsi_read(cdsidev, CDSI0_CDSIRX_INTERNAL_STAT_OFFS);
    printf("[%s] internal_stat = 0x%08x \n", __func__, internal_stat);
    while (internal_stat != INTERNAL_STAT_BUSY) {
        internal_stat = cdsi_read(cdsidev, CDSI0_CDSIRX_INTERNAL_STAT_OFFS);
    }
#endif

    /* Stop CDSIRX */
    cdsi_write(cdsidev, CDSI0_CDSIRX_START_OFFS, CDSI0_CDSIRX_STOP);

    /* Clear CDSIRX internal state */
    cdsi_write(cdsidev, CDSI0_CDSIRX_SYSTEM_INIT_OFFS,
               CDSI0_CDSIRX_SYSTEM_CLEAR_VAL);

    /* Disable CDSIRX */
    cdsi_write(cdsidev, CDSI0_CDSIRX_CLKEN_OFFS, CDSI0_CDSIRX_CLKDISABLE_VAL);

    /* Disable RX Bridge */
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS, CDSI0_AL_RX_BRG_DISABLE_VAL);

    usleep(CSI2_DELAY_10);

#if 0
    printf("[%s] Check RX Bridge Status...\n", __func__);
    /* Check RX Bridge Status - Check the internal state is no busy */
    rdata3 = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS) & 0x4;
    while (rdata3 != 0x0) {
        rdata3 = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS) & 0x4;
    }
#endif

    return 0;
}

uint8_t mipi_csi2_get_datatype(struct cdsi_dev *cdsidev)
{
    return cdsidev->datatype;
}

int mipi_csi2_set_datatype(struct cdsi_dev *cdsidev, uint8_t data_type)
{
    cdsidev->datatype = data_type;

    return 0;
}

uint8_t mipi_csi2_get_virtual_channel(struct cdsi_dev *cdsidev)
{
    return cdsi_read(cdsidev, CDSI0_CDSIRX_VC_ENABLE_OFFS);
}

int mipi_csi2_set_virtual_channel(struct cdsi_dev *cdsidev, uint8_t VC_enable )
{
    /*
     *
     * 0:disabled, 1:enabled
     * 0x00: Virtual Channel 0
     * 0x01: Virtual Channel 1
     * 0x02: Virtual Channel 2
     * 0x03: Virtual Channel 3
     *
     */
    cdsi_write(cdsidev, CDSI0_CDSIRX_VC_ENABLE_OFFS, VC_enable);

    return 0;
}

int mipi_csi2_set_lane(struct cdsi_dev *cdsidev)
{
    /*
     * 
     * BIT4: Clock Lane
     * 0: Lane operation disabled 
     * 1: Lane operation enabled
     * 
     * BIT0-BIT2: Data Lane
     * In order to receive data, at least Data Lane 0 shall be enabled.
     * 000: All data lane is disabled.
     * 001: Data Lane 0 is enabled.
     * 010: Data Lane 0 and 1 are enabled.
     * 011: Data Lane 0, 1 and 2 are enabled.
     * 100: Data Lane 0, 1, 2 and 3 are enabled
     *
     */
    cdsi_write(cdsidev, CDSI0_CDSIRX_LANE_ENABLE_OFFS, cdsidev->lanes);

    return 0;
}

uint8_t mipi_csi2_get_lane(struct cdsi_dev *cdsidev)
{
    return cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_ENABLE_OFFS);
}

void mipi_csi2_get_error(struct cdsi_dev *cdsidev)
{
    uint32_t err_msg = 0;

    err_msg = cdsi_read(cdsidev, CDSI0_CDSIRX_ERR_STATUS_OFFS);
    printf("[%s]Error Message: 0x%x \n", __func__, err_msg);
}
