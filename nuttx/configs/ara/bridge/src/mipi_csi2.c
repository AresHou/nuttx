#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <debug.h>
#include <arch/board/cdsi0_offs_def.h>
#include <arch/board/cdsi0_reg_def.h>
#include <arch/board/mipi_csi2.h>

#if 1
int mipi_csi2_start(struct cdsi_dev *cdsidev)
{
    uint32_t rdata0;
    
    printf("[%s]start CSI-2 rx\n", __func__);
    
    //refer to p.177 on the spec. ARA_ES2_APBridge_rev100.pdf
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
    cdsi_write(cdsidev, CDSI0_CDSIRX_HSRXTO_COUNT_OFFS, CDSIRX_HSRXTO_COUNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_FUNC_MODE_OFFS, CDSIRX_FUNC_MODE_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_PPI_DPHY_LPTXTIMECNT_OFFS,
               CDSIRX_PPI_DPHY_LPTXTIMECNT_VAL);
    cdsi_write(cdsidev, CDSI0_CDSIRX_DSI_LPTX_MODE_OFFS, CDSIRX_DSI_LPTX_MODE_VAL);
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

    /* Wait LPRX calibration finish */
    rdata0 = cdsi_read(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    while ((rdata0 &
            CDSI0_CDSIRX_LPRX_STATE_INT_STAT_AUTOCALDONE_MASK) == 0x0) {
        rdata0 = cdsi_read(cdsidev, CDSI0_CDSIRX_LPRX_STATE_INT_STAT_OFFS);
    }
    printf("First LPRX_STATE_INT: %d\n", rdata0);
    
    return 0;
}
#endif 

#if 1
/* Refer to page 192 5.9.5.2.1.2. Stop Sequence, located at the spec. ARA_ES2_APBridge_rev100.pdf */
int mipi_csi2_stop(struct cdsi_dev *cdsidev)
{
    uint32_t rdata3;
    uint8_t lane_status_hs, lane_status_lp, internal_stat;

    printf("[%s]stop CSI-2 rx + \n", __func__);
    
#if 0
    /* Check Lane status */
    ChkHSLane = cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_STATUS_HS_OFFS);
    printf("[%s] lane_status_hs = 0x%x \n", __func__, lane_status_hs);
    while ((lane_status_hs >> 7) == STOPSTATE) {
        lane_status_hs = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS);
    }    
    
    lane_status_lp = cdsi_read(cdsidev, CDSI0_CDSIRX_LANE_STATUS_LP_OFFS);
    printf("[%s] lane_status_lp = 0x%x \n", __func__, lane_status_lp);
    while ((lane_status_lp & 0x0f) == 0x1) {
        lane_status_lp = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS);
    } 
    
    /* Check CDSIRX Internal state. */
    internal_stat = cdsi_read(cdsidev, CDSI0_CDSIRX_INTERNAL_STAT_OFFS);
    printf("[%s] internal_stat = 0x%x \n", __func__, internal_stat);
    while (internal_stat == INTERNAL_STAT_BUSY) {
        internal_stat = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS);
    } 
#endif
    
    /* Stop CDSIRX */
    cdsi_write(cdsidev, CDSI0_CDSIRX_START_OFFS, 0x00000000);
    
    /* Clear CDSIRX internal state */
    cdsi_write(cdsidev, CDSI0_CDSIRX_SYSTEM_INIT_OFFS, 0x00000001);
    
    /* Disable CDSIRX */
    cdsi_write(cdsidev, CDSI0_CDSIRX_CLKEN_OFFS, 0x00000000);
    
    /* Disable RX Bridge */
    cdsi_write(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS, 0x00000000);

    /* Check RX Bridge Status */
    /* Wait LPRX calibration finish */
    rdata3 = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS);
    while (rdata3 == 0x0) {
        rdata3 = cdsi_read(cdsidev, CDSI0_AL_RX_BRG_MODE_OFFS);
    }
    
    printf("[%s]stop CSI-2 rx - \n", __func__);
    
    return 0;

}
#endif

uint8_t mipi_csi2_get_datatype(struct cdsi_dev *cdsidev)
{
    uint8_t dtype;

    dtype = cdsidev->datatype;

    return dtype;
}

int mipi_csi2_set_datatype(struct cdsi_dev *cdsidev, uint8_t data_type)
{
    uint8_t dtype;

    dtype = cdsidev->datatype;
    cdsidev->datatype = data_type;
    dtype = cdsidev->datatype;
    
    return 0;
}

uint8_t mipi_csi2_get_virtual_channel(struct cdsi_dev *cdsidev)
{
    uint8_t v_channel;

    v_channel = cdsi_read(cdsidev, CDSI0_CDSIRX_VC_ENABLE_OFFS);

    return v_channel;
}

int mipi_csi2_set_virtual_channel(struct cdsi_dev *cdsidev, uint8_t VCEn )
{

    /* 2.7.1.3.7 */
    VCEn = cdsidev->v_channel;
    cdsi_write(cdsidev, CDSI0_CDSIRX_VC_ENABLE_OFFS, VCEn);


    return 0;
}

int mipi_csi2_set_lanes(struct cdsi_dev *cdsidev, uint8_t DTLaneEn)
{
    
    /* 2.7.1.3.6 */
    /* 
       000: All data lane is disabled.
       001: Data Lane 0 is enabled.
       010: Data Lane 0 and 1 are enabled.
       011: Data Lane 0, 1 and 2 are enabled.
       100: Data Lane 0, 1, 2 and 3 are enabled.
    */
    cdsi_write(cdsidev, CDSI0_CDSIRX_LANE_ENABLE_OFFS, DTLaneEn);

    return 0;
}

int mipi_csi2_get_error(struct cdsi_dev *cdsidev)
{
    
    
    return 0;
}
