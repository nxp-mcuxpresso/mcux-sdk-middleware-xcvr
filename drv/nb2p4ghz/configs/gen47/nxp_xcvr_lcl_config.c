/*
 * Copyright 2018-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */

#include "nxp_xcvr_lcl_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 *******************************************************************************/
const xcvr_lcl_tsm_config_t xcvr_lcl_tsm_generic_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /*********** xcvr_lcl_tsm_generic_config *******/
    /***********************************************/
    /* CTRL register value is not modified for RSM operation. */
    /* LPPS_CTRL register value is not modified for RSM operation. */
    .END_OF_SEQ    = 0xCDCBBFBDU,
    .WU_LATENCY    = (XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY(0U) | XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY(4U)),
    .RECYCLE_COUNT = 0x002919C8U,
    .FAST_CTRL1    = XCVR_TSM_FAST_CTRL1_FAST_TX_WU_EN(1U) | XCVR_TSM_FAST_CTRL1_FAST_RX_WU_EN(1U) |
                  XCVR_TSM_FAST_CTRL1_FAST_RX2TX_EN(1U) | XCVR_TSM_FAST_CTRL1_PWRSAVE_TX_WU_EN(0U) |
                  XCVR_TSM_FAST_CTRL1_PWRSAVE_RX_WU_EN(0U) | XCVR_TSM_FAST_CTRL1_FAST_RX2TX_START(0x2CU) |
                  XCVR_TSM_FAST_CTRL1_FAST_TX2RX_EN(1U) | XCVR_TSM_FAST_CTRL1_FAST_TX2RX_START(0x2CU),
    .FAST_CTRL2 = XCVR_TSM_FAST_CTRL2_FAST_START_TX(0x37) | XCVR_TSM_FAST_CTRL2_FAST_DEST_TX(0xBB) |
                  XCVR_TSM_FAST_CTRL2_FAST_START_RX(0x37) | XCVR_TSM_FAST_CTRL2_FAST_DEST_RX(0xC8),
    .FAST_CTRL3 = XCVR_TSM_FAST_CTRL3_FAST_RX2TX_START_FC(0x19) | XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC(0x19),
    /* TIMING00 register value is not modified for RSM operation. */
    /* TIMING01 register value is not modified for RSM operation. */
    /* TIMING02 register value is not modified for RSM operation. */
    /* TIMING03 register value is not modified for RSM operation. */
    /* TIMING04 register value is not modified for RSM operation. */
    /* TIMING05 register value is not modified for RSM operation. */
    /* TIMING06 register value is not modified for RSM operation. */
    /* TIMING07 register value is not modified for RSM operation. */
    /* TIMING08 register value is not modified for RSM operation. */
    .TIMING09 = 0xFFFFFFFFU,
    .TIMING10 = 0x11081108U,
    .TIMING11 = 0xCC1ABE1AU,
    .TIMING12 = 0xCC22BE22U,
    .TIMING13 = 0xFFFFFFFFU,
    .TIMING14 = 0x0000BEBBU,
    .TIMING15 = 0xCCCABEBCU,
    .TIMING16 = 0xC9C80000U,
    .TIMING17 = 0xCCC80000U,
    .TIMING18 = 0xCCC80000U,
    .TIMING19 = 0x11061106U,
    .TIMING20 = 0x11001100U,
    .TIMING21 = 0x19001900U,
    .TIMING22 = 0x19001900U,
    .TIMING23 = 0x19001900U,
    .TIMING24 = 0xCC00BE00U,
    .TIMING25 = 0xCD00BF00U,
    .TIMING26 = 0xCD00BF00U,
    .TIMING27 = 0xCD00BF00U,
    .TIMING28 = 0xCD00BF00U,
    .TIMING29 = 0xCD00BF00U,
    .TIMING30 = 0xCC2BFFFFU,
    .TIMING31 = 0xCC08BE08U,
    .TIMING32 = 0x19111911U,
    .TIMING33 = 0x2B112B11U,
    .TIMING34 = 0xCC11BE11U,
    .TIMING35 = 0xCC11BE11U,
    .TIMING36 = 0xCC11BE11U,
    .TIMING37 = 0xCC11BE11U,
    .TIMING38 = 0xCC11BE11U,
    .TIMING39 = 0xCC11BE11U,
    .TIMING40 = 0xCC11BE11U,
    .TIMING41 = 0xCC11BE11U,
    .TIMING42 = 0xCC12BE12U,
    .TIMING43 = 0xCC12BE12U,
    .TIMING44 = 0xCC12BE12U,
    .TIMING45 = 0xCC12BE12U,
    .TIMING46 = 0xFFFFBE12U,
    .TIMING47 = 0xCC2ABE2AU,
    .TIMING48 = 0xCC2BBE2BU,
    .TIMING49 = 0xCC2BBE2BU,
    .TIMING50 = 0xCC2BFFFFU,
    .TIMING51 = 0xCC2BFFFFU,
    .TIMING52 = 0xFFFFFFFFU,
    .TIMING53 = 0xFFFFFFFFU,
    .TIMING54 = 0xFFFFFFFFU,
    .TIMING55 = 0xFFFFFFFFU,
    .TIMING56 = 0xFFFFFFFFU,
    .TIMING57 = 0xFFFFFFFFU,
    .TIMING58 = 0xFFFFFFFFU,
    .TIMING59 = 0xFFFFFFFFU,
    .TIMING60 = 0xFFFFFFFFU,
    .TIMING61 = 0xFFFFFFFFU,
    /* OVRD0 register value is not modified for RSM operation. */
    /* OVRD1 register value is not modified for RSM operation. */
    /* OVRD2 register value is not modified for RSM operation. */
    /* OVRD3 register value is not modified for RSM operation. */
    /* OVRD4 register value is not modified for RSM operation. */

    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /*********** xcvr_lcl_tsm_generic_config **********/
    /***********************************************/
};

const xcvr_lcl_rsm_reg_config_t xcvr_lcl_rsm_generic_config = {
/*!<  RSM/TSM register configuration for the generic. Meant to overlay after applying
 the  ::xcvr_lcl_tsm_generic_config settings */

/***********************************************/
/*********** START OF GENERATED CODE ***********/
/*********** xcvr_lcl_rsm_generic_config *******/
/***********************************************/
#if (0)
    .RSM_CTRL0 = 0x050010F0U, /* Fast FC RX & TX WU enabled, Fast IP RX WU enabled, Fast IP TX WU enabled */
    .RSM_CTRL1 = 0x0a31f8e7U,
    .RSM_CTRL2 = 0x00000000U,
    .RSM_CTRL3 = 0x00000106U,
    .RSM_CTRL4 = 0x005a000aU,
    .RSM_CTRL5 = 0x50U, /* T_FM is always 80usec */
    .RSM_CTRL6 = 0x20U, /* Set XCVR_MISC_RSM_CTRL6_RSM_PKTRAM_EXTEND_MASK */
    .RSM_CTRL7 = 0x0U,
#else
    .RSM_CTRL0 = XCVR_MISC_RSM_CTRL0_RSM_MODE(0x0U) | XCVR_MISC_RSM_CTRL0_RSM_RATE(0x0U) |
                 XCVR_MISC_RSM_CTRL0_RSM_RX_EN(0x0U) | XCVR_MISC_RSM_CTRL0_RSM_TX_EN(0x0U) |
                 XCVR_MISC_RSM_CTRL0_RSM_FAST_IP_RX_WU(0x1U) | XCVR_MISC_RSM_CTRL0_RSM_FAST_IP_TX_WU(0x1U) |
                 XCVR_MISC_RSM_CTRL0_RSM_FAST_FC_RX_WU(0x1U) | XCVR_MISC_RSM_CTRL0_RSM_FAST_FC_TX_WU(0x1U) |
                 XCVR_MISC_RSM_CTRL0_RSM_SW_ABORT(0x0U) | XCVR_MISC_RSM_CTRL0_RSM_SN_EN(0x0U) |
                 XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL(0x6U) | /* NBU trigger */
                 XCVR_MISC_RSM_CTRL0_RSM_TRIG_DLY(0x0U) | XCVR_MISC_RSM_CTRL0_RSM_STEPS(0x0U),
    .RSM_CTRL1 = XCVR_MISC_RSM_CTRL1_RSM_T_FC(0x9BU) | XCVR_MISC_RSM_CTRL1_RSM_T_IP1(0x96U) |
                 XCVR_MISC_RSM_CTRL1_RSM_T_IP2(0x96U) | XCVR_MISC_RSM_CTRL1_RSM_T_S(0x0AU),
    .RSM_CTRL2 = XCVR_MISC_RSM_CTRL2_RSM_T_PM0(0x0U) | XCVR_MISC_RSM_CTRL2_RSM_T_PM1(0x0U) |
                 XCVR_MISC_RSM_CTRL2_RSM_RTT_TYPE(0x0U) | XCVR_MISC_RSM_CTRL2_RSM_ACTIVE_OVRD_LCL(0x0U) |
                 XCVR_MISC_RSM_CTRL2_RSM_ACTIVE_OVRD_EN_LCL(0x0U) | XCVR_MISC_RSM_CTRL2_RSM_ACTIVE_OVRD_TXDIG(0x0U) |
                 XCVR_MISC_RSM_CTRL2_RSM_ACTIVE_OVRD_EN_TXDIG(0x0U) | XCVR_MISC_RSM_CTRL2_RSM_ACTIVE_OVRD_RXDIG(0x0U) |
                 XCVR_MISC_RSM_CTRL2_RSM_ACTIVE_OVRD_EN_RXDIG(0x0U),
    .RSM_CTRL3 = XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY(0x6U) | XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DIS(0x0U) |
                 XCVR_MISC_RSM_CTRL3_RSM_AA_HAMM(0x0U) | XCVR_MISC_RSM_CTRL3_RSM_HPM_CAL(0x1U) |
                 XCVR_MISC_RSM_CTRL3_RSM_CTUNE(0x0U) | XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN(0x0U) |
                 XCVR_MISC_RSM_CTRL3_RSM_RX_PHY_EN_MASK_DIS(0x0U) | XCVR_MISC_RSM_CTRL3_RSM_RX_SIGNALS_MASK_DIS(0x0U) |
                 XCVR_MISC_RSM_CTRL3_RSM_SEQ_RCCAL_PUP_MASK_DIS(0x0U) | XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR(0x0U),
    .RSM_CTRL4 = XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY0(0xAU) | XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY(0x0U) |
                 XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0(0x5AU),
    .RSM_CTRL5 = XCVR_MISC_RSM_CTRL5_RSM_T_FM(0x50), /* T_FM is always 80usec */
    .RSM_CTRL6 = XCVR_MISC_RSM_CTRL6_RSM_RXLAT_DIG(0x0U) | XCVR_MISC_RSM_CTRL6_RSM_SKIP_RECYCLE_R2R(0x0U) |
                 XCVR_MISC_RSM_CTRL6_RSM_PKTRAM_EXTEND(0x1U) | /* Set XCVR_MISC_RSM_CTRL6_RSM_PKTRAM_EXTEND */
                 XCVR_MISC_RSM_CTRL6_RSM_EARLY_MOD_DIS(0x0U) | XCVR_MISC_RSM_CTRL6_RSM_MODE0_TIMEOUT(0x0U),
    .RSM_CTRL7 = XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR(0x0U) | XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_DELTA(0x0U) |
                 XCVR_MISC_RSM_CTRL7_RSM_TIME_CORR_MODE(0x0U) | XCVR_MISC_RSM_CTRL7_RSM_TIME_ALIGN_MODE(0x0U) |
                 XCVR_MISC_RSM_CTRL7_RSM_TIME_ALIGN_OFFSET(0x0U),

#endif
    .do_rxdig_rccal = (bool)true
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /*********** xcvr_lcl_rsm_generic_config **********/
    /***********************************************/
};
