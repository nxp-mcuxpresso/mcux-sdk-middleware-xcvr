/*
 * Copyright 2018-2023 NXP
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
    .WU_LATENCY    = 0x00090000U,
    .RECYCLE_COUNT = 0x002919C9U,
    .FAST_CTRL1    = 0x2b802b04U,
    .FAST_CTRL2    = 0xC837BB37U,
    .FAST_CTRL3    = 0x19001900U,
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
    .TIMING16 = 0xCAC90000U,
    .TIMING17 = 0xCCC90000U,
    .TIMING18 = 0xCCC90000U,
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
    .TIMING44 = 0xCC12FFFFU,
    .TIMING45 = 0xCC12BE12U,
    .TIMING46 = 0xFFFFBE12U,
    .TIMING47 = 0xCC2ABE2AU,
    .TIMING48 = 0xCC2BBE2BU,
    .TIMING49 = 0xCC2BBE2BU,
    .TIMING50 = 0xCC2BFFFFU,
    .TIMING51 = 0xCC2BFFFFU,
    /* TIMING52 register value is not modified for RSM operation. */
    /* OVRD0 register value is not modified for RSM operation. */
    /* OVRD1 register value is not modified for RSM operation. */
    /* OVRD2 register value is not modified for RSM operation. */
    /* OVRD3 register value is not modified for RSM operation. */

    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /*********** xcvr_lcl_tsm_generic_config **********/
    /***********************************************/

};

const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_generic_config = /*!<  RSM/TSM register configuration for the generic. Meant to overlay after applying
                                     the  ::xcvr_lcl_tsm_generic_config settings */
    {
        /***********************************************/
        /*********** START OF GENERATED CODE ***********/
        /*********** xcvr_lcl_rsm_generic_config *******/
        /***********************************************/
        .RSM_CTRL0 = 0x050010f0U, .RSM_CTRL1 = 0x0a31f8e7U, .RSM_CTRL2 = 0x001c5103U,
        .RSM_CTRL3 = 0x00000106U, .RSM_CTRL4 = 0x005a000aU, .do_rxdig_rccal = (bool)true,
        /***********************************************/
        /************ END OF GENERATED CODE ************/
        /*********** xcvr_lcl_rsm_generic_config **********/
        /***********************************************/
};
