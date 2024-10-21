/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2018-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */

#include "fsl_common.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_mode_config.h"

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

xcvrStatus_t XCVR_RadioGenRegSetup(const xcvr_config_t *xcvr_config)
{
    xcvrStatus_t status = gXcvrInvalidConfiguration_c;

    /* Check parameter */
    if (xcvr_config != NULLPTR)
    {
        /* Generated XCVR_RadioRegSetup for all XCVR registers goes here */
        /***********************************************/
        /*********** START OF GENERATED CODE ***********/
        /**************** XCVR_RadioRegSetup ***********/
        /***********************************************/
        const xcvr_common_config_t *common_cfg               = xcvr_config->common_cfg;
        const xcvr_mode_datarate_config_t *mode_datarate_cfg = xcvr_config->mode_data_rate_cfg;
        /*******************/
        /* GEN4PHY configs */
        /*******************/
        GEN4PHY->DEMOD_WAVE[0].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[0].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[0].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[0].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[0].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[0].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[1].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[1].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[1].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[1].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[1].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[1].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[2].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[2].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[2].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[2].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[2].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[2].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[3].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[3].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[3].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[3].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[3].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[3].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[4].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[4].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[4].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[4].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[4].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[4].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[5].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[5].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[5].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[5].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[5].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[5].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[6].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[6].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[6].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[6].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[6].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[6].dmd_wave_reg2;
        GEN4PHY->DEMOD_WAVE[7].DMD_WAVE_REG0 = mode_datarate_cfg->demod_wave[7].dmd_wave_reg0;
        GEN4PHY->DEMOD_WAVE[7].DMD_WAVE_REG1 = mode_datarate_cfg->demod_wave[7].dmd_wave_reg1;
        GEN4PHY->DEMOD_WAVE[7].DMD_WAVE_REG2 = mode_datarate_cfg->demod_wave[7].dmd_wave_reg2;
        GEN4PHY->DMD_CTRL0                   = common_cfg->dmd_ctrl0;
        GEN4PHY->DMD_CTRL1                   = common_cfg->dmd_ctrl1;
        GEN4PHY->FSK_CFG0                    = mode_datarate_cfg->fsk_cfg0 | common_cfg->fsk_cfg0;
        GEN4PHY->FSK_CFG1                    = mode_datarate_cfg->fsk_cfg1;
        GEN4PHY->FSK_CFG2                    = common_cfg->fsk_cfg2;
        GEN4PHY->FSK_FAD_CFG                 = common_cfg->fsk_fad_cfg;
        GEN4PHY->FSK_FAD_CTRL                = common_cfg->fsk_fad_ctrl;
        GEN4PHY->FSK_PD_CFG0                 = mode_datarate_cfg->fsk_pd_cfg0;
        GEN4PHY->FSK_PD_CFG1                 = mode_datarate_cfg->fsk_pd_cfg1;
        GEN4PHY->FSK_PD_CFG2                 = mode_datarate_cfg->fsk_pd_cfg2;
        GEN4PHY->FSK_PD_PH[0]                = mode_datarate_cfg->fsk_pd_ph[0];
        GEN4PHY->FSK_PD_PH[1]                = mode_datarate_cfg->fsk_pd_ph[1];
        GEN4PHY->FSK_PT                      = mode_datarate_cfg->fsk_pt | common_cfg->fsk_pt;
        GEN4PHY->LR_AA_CFG                   = mode_datarate_cfg->lr_aa_cfg;
        GEN4PHY->LR_PD_CFG                   = mode_datarate_cfg->lr_pd_cfg;
        GEN4PHY->LR_PD_PH[0]                 = mode_datarate_cfg->lr_pd_ph[0];
        GEN4PHY->LR_PD_PH[1]                 = mode_datarate_cfg->lr_pd_ph[1];
        GEN4PHY->LR_PD_PH[2]                 = mode_datarate_cfg->lr_pd_ph[2];
        GEN4PHY->LR_PD_PH[3]                 = mode_datarate_cfg->lr_pd_ph[3];
        GEN4PHY->MISC                        = common_cfg->misc;
        GEN4PHY->PREPHY_MISC                 = common_cfg->prephy_misc;
        GEN4PHY->RTT_CTRL                    = common_cfg->rtt_ctrl;
        GEN4PHY->RTT_REF                     = mode_datarate_cfg->rtt_ref;
        GEN4PHY->SM_CFG                      = mode_datarate_cfg->sm_cfg | common_cfg->sm_cfg;
        /**********************/
        /* RADIO_CTRL configs */
        /**********************/
        RADIO_CTRL->LL_CTRL = mode_datarate_cfg->ll_ctrl;
        /***********************/
        /* XCVR_ANALOG configs */
        /***********************/
        XCVR_ANALOG->LDO_0     = common_cfg->ldo_0;
        XCVR_ANALOG->LDO_1     = common_cfg->ldo_1;
        XCVR_ANALOG->PLL       = common_cfg->pll;
        XCVR_ANALOG->RX_0      = common_cfg->rx_0;
        XCVR_ANALOG->RX_1      = common_cfg->rx_1;
        XCVR_ANALOG->TX_DAC_PA = common_cfg->tx_dac_pa | mode_datarate_cfg->tx_dac_pa;
        XCVR_ANALOG->XO_DIST   = common_cfg->xo_dist;
        /*********************/
        /* XCVR_MISC configs */
        /*********************/
        XCVR_MISC->IPS_FO_ADDR[0]      = mode_datarate_cfg->ips_fo_addr[0];
        XCVR_MISC->IPS_FO_DRS0_DATA[0] = mode_datarate_cfg->ips_fo_drs0_data[0];
        XCVR_MISC->IPS_FO_DRS1_DATA[0] = mode_datarate_cfg->ips_fo_drs1_data[0];
        XCVR_MISC->LDO_TRIM_0          = common_cfg->ldo_trim_0;
        XCVR_MISC->LDO_TRIM_1          = common_cfg->ldo_trim_1;
        XCVR_MISC->XCVR_CTRL           = common_cfg->xcvr_ctrl | mode_datarate_cfg->xcvr_ctrl;
        /************************/
        /* XCVR_PLL_DIG configs */
        /************************/
        XCVR_PLL_DIG->CHAN_MAP             = mode_datarate_cfg->chan_map | common_cfg->chan_map;
        XCVR_PLL_DIG->CHAN_MAP_EXT         = common_cfg->chan_map_ext | mode_datarate_cfg->chan_map_ext;
        XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL1 = common_cfg->data_rate_ovrd_ctrl1;
        XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL2 = mode_datarate_cfg->data_rate_ovrd_ctrl2;
        XCVR_PLL_DIG->DELAY_MATCH          = mode_datarate_cfg->delay_match | common_cfg->delay_match;
        XCVR_PLL_DIG->HPMCAL_CTRL          = mode_datarate_cfg->hpmcal_ctrl | common_cfg->hpmcal_ctrl;
        XCVR_PLL_DIG->HPM_BUMP             = common_cfg->hpm_bump | mode_datarate_cfg->hpm_bump;
        XCVR_PLL_DIG->HPM_CAL_TIMING       = common_cfg->hpm_cal_timing;
        XCVR_PLL_DIG->HPM_CTRL             = common_cfg->hpm_ctrl;
        XCVR_PLL_DIG->HPM_SDM_RES          = mode_datarate_cfg->hpm_sdm_res | common_cfg->hpm_sdm_res;
        XCVR_PLL_DIG->LOCK_DETECT          = common_cfg->lock_detect;
        XCVR_PLL_DIG->LPM_CTRL             = common_cfg->lpm_ctrl;
        XCVR_PLL_DIG->LPM_SDM_CTRL1        = common_cfg->lpm_sdm_ctrl1;
        XCVR_PLL_DIG->MOD_CTRL             = common_cfg->mod_ctrl;
        XCVR_PLL_DIG->PLL_DATARATE_CTRL    = common_cfg->pll_datarate_ctrl;
        XCVR_PLL_DIG->TUNING_CAP_RX_CTRL   = common_cfg->tuning_cap_rx_ctrl;
        XCVR_PLL_DIG->TUNING_CAP_TX_CTRL   = common_cfg->tuning_cap_tx_ctrl;
        /***********************/
        /* XCVR_RX_DIG configs */
        /***********************/
        XCVR_RX_DIG->ACQ_FILT_0_3       = mode_datarate_cfg->acq_filt_0_3;
        XCVR_RX_DIG->ACQ_FILT_0_3_DRS   = mode_datarate_cfg->acq_filt_0_3_drs;
        XCVR_RX_DIG->ACQ_FILT_10_11     = mode_datarate_cfg->acq_filt_10_11;
        XCVR_RX_DIG->ACQ_FILT_10_11_DRS = mode_datarate_cfg->acq_filt_10_11_drs;
        XCVR_RX_DIG->ACQ_FILT_4_7       = mode_datarate_cfg->acq_filt_4_7;
        XCVR_RX_DIG->ACQ_FILT_4_7_DRS   = mode_datarate_cfg->acq_filt_4_7_drs;
        XCVR_RX_DIG->ACQ_FILT_8_9       = mode_datarate_cfg->acq_filt_8_9;
        XCVR_RX_DIG->ACQ_FILT_8_9_DRS   = mode_datarate_cfg->acq_filt_8_9_drs;
        XCVR_RX_DIG->AGC_CTRL           = common_cfg->agc_ctrl;
        XCVR_RX_DIG->AGC_CTRL_STAT      = common_cfg->agc_ctrl_stat;
        XCVR_RX_DIG->AGC_IDX0_GAIN_CFG  = common_cfg->agc_idx0_gain_cfg;
        XCVR_RX_DIG->AGC_IDX0_GAIN_VAL  = common_cfg->agc_idx0_gain_val;
        XCVR_RX_DIG->AGC_IDX0_THR       = common_cfg->agc_idx0_thr;
        XCVR_RX_DIG->AGC_IDX10_GAIN_CFG = common_cfg->agc_idx10_gain_cfg;
        XCVR_RX_DIG->AGC_IDX10_GAIN_VAL = common_cfg->agc_idx10_gain_val;
        XCVR_RX_DIG->AGC_IDX10_THR      = common_cfg->agc_idx10_thr;
        XCVR_RX_DIG->AGC_IDX11_GAIN_CFG = common_cfg->agc_idx11_gain_cfg | mode_datarate_cfg->agc_idx11_gain_cfg;
        XCVR_RX_DIG->AGC_IDX11_GAIN_VAL = common_cfg->agc_idx11_gain_val | mode_datarate_cfg->agc_idx11_gain_val;
        XCVR_RX_DIG->AGC_IDX11_THR      = common_cfg->agc_idx11_thr;
        XCVR_RX_DIG->AGC_IDX1_GAIN_CFG  = common_cfg->agc_idx1_gain_cfg;
        XCVR_RX_DIG->AGC_IDX1_GAIN_VAL  = common_cfg->agc_idx1_gain_val;
        XCVR_RX_DIG->AGC_IDX1_THR       = common_cfg->agc_idx1_thr;
        XCVR_RX_DIG->AGC_IDX2_GAIN_CFG  = common_cfg->agc_idx2_gain_cfg;
        XCVR_RX_DIG->AGC_IDX2_GAIN_VAL  = common_cfg->agc_idx2_gain_val;
        XCVR_RX_DIG->AGC_IDX2_THR       = common_cfg->agc_idx2_thr;
        XCVR_RX_DIG->AGC_IDX3_GAIN_CFG  = common_cfg->agc_idx3_gain_cfg;
        XCVR_RX_DIG->AGC_IDX3_GAIN_VAL  = common_cfg->agc_idx3_gain_val;
        XCVR_RX_DIG->AGC_IDX3_THR       = common_cfg->agc_idx3_thr;
        XCVR_RX_DIG->AGC_IDX4_GAIN_CFG  = common_cfg->agc_idx4_gain_cfg;
        XCVR_RX_DIG->AGC_IDX4_GAIN_VAL  = common_cfg->agc_idx4_gain_val;
        XCVR_RX_DIG->AGC_IDX4_THR       = common_cfg->agc_idx4_thr;
        XCVR_RX_DIG->AGC_IDX5_GAIN_CFG  = common_cfg->agc_idx5_gain_cfg;
        XCVR_RX_DIG->AGC_IDX5_GAIN_VAL  = common_cfg->agc_idx5_gain_val;
        XCVR_RX_DIG->AGC_IDX5_THR       = common_cfg->agc_idx5_thr;
        XCVR_RX_DIG->AGC_IDX6_GAIN_CFG  = common_cfg->agc_idx6_gain_cfg;
        XCVR_RX_DIG->AGC_IDX6_GAIN_VAL  = common_cfg->agc_idx6_gain_val;
        XCVR_RX_DIG->AGC_IDX6_THR       = common_cfg->agc_idx6_thr;
        XCVR_RX_DIG->AGC_IDX7_GAIN_CFG  = common_cfg->agc_idx7_gain_cfg;
        XCVR_RX_DIG->AGC_IDX7_GAIN_VAL  = common_cfg->agc_idx7_gain_val;
        XCVR_RX_DIG->AGC_IDX7_THR       = common_cfg->agc_idx7_thr;
        XCVR_RX_DIG->AGC_IDX8_GAIN_CFG  = common_cfg->agc_idx8_gain_cfg;
        XCVR_RX_DIG->AGC_IDX8_GAIN_VAL  = common_cfg->agc_idx8_gain_val;
        XCVR_RX_DIG->AGC_IDX8_THR       = common_cfg->agc_idx8_thr;
        XCVR_RX_DIG->AGC_IDX9_GAIN_CFG  = common_cfg->agc_idx9_gain_cfg;
        XCVR_RX_DIG->AGC_IDX9_GAIN_VAL  = common_cfg->agc_idx9_gain_val;
        XCVR_RX_DIG->AGC_IDX9_THR       = common_cfg->agc_idx9_thr;
        XCVR_RX_DIG->AGC_MIS_GAIN_CFG   = common_cfg->agc_mis_gain_cfg;
        XCVR_RX_DIG->AGC_THR_FAST       = common_cfg->agc_thr_fast;
        XCVR_RX_DIG->AGC_THR_FAST_DRS   = common_cfg->agc_thr_fast_drs;
        XCVR_RX_DIG->AGC_THR_MIS        = common_cfg->agc_thr_mis;
        XCVR_RX_DIG->AGC_TIMING0        = common_cfg->agc_timing0 | mode_datarate_cfg->agc_timing0;
        XCVR_RX_DIG->AGC_TIMING0_DRS    = mode_datarate_cfg->agc_timing0_drs | common_cfg->agc_timing0_drs;
        XCVR_RX_DIG->AGC_TIMING1        = mode_datarate_cfg->agc_timing1 | common_cfg->agc_timing1;
        XCVR_RX_DIG->AGC_TIMING1_DRS    = mode_datarate_cfg->agc_timing1_drs;
        XCVR_RX_DIG->AGC_TIMING2        = mode_datarate_cfg->agc_timing2 | common_cfg->agc_timing2;
        XCVR_RX_DIG->AGC_TIMING2_DRS    = mode_datarate_cfg->agc_timing2_drs;
        XCVR_RX_DIG->CTRL0              = common_cfg->ctrl0 | mode_datarate_cfg->ctrl0;
        XCVR_RX_DIG->CTRL0_DRS          = mode_datarate_cfg->ctrl0_drs;
        XCVR_RX_DIG->CTRL1              = common_cfg->ctrl1;
        XCVR_RX_DIG->CTRL2              = common_cfg->ctrl2;
        XCVR_RX_DIG->DCOC_CTRL0         = common_cfg->dcoc_ctrl0 | mode_datarate_cfg->dcoc_ctrl0;
        XCVR_RX_DIG->DCOC_CTRL0_DRS     = mode_datarate_cfg->dcoc_ctrl0_drs;
        XCVR_RX_DIG->DCOC_CTRL1         = common_cfg->dcoc_ctrl1;
        XCVR_RX_DIG->DEMOD_FILT_0_1     = mode_datarate_cfg->demod_filt_0_1;
        XCVR_RX_DIG->DEMOD_FILT_0_1_DRS = mode_datarate_cfg->demod_filt_0_1_drs;
        XCVR_RX_DIG->DEMOD_FILT_2_4     = mode_datarate_cfg->demod_filt_2_4;
        XCVR_RX_DIG->DEMOD_FILT_2_4_DRS = mode_datarate_cfg->demod_filt_2_4_drs;
        XCVR_RX_DIG->IQMC_CTRL1_DRS     = common_cfg->iqmc_ctrl1_drs;
        XCVR_RX_DIG->NB_RSSI_CTRL0      = common_cfg->nb_rssi_ctrl0;
        XCVR_RX_DIG->NB_RSSI_CTRL1      = common_cfg->nb_rssi_ctrl1;
        XCVR_RX_DIG->RCCAL_CTRL0        = mode_datarate_cfg->rccal_ctrl0;
        XCVR_RX_DIG->RSSI_GLOBAL_CTRL   = common_cfg->rssi_global_ctrl | mode_datarate_cfg->rssi_global_ctrl;
        XCVR_RX_DIG->TQI_CTRL           = common_cfg->tqi_ctrl;
        XCVR_RX_DIG->TQI_THR            = common_cfg->tqi_thr;
        XCVR_RX_DIG->WB_RSSI_CTRL       = common_cfg->wb_rssi_ctrl;
        /********************/
        /* XCVR_TSM configs */
        /********************/
        XCVR_TSM->CTRL          = common_cfg->ctrl;
        XCVR_TSM->END_OF_SEQ    = common_cfg->end_of_seq;
        XCVR_TSM->FAST_CTRL1    = common_cfg->fast_ctrl1;
        XCVR_TSM->FAST_CTRL2    = common_cfg->fast_ctrl2;
        XCVR_TSM->FAST_CTRL3    = common_cfg->fast_ctrl3;
        XCVR_TSM->RECYCLE_COUNT = common_cfg->recycle_count;
        XCVR_TSM->TIMING00      = common_cfg->timing00;
        XCVR_TSM->TIMING01      = common_cfg->timing01;
        XCVR_TSM->TIMING02      = common_cfg->timing02;
        XCVR_TSM->TIMING03      = common_cfg->timing03;
        XCVR_TSM->TIMING04      = common_cfg->timing04;
        XCVR_TSM->TIMING05      = common_cfg->timing05;
        XCVR_TSM->TIMING06      = common_cfg->timing06;
        XCVR_TSM->TIMING07      = common_cfg->timing07;
        XCVR_TSM->TIMING08      = common_cfg->timing08;
        XCVR_TSM->TIMING09      = common_cfg->timing09;
        XCVR_TSM->TIMING10      = common_cfg->timing10;
        XCVR_TSM->TIMING11      = common_cfg->timing11;
        XCVR_TSM->TIMING12      = common_cfg->timing12;
        XCVR_TSM->TIMING13      = common_cfg->timing13;
        XCVR_TSM->TIMING14      = common_cfg->timing14;
        XCVR_TSM->TIMING15      = common_cfg->timing15;
        XCVR_TSM->TIMING16      = common_cfg->timing16;
        XCVR_TSM->TIMING17      = common_cfg->timing17;
        XCVR_TSM->TIMING18      = common_cfg->timing18;
        XCVR_TSM->TIMING19      = common_cfg->timing19;
        XCVR_TSM->TIMING20      = common_cfg->timing20;
        XCVR_TSM->TIMING21      = common_cfg->timing21;
        XCVR_TSM->TIMING22      = common_cfg->timing22;
        XCVR_TSM->TIMING23      = common_cfg->timing23;
        XCVR_TSM->TIMING24      = common_cfg->timing24;
        XCVR_TSM->TIMING25      = common_cfg->timing25;
        XCVR_TSM->TIMING26      = common_cfg->timing26;
        XCVR_TSM->TIMING27      = common_cfg->timing27;
        XCVR_TSM->TIMING28      = common_cfg->timing28;
        XCVR_TSM->TIMING29      = common_cfg->timing29;
        XCVR_TSM->TIMING30      = common_cfg->timing30;
        XCVR_TSM->TIMING31      = common_cfg->timing31;
        XCVR_TSM->TIMING32      = common_cfg->timing32;
        XCVR_TSM->TIMING33      = common_cfg->timing33;
        XCVR_TSM->TIMING34      = common_cfg->timing34;
        XCVR_TSM->TIMING35      = common_cfg->timing35;
        XCVR_TSM->TIMING36      = common_cfg->timing36;
        XCVR_TSM->TIMING37      = common_cfg->timing37;
        XCVR_TSM->TIMING38      = common_cfg->timing38;
        XCVR_TSM->TIMING39      = common_cfg->timing39;
        XCVR_TSM->TIMING40      = common_cfg->timing40;
        XCVR_TSM->TIMING41      = common_cfg->timing41;
        XCVR_TSM->TIMING42      = common_cfg->timing42;
        XCVR_TSM->TIMING43      = common_cfg->timing43;
        XCVR_TSM->TIMING44      = common_cfg->timing44;
        XCVR_TSM->TIMING45      = common_cfg->timing45;
        XCVR_TSM->TIMING46      = common_cfg->timing46;
        XCVR_TSM->TIMING47      = common_cfg->timing47;
        XCVR_TSM->TIMING48      = common_cfg->timing48;
        XCVR_TSM->TIMING49      = common_cfg->timing49;
        XCVR_TSM->TIMING50      = common_cfg->timing50;
        XCVR_TSM->TIMING51      = common_cfg->timing51;
        XCVR_TSM->TIMING52      = common_cfg->timing52;
        XCVR_TSM->TIMING53      = common_cfg->timing53;
        XCVR_TSM->TIMING54      = common_cfg->timing54;
        XCVR_TSM->TIMING55      = common_cfg->timing55;
        XCVR_TSM->TIMING56      = common_cfg->timing56;
        XCVR_TSM->TIMING57      = common_cfg->timing57;
        XCVR_TSM->TIMING58      = common_cfg->timing58;
        XCVR_TSM->TIMING59      = common_cfg->timing59;
        XCVR_TSM->TIMING60      = common_cfg->timing60;
        XCVR_TSM->TIMING61      = common_cfg->timing61;
        XCVR_TSM->WU_LATENCY    = common_cfg->wu_latency;
        /***********************/
        /* XCVR_TX_DIG configs */
        /***********************/
        XCVR_TX_DIG->DATARATE_CONFIG_FILTER_CTRL = mode_datarate_cfg->datarate_config_filter_ctrl;
        XCVR_TX_DIG->DATARATE_CONFIG_FSK_CTRL    = mode_datarate_cfg->datarate_config_fsk_ctrl;
        XCVR_TX_DIG->DATARATE_CONFIG_GFSK_CTRL   = mode_datarate_cfg->datarate_config_gfsk_ctrl;
        XCVR_TX_DIG->DATA_PADDING_CTRL           = common_cfg->data_padding_ctrl | mode_datarate_cfg->data_padding_ctrl;
        XCVR_TX_DIG->DATA_PADDING_CTRL_1 = common_cfg->data_padding_ctrl_1 | mode_datarate_cfg->data_padding_ctrl_1;
        XCVR_TX_DIG->DATA_PADDING_CTRL_2 = mode_datarate_cfg->data_padding_ctrl_2;
        XCVR_TX_DIG->FSK_CTRL            = mode_datarate_cfg->fsk_ctrl;
        XCVR_TX_DIG->GFSK_COEFF_0_1      = mode_datarate_cfg->gfsk_coeff_0_1;
        XCVR_TX_DIG->GFSK_COEFF_2_3      = mode_datarate_cfg->gfsk_coeff_2_3;
        XCVR_TX_DIG->GFSK_COEFF_4_5      = mode_datarate_cfg->gfsk_coeff_4_5;
        XCVR_TX_DIG->GFSK_COEFF_6_7      = mode_datarate_cfg->gfsk_coeff_6_7;
        XCVR_TX_DIG->GFSK_CTRL           = mode_datarate_cfg->gfsk_ctrl;
        XCVR_TX_DIG->IMAGE_FILTER_CTRL   = common_cfg->image_filter_ctrl | mode_datarate_cfg->image_filter_ctrl;
        XCVR_TX_DIG->PA_CTRL             = common_cfg->pa_ctrl | mode_datarate_cfg->pa_ctrl;
        XCVR_TX_DIG->PA_RAMP_TBL0        = common_cfg->pa_ramp_tbl0;
        XCVR_TX_DIG->PA_RAMP_TBL1        = common_cfg->pa_ramp_tbl1;
        XCVR_TX_DIG->PA_RAMP_TBL2        = common_cfg->pa_ramp_tbl2;
        XCVR_TX_DIG->PA_RAMP_TBL3        = common_cfg->pa_ramp_tbl3;
        XCVR_TX_DIG->SWITCH_TX_CTRL      = common_cfg->switch_tx_ctrl;
        XCVR_TX_DIG->TXDIG_CTRL          = mode_datarate_cfg->txdig_ctrl | common_cfg->txdig_ctrl;
        /* Return status */
        status = gXcvrSuccess_c;
        /***********************************************/
        /************ END OF GENERATED CODE ************/
        /**************** XCVR_RadioRegSetup ***********/
        /***********************************************/
    }

    return status;
}

xcvrStatus_t XCVR_RadioGenRBMESetup(const xcvr_coding_config_t *rbme)
{
    xcvrStatus_t status = gXcvrInvalidConfiguration_c;

    /* Check parameter */
    if (rbme != NULLPTR)
    {
        /* Generated XCVR_RadioGenRBMESetup for all RBME registers goes here */
        /***********************************************/
        /*********** START OF GENERATED CODE ***********/
        /************** XCVR_RadioGenRBMESetup *********/
        /***********************************************/
        /*******************/
        /* GEN4PHY configs */
        /*******************/
        GEN4PHY->DMD_CTRL1 = rbme->dmd_ctrl1;
        /****************/
        /* RBME configs */
        /****************/
        RBME->CRCW_CFG        = rbme->crcw_cfg;
        RBME->CRCW_CFG2       = rbme->crcw_cfg2;
        RBME->CRCW_CFG3       = rbme->crcw_cfg3;
        RBME->CRC_INIT        = rbme->crc_init;
        RBME->CRC_PHR_SZ      = rbme->crc_phr_sz;
        RBME->CRC_POLY        = rbme->crc_poly;
        RBME->CRC_XOR_OUT     = rbme->crc_xor_out;
        RBME->FCP_CFG         = rbme->fcp_cfg;
        RBME->FEC_BSZ_OV_B4SP = rbme->fec_bsz_ov_b4sp;
        RBME->FEC_CFG1        = rbme->fec_cfg1;
        RBME->FEC_CFG2        = rbme->fec_cfg2;
        RBME->FRAME_OVER_SZ   = rbme->frame_over_sz;
        RBME->NPAYL_OVER_SZ   = rbme->npayl_over_sz;
        RBME->PKT_SZ          = rbme->pkt_sz;
        RBME->SPREAD_CFG      = rbme->spread_cfg;
        RBME->WHITEN_CFG      = rbme->whiten_cfg;
        RBME->WHITEN_POLY     = rbme->whiten_poly;
        RBME->WHITEN_SZ_THR   = rbme->whiten_sz_thr;
        /* Return status */
        status = gXcvrSuccess_c;
        /***********************************************/
        /************ END OF GENERATED CODE ************/
        /************** XCVR_RadioGenRBMESetup *********/
        /***********************************************/
    }

    return status;
}
