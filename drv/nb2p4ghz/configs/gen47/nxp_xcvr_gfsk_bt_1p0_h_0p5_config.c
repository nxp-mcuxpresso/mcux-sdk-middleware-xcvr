/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2018-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nxp_xcvr_gfsk_bt_1p0_h_0p5_config.h"

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

/* MODE& DATA RATE combined configuration */
const xcvr_mode_datarate_config_t xcvr_gfsk_bt_1p0_h_0p5_1mbps_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /***** xcvr_gfsk_bt_1p0_h_0p5_1mbps_config *****/
    /***********************************************/
    .radio_mode    = GFSK_BT_1p0_h_0p5,
    .data_rate     = DR_1MBPS,
    .alt_data_rate = DR_2MBPS,

    /* GEN4PHY configs */
    /*******************/

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[0].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x14) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x10) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x0C) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x08) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x04),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[0].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x3C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x38) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x34) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x30),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[0].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x2C) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x28) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x24),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[1].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x14) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x10) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x0C) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x08) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x04),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[1].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x3C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x38) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x34) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x38),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[1].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x3C) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x00) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x04),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[2].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x0C) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x08) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x04) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x00) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x3C),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[2].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x04) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x08) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x0C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x08),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[2].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x04) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x00) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x3C),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[3].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x0C) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x08) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x04) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x00) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x3C),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[3].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x04) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x08) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x0C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x10),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[3].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x14) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x18) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x1C),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[4].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x34) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x38) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x3C) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x00) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x04),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[4].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x3C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x38) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x34) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x30),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[4].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x2C) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x28) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x24),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[5].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x34) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x38) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x3C) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x00) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x04),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[5].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x3C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x38) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x34) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x38),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[5].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x3C) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x00) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x04),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[6].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x2C) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x30) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x34) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x38) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x3C),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[6].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x04) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x08) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x0C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x08),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[6].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x04) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x00) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x3C),

    /* DMD_WAVE_REG0 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[7].dmd_wave_reg0 = GEN4PHY_DMD_WAVE_REG0_SMPL0(0x2C) | GEN4PHY_DMD_WAVE_REG0_SMPL1(0x30) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL2(0x34) | GEN4PHY_DMD_WAVE_REG0_SMPL3(0x38) |
                                   GEN4PHY_DMD_WAVE_REG0_SMPL4(0x3C),

    /* DMD_WAVE_REG1 configuration, dependencies: ['MD', 'MD', 'MD', 'MD', 'MD'] */
    .demod_wave[7].dmd_wave_reg1 = GEN4PHY_DMD_WAVE_REG1_SMPL5(0x00) | GEN4PHY_DMD_WAVE_REG1_SMPL6(0x04) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL7(0x08) | GEN4PHY_DMD_WAVE_REG1_SMPL8(0x0C) |
                                   GEN4PHY_DMD_WAVE_REG1_SMPL9(0x10),

    /* DMD_WAVE_REG2 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .demod_wave[7].dmd_wave_reg2 =
        GEN4PHY_DMD_WAVE_REG2_SMPL10(0x14) | GEN4PHY_DMD_WAVE_REG2_SMPL11(0x18) | GEN4PHY_DMD_WAVE_REG2_SMPL12(0x1C),

    /* FSK_CFG0 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'MD', 'MD'] */
    .fsk_cfg0 = GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_1M(0x0F) | GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_2M(0x11) |
                GEN4PHY_FSK_CFG0_MSK2FSK_SEED(0) | GEN4PHY_FSK_CFG0_MSK_EN(0),

    /* FSK_CFG1 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .fsk_cfg1 = GEN4PHY_FSK_CFG1_OVERH(0x020) | GEN4PHY_FSK_CFG1_OVERH_INV(0x100) | GEN4PHY_FSK_CFG1_SYNCTSCALE(0xF),

    /* FSK_PD_CFG0 configuration, dependencies: ['MD', 'MD'] */
    .fsk_pd_cfg0 = GEN4PHY_FSK_PD_CFG0_PD_IIR_ALPHA(0xFF) | GEN4PHY_FSK_PD_CFG0_PREAMBLE_T_SCALE(0xC),

    /* FSK_PD_CFG1 configuration, dependencies: ['MD'] */
    .fsk_pd_cfg1 = GEN4PHY_FSK_PD_CFG1_PREAMBLE_PATTERN(0x55),

    /* FSK_PD_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_pd_cfg2 = GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_1M(0xEB) | GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_2M(0xF6),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[0] = GEN4PHY_FSK_PD_PH_REF0(0x04) | GEN4PHY_FSK_PD_PH_REF1(0x06) | GEN4PHY_FSK_PD_PH_REF2(0x04) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[1] = GEN4PHY_FSK_PD_PH_REF0(0x3C) | GEN4PHY_FSK_PD_PH_REF1(0x3A) | GEN4PHY_FSK_PD_PH_REF2(0x3C) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PT configuration, dependencies: ['MD+DR', 'COM', 'COM', 'COM'] */
    .fsk_pt = GEN4PHY_FSK_PT_AGC_TIMEOUT(0x0048),

    /* LR_AA_CFG configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .lr_aa_cfg = GEN4PHY_LR_AA_CFG_AA_COR_THRESH(0x00) | GEN4PHY_LR_AA_CFG_AA_HAM_THRESH(0x00) |
                 GEN4PHY_LR_AA_CFG_AA_LR_CORR_GAIN(0x00) | GEN4PHY_LR_AA_CFG_ACCESS_ADDR_HAM(0x00),

    /* LR_PD_CFG configuration, dependencies: ['MD', 'MD', 'MD'] */
    .lr_pd_cfg = GEN4PHY_LR_PD_CFG_CORR_TH(0x00) | GEN4PHY_LR_PD_CFG_FREQ_TH(0x00) | GEN4PHY_LR_PD_CFG_NO_PEAKS(0),

    /* LR_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .lr_pd_ph[0] = GEN4PHY_LR_PD_PH_REF0(0x00) | GEN4PHY_LR_PD_PH_REF1(0x00) | GEN4PHY_LR_PD_PH_REF2(0x00) |
                   GEN4PHY_LR_PD_PH_REF3(0x00),

    /* LR_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .lr_pd_ph[1] = GEN4PHY_LR_PD_PH_REF0(0x00) | GEN4PHY_LR_PD_PH_REF1(0x00) | GEN4PHY_LR_PD_PH_REF2(0x00) |
                   GEN4PHY_LR_PD_PH_REF3(0x00),

    /* LR_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .lr_pd_ph[2] = GEN4PHY_LR_PD_PH_REF0(0x00) | GEN4PHY_LR_PD_PH_REF1(0x00) | GEN4PHY_LR_PD_PH_REF2(0x00) |
                   GEN4PHY_LR_PD_PH_REF3(0x00),

    /* LR_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .lr_pd_ph[3] = GEN4PHY_LR_PD_PH_REF0(0x00) | GEN4PHY_LR_PD_PH_REF1(0x00) | GEN4PHY_LR_PD_PH_REF2(0x00) |
                   GEN4PHY_LR_PD_PH_REF3(0x00),

    /* RTT_REF configuration, dependencies: ['MD', 'MD', 'MD'] */
    .rtt_ref = GEN4PHY_RTT_REF_FM_REF_010(0x5e) | GEN4PHY_RTT_REF_FM_REF_110(0x6e) | GEN4PHY_RTT_REF_FM_REF_111(0x80),

    /* SM_CFG configuration, dependencies: ['MD', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .sm_cfg = GEN4PHY_SM_CFG_AA_TIMEOUT_UNCODED(0x0A0),

    /* RADIO_CTRL configs */
    /**********************/

    /* LL_CTRL configuration, dependencies: ['MD'] */
    .ll_ctrl = RADIO_CTRL_LL_CTRL_ACTIVE_LL(2),

    /* XCVR_ANALOG configs */
    /***********************/

    /* TX_DAC_PA configuration, dependencies: ['COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .tx_dac_pa = XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_CFBK_DRS(0),

    /* XCVR_MISC configs */
    /*********************/

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[0] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x370) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[1] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x02C) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[0] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000000),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[1] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000000),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[0] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000000),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[1] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000000),

    /* XCVR_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR', 'COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM',
       'COM', 'COM', 'COM'] */
    .xcvr_ctrl = XCVR_MISC_XCVR_CTRL_DATA_RATE_DRS(0) | XCVR_MISC_XCVR_CTRL_DEMOD_SEL(1) |
                 XCVR_MISC_XCVR_CTRL_LL_CFG_CAPT_DIS(0),

    /* XCVR_PLL_DIG configs */
    /************************/

    /* CHAN_MAP configuration, dependencies: ['MD', 'COM', 'COM', 'COM', 'COM'] */
    .chan_map = XCVR_PLL_DIG_CHAN_MAP_BAND_SELECT(0),

/* CHAN_MAP_EXT configuration, dependencies: ['COM', 'DR'] */
#if RF_OSC_26MHZ == 1
    .chan_map_ext = XCVR_PLL_DIG_CHAN_MAP_EXT_NUM_OFFSET(0x013B13B),
#else
    .chan_map_ext         = XCVR_PLL_DIG_CHAN_MAP_EXT_NUM_OFFSET(0x0100000),
#endif /* RF_OSC_26MHZ == 1 */

/* DATA_RATE_OVRD_CTRL2 configuration, dependencies: ['MD+DR'] */
#if RF_OSC_26MHZ == 1
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x01D89D9),
#else
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x0180000),
#endif /* RF_OSC_26MHZ == 1 */

    /* DELAY_MATCH configuration, dependencies: ['MD+DR', 'COM', 'MD+DR'] */
    .delay_match = XCVR_PLL_DIG_DELAY_MATCH_HPM_INTEGER_DELAY(0x04) |
#if RF_OSC_26MHZ == 1
                   XCVR_PLL_DIG_DELAY_MATCH_LPM_SDM_DELAY(0x02),
#else
                   XCVR_PLL_DIG_DELAY_MATCH_LPM_SDM_DELAY(0x03),
#endif /* RF_OSC_26MHZ == 1 */

    /* HPMCAL_CTRL configuration, dependencies: ['MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .hpmcal_ctrl = XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE(0),

    /* HPM_BUMP configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR', 'COM', 'COM'] */
    .hpm_bump = XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_CAL(2) | XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_TX(2),

    /* HPM_SDM_RES configuration, dependencies: ['MD+DR', 'COM'] */
    .hpm_sdm_res = XCVR_PLL_DIG_HPM_SDM_RES_HPM_COUNT_ADJUST(0x02),

/* XCVR_RX_DIG configs */
/***********************/

/* ACQ_FILT_0_3 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3 = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x3E) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x04) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0x0D) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0x10),
#else
    .acq_filt_0_3 = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x01) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x04) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0x7F) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0x73),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_0_3_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x06) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0A) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x0A) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x03),
#else
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x01) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x3E) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x78) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x7B),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11 = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x06C) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x094),
#else
    .acq_filt_10_11     = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x061) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x0EC),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x067) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x081),
#else
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x06E) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x0D1),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7 = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x06) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x73) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0xDF) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xDE),
#else
    .acq_filt_4_7       = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x79) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x14) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0x21) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xF7),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x75) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x68) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0xE3) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xF0),
#else
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x0C) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x19) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0x09) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xDD),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9 = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0x1FA) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0x030),
#else
    .acq_filt_8_9     = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0x1BD) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0x1D7),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0x00F) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0x03C),
#else
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0x1C7) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0x1FA),
#endif /* RF_OSC_26MHZ == 1 */

    /* AGC_IDX11_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'MD+DR',
       'MD+DR'] */
    .agc_idx11_gain_cfg = XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_MAG_THR_11_DRS_OFS(0x00) |
                          XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_MAG_THR_HI_11_DRS_OFS(0x00),

    /* AGC_IDX11_GAIN_VAL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .agc_idx11_gain_val =
        XCVR_RX_DIG_AGC_IDX11_GAIN_VAL_MAG_THR_11(0x000) | XCVR_RX_DIG_AGC_IDX11_GAIN_VAL_MAG_THR_HI_11(0x000),

    /* AGC_TIMING0 configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing0 = XCVR_RX_DIG_AGC_TIMING0_AGC_GAIN_STEP_WAIT(0x10),

    /* AGC_TIMING0_DRS configuration, dependencies: ['MD+DR', 'COM'] */
    .agc_timing0_drs = XCVR_RX_DIG_AGC_TIMING0_DRS_AGC_GAIN_STEP_WAIT(0x10),

    /* AGC_TIMING1 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing1 = XCVR_RX_DIG_AGC_TIMING1_AGC_FREEZE_TIMEOUT(0x0A) | XCVR_RX_DIG_AGC_TIMING1_AGC_HOLD_TIMEOUT(0x08),

    /* AGC_TIMING1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing1_drs =
        XCVR_RX_DIG_AGC_TIMING1_DRS_AGC_FREEZE_TIMEOUT(0x0A) | XCVR_RX_DIG_AGC_TIMING1_DRS_AGC_HOLD_TIMEOUT(0x08),

    /* AGC_TIMING2 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM'] */
    .agc_timing2 = XCVR_RX_DIG_AGC_TIMING2_AGC_UNFREEZE_FEAT_TIMEOUT(0x000) |
                   XCVR_RX_DIG_AGC_TIMING2_AGC_UNHOLD_FEAT_TIMEOUT(0x000),

    /* AGC_TIMING2_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing2_drs = XCVR_RX_DIG_AGC_TIMING2_DRS_AGC_UNFREEZE_FEAT_TIMEOUT(0x000) |
                       XCVR_RX_DIG_AGC_TIMING2_DRS_AGC_UNHOLD_FEAT_TIMEOUT(0x000),

/* CTRL0 configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR', 'MD+DR', 'COM', 'COM', 'MD+DR', 'COM', 'COM',
 * 'MD+DR', 'COM', 'COM', 'COM'] */
#if RF_OSC_26MHZ == 1
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(2) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x027) |
#else
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(3) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x020) |
#endif /* RF_OSC_26MHZ == 1 */
             XCVR_RX_DIG_CTRL0_RX_ACQ_FILT_LEN(0) | XCVR_RX_DIG_CTRL0_RX_FSK_ZB_SEL(0),

/* CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(1) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x03B),
#else
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(2) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x030),
#endif /* RF_OSC_26MHZ == 1 */

    /* DCOC_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'MD+DR', 'COM', 'COM',
       'COM', 'COM', 'COM', 'COM', 'MD+DR', 'MD+DR', 'COM', 'COM', 'MD+DR', 'MD+DR'] */
    .dcoc_ctrl0 = XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_ORDER(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFII(0x8) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFIIP(1) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQ(0x2) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQP(0),

    /* DCOC_CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
    .dcoc_ctrl0_drs = XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFII(4) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFIIP(1) |
                      XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQ(1) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQP(0),

/* DEMOD_FILT_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0x1FD) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0x1FB),
#else
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0x1FD) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0x1FB),
#endif /* RF_OSC_26MHZ == 1 */

    /* DEMOD_FILT_0_1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1_drs = XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H0(0x1FD) | XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H1(0x1FB),

/* DEMOD_FILT_2_4 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0x01E) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x085) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x0C6),
#else
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0x01E) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x085) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x0C6),
#endif /* RF_OSC_26MHZ == 1 */

    /* DEMOD_FILT_2_4_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4_drs = XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H2(0x01E) | XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H3(0x086) |
                          XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H4(0x0C8),

    /* RCCAL_CTRL0 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR']
     */
    .rccal_ctrl0 = XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE(2) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS(1) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_CCODE_OFFSET(0x00) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE(1) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS(0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CMPOUT_INV(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CODE_OFFSET(0x0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_SMPL_DLY(0),

    /* RSSI_GLOBAL_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .rssi_global_ctrl = XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CCA1_ED_EN(0),

    /* XCVR_TX_DIG configs */
    /***********************/

    /* DATARATE_CONFIG_FILTER_CTRL configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR',
       'MD+DR', 'MD+DR'] */
    .datarate_config_filter_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_FIR_FILTER_OVRD(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FILTER_OVRD_EN(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_FILTER_OVRD(0) |
#if RF_OSC_26MHZ == 1
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FIR_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#else
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(1) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FIR_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#endif /* RF_OSC_26MHZ == 1 */
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_FILTER_OVRD(0),

/* DATARATE_CONFIG_FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x0000) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0000),
#else
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x0000) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0000),
#endif /* RF_OSC_26MHZ == 1 */

/* DATARATE_CONFIG_GFSK_CTRL configuration, dependencies: ['MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x09FD),
#else
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x080D),
#endif /* RF_OSC_26MHZ == 1 */

    /* DATA_PADDING_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR', 'MD+DR', 'COM', 'COM'] */
    .data_padding_ctrl = XCVR_TX_DIG_DATA_PADDING_CTRL_DATA_PADDING_SEL(0) | XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY(0) |
                         XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN(1),

    /* DATA_PADDING_CTRL_1 configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .data_padding_ctrl_1 =
        XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY(0x04) | XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(0x04),

/* DATA_PADDING_CTRL_2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .data_padding_ctrl_2 =
        XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_MFDEV(0x1C00) | XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_PFDEV(0x0400),
#else
    .data_padding_ctrl_2 =
        XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_MFDEV(0x1C00) | XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_PFDEV(0x0400),
#endif /* RF_OSC_26MHZ == 1 */

/* FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .fsk_ctrl = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x0000) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0000),
#else
    .fsk_ctrl       = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x0000) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x000) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x000),
#else
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x000) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_2_3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x001) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x008),
#else
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x000) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_4_5 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x01E) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x052),
#else
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x001) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x018),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_6_7 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x0A2) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x0E3),
#else
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x08D) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x158),
#endif /* RF_OSC_26MHZ == 1 */

    /* GFSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
    .gfsk_ctrl = XCVR_TX_DIG_GFSK_CTRL_BT_EQ_OR_GTR_ONE(1) | XCVR_TX_DIG_GFSK_CTRL_GFSK_COEFF_MAN(1) |
#if RF_OSC_26MHZ == 1
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x4E4) |
#else
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x400) |
#endif /* RF_OSC_26MHZ == 1 */
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_ZERO_FDEV_EN(1),

    /* IMAGE_FILTER_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .image_filter_ctrl = XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FILTER_OVRD_EN(0),

    /* PA_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .pa_ctrl = XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL(1),

    /* TXDIG_CTRL configuration, dependencies: ['MD+DR', 'COM', 'MD+DR', 'MD+DR'] */
    .txdig_ctrl = XCVR_TX_DIG_TXDIG_CTRL_DATA_STREAM_SEL(0) | XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL(0) |
                  XCVR_TX_DIG_TXDIG_CTRL_PFC_EN(0),
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /***** xcvr_gfsk_bt_1p0_h_0p5_1mbps_config *****/
    /***********************************************/
};

/* COMPLETE config structures assembled from mode_datarate, mode, datarate, and common structure pointers */

/*! @brief GFSK BT=0.5, h=0.5 complete 1Mbps with 2Mbps alternate rate configuration. */
const xcvr_config_t xcvr_gfsk_bt_1p0_h_0p5_1mbps_full_config = {
    .common_cfg         = &xcvr_common_config,
    .mode_data_rate_cfg = &xcvr_gfsk_bt_1p0_h_0p5_1mbps_config,
};