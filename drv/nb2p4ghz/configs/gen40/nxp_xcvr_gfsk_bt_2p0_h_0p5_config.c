/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2018-2020,2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nxp_xcvr_gfsk_bt_2p0_h_0p5_config.h"

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
const xcvr_mode_datarate_config_t xcvr_gfsk_bt_2p0_h_0p5_1mbps_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /***** xcvr_gfsk_bt_2p0_h_0p5_1mbps_config *****/
    /***********************************************/
    .radio_mode    = GFSK_BT_2p0_h_0p5,
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

    /* FSK_CFG0 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'MD+DR', 'MD', 'MD'] */
    .fsk_cfg0 = GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_1M(0x0F) | GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_2M(0x11) |
                GEN4PHY_FSK_CFG0_HAM_CHK_LOW_PWR(0) | GEN4PHY_FSK_CFG0_MSK2FSK_SEED(0) | GEN4PHY_FSK_CFG0_MSK_EN(0),

    /* FSK_CFG1 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .fsk_cfg1 = GEN4PHY_FSK_CFG1_OVERH(0x040) | GEN4PHY_FSK_CFG1_OVERH_INV(0x100) | GEN4PHY_FSK_CFG1_SYNCTSCALE(0xF),

    /* FSK_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM'] */
    .fsk_cfg2 = GEN4PHY_FSK_CFG2_MAG_THRESH_1M(0x0034) | GEN4PHY_FSK_CFG2_MAG_THRESH_HI_1M(0x007C),

    /* FSK_CFG3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_cfg3 = GEN4PHY_FSK_CFG3_MAG_THRESH_2M(0x004C) | GEN4PHY_FSK_CFG3_MAG_THRESH_HI_2M(0x00AC),

    /* FSK_PD_CFG0 configuration, dependencies: ['MD', 'MD'] */
    .fsk_pd_cfg0 = GEN4PHY_FSK_PD_CFG0_PD_IIR_ALPHA(0xFF) | GEN4PHY_FSK_PD_CFG0_PREAMBLE_T_SCALE(0xC),

    /* FSK_PD_CFG1 configuration, dependencies: ['MD'] */
    .fsk_pd_cfg1 = GEN4PHY_FSK_PD_CFG1_PREAMBLE_PATTERN(0x55),

    /* FSK_PD_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_pd_cfg2 = GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_1M(0xEB) | GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_2M(0xEB),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[0] = GEN4PHY_FSK_PD_PH_REF0(0x04) | GEN4PHY_FSK_PD_PH_REF1(0x06) | GEN4PHY_FSK_PD_PH_REF2(0x04) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[1] = GEN4PHY_FSK_PD_PH_REF0(0x3C) | GEN4PHY_FSK_PD_PH_REF1(0x3A) | GEN4PHY_FSK_PD_PH_REF2(0x3C) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PT configuration, dependencies: ['MD+DR', 'COM', 'COM', 'COM', 'COM'] */
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

    /* SM_CFG configuration, dependencies: ['MD', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .sm_cfg = GEN4PHY_SM_CFG_AA_TIMEOUT_UNCODED(0x0A0),

    /* RADIO_CTRL configs */
    /**********************/

    /* LL_CTRL configuration, dependencies: ['MD'] */
    .ll_ctrl = RADIO_CTRL_LL_CTRL_ACTIVE_LL(2),

    /* XCVR_ANALOG configs */
    /***********************/

    /* PLL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'DR', 'DR', 'COM'] */
    .pll = XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVM(4) | XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVM_DRS(6),

    /* TX_DAC_PA configuration, dependencies: ['COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM'] */
    .tx_dac_pa = XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_CFBK_DRS(0),

    /* XCVR_MISC configs */
    /*********************/

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[0] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x300) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(1),

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[1] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x000) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[0] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000120),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[1] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000000),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[0] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000122),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[1] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000000),

    /* XCVR_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM'] */
    .xcvr_ctrl = XCVR_MISC_XCVR_CTRL_DATA_RATE_DRS(0),

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

/* DATA_RATE_OVRD_CTRL1 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .data_rate_ovrd_ctrl1 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_CAL_SCALE_CFG1(0x9) |
                            XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_CAL_CFG1(1) |
                            XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_TX_CFG1(1) |
                            XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_LPM_SCALE_CFG1(0x9),
#else
    .data_rate_ovrd_ctrl1 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_CAL_SCALE_CFG1(0x9) |
                            XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_CAL_CFG1(1) |
                            XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_TX_CFG1(1) |
                            XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_LPM_SCALE_CFG1(0x8),
#endif /* RF_OSC_26MHZ == 1 */

/* DATA_RATE_OVRD_CTRL2 configuration, dependencies: ['MD+DR'] */
#if RF_OSC_26MHZ == 1
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x01D89D9),
#else
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x0180000),
#endif /* RF_OSC_26MHZ == 1 */

/* XCVR_RX_DIG configs */
/***********************/

/* ACQ_FILT_0_3 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3 = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0xFFFB) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0001) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0x000C) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0x0013),
#else
    .acq_filt_0_3         = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0001) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0005) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0x0002) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0xFFF4),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_0_3_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0003) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x000A) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x000C) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x0007),
#else
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0002) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0000) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0xFFF8) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0xFFF7),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11 = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x006F) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x009C),
#else
    .acq_filt_10_11     = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x005B) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x00FB),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x0069) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x0088),
#else
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x006C) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x00DB),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7 = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x000C) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0xFFF7) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0xFFDF) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xFFD8),
#else
    .acq_filt_4_7       = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0xFFF1) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x000F) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0x0028) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0x0003),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0xFFFA) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0xFFE9) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0xFFE0) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xFFE9),
#else
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x0008) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x001B) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0x0011) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xFFE2),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9 = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0xFFF2) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0x002B),
#else
    .acq_filt_8_9     = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0xFFBB) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFFC7),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0x0008) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0x0039),
#else
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0xFFC0) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0xFFEF),
#endif /* RF_OSC_26MHZ == 1 */

    /* AGC_TIMING0 configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing0 = XCVR_RX_DIG_AGC_TIMING0_AGC_GAIN_STEP_WAIT(0x10),

    /* AGC_TIMING0_DRS configuration, dependencies: ['MD+DR'] */
    .agc_timing0_drs = XCVR_RX_DIG_AGC_TIMING0_DRS_AGC_GAIN_STEP_WAIT(0x10),

    /* AGC_TIMING1 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing1 = XCVR_RX_DIG_AGC_TIMING1_AGC_FREEZE_TIMEOUT(0x0A) | XCVR_RX_DIG_AGC_TIMING1_AGC_HOLD_TIMEOUT(0x08),

    /* AGC_TIMING1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing1_drs =
        XCVR_RX_DIG_AGC_TIMING1_DRS_AGC_FREEZE_TIMEOUT(0x0A) | XCVR_RX_DIG_AGC_TIMING1_DRS_AGC_HOLD_TIMEOUT(0x08),

    /* AGC_TIMING2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing2 = XCVR_RX_DIG_AGC_TIMING2_AGC_UNFREEZE_FEAT_TIMEOUT(0x000) |
                   XCVR_RX_DIG_AGC_TIMING2_AGC_UNHOLD_FEAT_TIMEOUT(0x000),

    /* AGC_TIMING2_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing2_drs = XCVR_RX_DIG_AGC_TIMING2_DRS_AGC_UNFREEZE_FEAT_TIMEOUT(0x000) |
                       XCVR_RX_DIG_AGC_TIMING2_DRS_AGC_UNHOLD_FEAT_TIMEOUT(0x000),

/* CTRL0 configuration, dependencies: ['COM', 'MD+DR', 'MD+DR', 'MD+DR', 'COM', 'MD', 'COM', 'COM', 'COM', 'COM', 'COM']
 */
#if RF_OSC_26MHZ == 1
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(2) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x027) |
#else
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(3) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x020) |
#endif /* RF_OSC_26MHZ == 1 */
             XCVR_RX_DIG_CTRL0_RX_ACQ_FILT_LEN(0),

/* CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(1) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x03B),
#else
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(2) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x030),
#endif /* RF_OSC_26MHZ == 1 */

    /* DCOC_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM',
       'COM', 'COM', 'MD+DR', 'DR', 'COM', 'COM', 'MD+DR', 'DR'] */
    .dcoc_ctrl0 = XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_ORDER(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFII(0x8) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFIIP(1) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQ(0x2) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQP(0),

    /* DCOC_CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
    .dcoc_ctrl0_drs = XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFII(4) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFIIP(1) |
                      XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQ(1) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQP(0),

/* DEMOD_FILT_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0xFFFD) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0xFFFA),
#else
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0xFFFD) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0xFFFA),
#endif /* RF_OSC_26MHZ == 1 */

    /* DEMOD_FILT_0_1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1_drs = XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H0(0xFFFD) | XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H1(0xFFFA),

/* DEMOD_FILT_2_4 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0x001C) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x0086) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x00CA),
#else
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0x001C) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x0086) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x00CA),
#endif /* RF_OSC_26MHZ == 1 */

    /* DEMOD_FILT_2_4_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4_drs = XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H2(0x001A) | XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H3(0x0087) |
                          XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H4(0x00CE),

    /* RCCAL_CTRL0 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR']
     */
    .rccal_ctrl0 = XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE(2) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS(1) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_CCODE_OFFSET(0x00) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE(1) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS(0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CMPOUT_INV(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CODE_OFFSET(0x0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_SMPL_DLY(0),

    /* XCVR_TX_DIG configs */
    /***********************/

    /* DATARATE_CONFIG_FILTER_CTRL configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR',
       'MD+DR'] */
    .datarate_config_filter_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_FIR_FILTER_OVRD(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FILTER_OVRD_EN(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_FILTER_OVRD(0) |
#if RF_OSC_26MHZ == 1
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#else
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(1) |
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
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0A08),
#else
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0826),
#endif /* RF_OSC_26MHZ == 1 */

/* FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .fsk_ctrl = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x0000) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0000),
#else
    .fsk_ctrl                  = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x0000) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x000) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x000),
#else
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x000) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_2_3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x000) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x000),
#else
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x000) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_4_5 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x000) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x00E),
#else
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x000) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x000),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_6_7 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x0D0) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x322),
#else
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x01C) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x3E3),
#endif /* RF_OSC_26MHZ == 1 */

    /* GFSK_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .gfsk_ctrl = XCVR_TX_DIG_GFSK_CTRL_GFSK_COEFF_MAN(1) |
#if RF_OSC_26MHZ == 1
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x4EC),
#else
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x400),
#endif /* RF_OSC_26MHZ == 1 */

    /* IMAGE_FILTER_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .image_filter_ctrl = XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FILTER_OVRD_EN(0),

    /* TXDIG_CTRL configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR'] */
    .txdig_ctrl = XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL(0) | XCVR_TX_DIG_TXDIG_CTRL_PFC_EN(0),
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /***** xcvr_gfsk_bt_2p0_h_0p5_1mbps_config *****/
    /***********************************************/
};

/* COMPLETE config structures assembled from mode_datarate, mode, datarate, and common structure pointers */

/*! @brief GFSK BT=0.5, h=0.5 complete 1Mbps with 2Mbps alternate rate configuration. */
const xcvr_config_t xcvr_gfsk_bt_2p0_h_0p5_1mbps_full_config = {
    .common_cfg         = &xcvr_common_config,
    .mode_data_rate_cfg = &xcvr_gfsk_bt_2p0_h_0p5_1mbps_config,
};
