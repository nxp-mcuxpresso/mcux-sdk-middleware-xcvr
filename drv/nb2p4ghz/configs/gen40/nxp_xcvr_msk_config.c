/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2018-2020,2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nxp_xcvr_msk_config.h"

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
const xcvr_mode_datarate_config_t xcvr_msk_1mbps_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /************ xcvr_msk_1mbps_config ************/
    /***********************************************/
    .radio_mode    = MSK,
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
    .fsk_cfg0 = GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_1M(0x1A) | GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_2M(0x1C) |
                GEN4PHY_FSK_CFG0_HAM_CHK_LOW_PWR(1) | GEN4PHY_FSK_CFG0_MSK2FSK_SEED(1) | GEN4PHY_FSK_CFG0_MSK_EN(1),

    /* FSK_CFG1 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .fsk_cfg1 = GEN4PHY_FSK_CFG1_OVERH(0x040) | GEN4PHY_FSK_CFG1_OVERH_INV(0x100) | GEN4PHY_FSK_CFG1_SYNCTSCALE(0xF),

    /* FSK_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM'] */
    .fsk_cfg2 = GEN4PHY_FSK_CFG2_MAG_THRESH_1M(0x0046) | GEN4PHY_FSK_CFG2_MAG_THRESH_HI_1M(0x0096),

    /* FSK_CFG3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_cfg3 = GEN4PHY_FSK_CFG3_MAG_THRESH_2M(0x0048) | GEN4PHY_FSK_CFG3_MAG_THRESH_HI_2M(0x00A8),

    /* FSK_PD_CFG0 configuration, dependencies: ['MD', 'MD'] */
    .fsk_pd_cfg0 = GEN4PHY_FSK_PD_CFG0_PD_IIR_ALPHA(0xFF) | GEN4PHY_FSK_PD_CFG0_PREAMBLE_T_SCALE(0xC),

    /* FSK_PD_CFG1 configuration, dependencies: ['MD'] */
    .fsk_pd_cfg1 = GEN4PHY_FSK_PD_CFG1_PREAMBLE_PATTERN(0x55),

    /* FSK_PD_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_pd_cfg2 = GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_1M(0xE6) | GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_2M(0xEB),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[0] = GEN4PHY_FSK_PD_PH_REF0(0x03) | GEN4PHY_FSK_PD_PH_REF1(0x05) | GEN4PHY_FSK_PD_PH_REF2(0x03) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[1] = GEN4PHY_FSK_PD_PH_REF0(0x3D) | GEN4PHY_FSK_PD_PH_REF1(0x3B) | GEN4PHY_FSK_PD_PH_REF2(0x3D) |
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
    .acq_filt_0_3 = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0003) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0000) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0xFFF8) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0xFFF7),
#else
    .acq_filt_0_3         = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0002) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0005) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0xFFFF) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0xFFF3),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_0_3_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0xFFFF) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0003) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x0008) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x000A),
#else
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0002) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0xFFFE) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0xFFF8) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0xFFFA),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11 = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x0066) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x00C9),
#else
    .acq_filt_10_11     = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x0061) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x00EF),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x0069) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x008E),
#else
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x006E) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x00D4),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7 = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x0006) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x001A) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0x0011) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xFFE7),
#else
    .acq_filt_4_7       = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0xFFF8) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x0014) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0x0022) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xFFF8),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x0003) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0xFFF4) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0xFFE6) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xFFE6),
#else
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x000B) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x001A) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0x000C) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xFFDF),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9 = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0xFFCA) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFFF5),
#else
    .acq_filt_8_9     = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0xFFBC) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFFD5),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0xFFFF) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0x0031),
#else
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0xFFC5) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0xFFF7),
#endif /* RF_OSC_26MHZ == 1 */

    /* AGC_TIMING0 configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing0 = XCVR_RX_DIG_AGC_TIMING0_AGC_GAIN_STEP_WAIT(0x08),

    /* AGC_TIMING0_DRS configuration, dependencies: ['MD+DR'] */
    .agc_timing0_drs = XCVR_RX_DIG_AGC_TIMING0_DRS_AGC_GAIN_STEP_WAIT(0x8),

    /* AGC_TIMING1 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing1 = XCVR_RX_DIG_AGC_TIMING1_AGC_FREEZE_TIMEOUT(0x27) | XCVR_RX_DIG_AGC_TIMING1_AGC_HOLD_TIMEOUT(0x05),

    /* AGC_TIMING1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing1_drs =
        XCVR_RX_DIG_AGC_TIMING1_DRS_AGC_FREEZE_TIMEOUT(0x0A) | XCVR_RX_DIG_AGC_TIMING1_DRS_AGC_HOLD_TIMEOUT(0x08),

    /* AGC_TIMING2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing2 = XCVR_RX_DIG_AGC_TIMING2_AGC_UNFREEZE_FEAT_TIMEOUT(0x07F) |
                   XCVR_RX_DIG_AGC_TIMING2_AGC_UNHOLD_FEAT_TIMEOUT(0x000),

    /* AGC_TIMING2_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .agc_timing2_drs = XCVR_RX_DIG_AGC_TIMING2_DRS_AGC_UNFREEZE_FEAT_TIMEOUT(0x000) |
                       XCVR_RX_DIG_AGC_TIMING2_DRS_AGC_UNHOLD_FEAT_TIMEOUT(0x000),

/* CTRL0 configuration, dependencies: ['COM', 'MD+DR', 'MD+DR', 'MD+DR', 'COM', 'MD', 'COM', 'COM', 'COM', 'COM', 'COM']
 */
#if RF_OSC_26MHZ == 1
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(2) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x03B) |
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
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0x0002) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0xFFF6),
#else
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0x0002) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0xFFF6),
#endif /* RF_OSC_26MHZ == 1 */

    /* DEMOD_FILT_0_1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1_drs = XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H0(0xFFD) | XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H1(0xFFB),

/* DEMOD_FILT_2_4 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0x0024) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x0082) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x00BA),
#else
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0x0024) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x0082) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x00BA),
#endif /* RF_OSC_26MHZ == 1 */

    /* DEMOD_FILT_2_4_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4_drs = XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H2(0x01F) | XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H3(0x086) |
                          XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H4(0x0C6),

    /* RCCAL_CTRL0 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR']
     */
    .rccal_ctrl0 = XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE(2) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_CCODE_OFFSET(0x00) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE(1) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS(0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CMPOUT_INV(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CODE_OFFSET(0x0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_SMPL_DLY(0),

    /* XCVR_TX_DIG configs */
    /***********************/

    /* DATARATE_CONFIG_FILTER_CTRL configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR',
       'MD+DR'] */
    .datarate_config_filter_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_FIR_FILTER_OVRD(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FILTER_OVRD_EN(1) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_FILTER_OVRD(0) |
#if RF_OSC_26MHZ == 1
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#else
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#endif /* RF_OSC_26MHZ == 1 */
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_FILTER_OVRD(0),

/* DATARATE_CONFIG_FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1610) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x09f0),
#else
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1800) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0800),
#endif /* RF_OSC_26MHZ == 1 */

/* DATARATE_CONFIG_GFSK_CTRL configuration, dependencies: ['MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0A28),
#else
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0840),
#endif /* RF_OSC_26MHZ == 1 */

/* FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .fsk_ctrl = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x1B0D) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x04F2),
#else
    .fsk_ctrl                  = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x1BF9) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0406),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x0048) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x0059),
#else
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x0019) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x002C),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_2_3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x006B) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x007D),
#else
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x0046) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x0069),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_4_5 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x008D) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x009A),
#else
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x0091) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x00B8),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_6_7 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x00A4) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x00A9),
#else
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x00D8) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x00EA),
#endif /* RF_OSC_26MHZ == 1 */

    /* GFSK_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .gfsk_ctrl = XCVR_TX_DIG_GFSK_CTRL_GFSK_COEFF_MAN(0) |
#if RF_OSC_26MHZ == 1
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x04F2),
#else
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x0406),
#endif /* RF_OSC_26MHZ == 1 */

    /* IMAGE_FILTER_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .image_filter_ctrl = XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FILTER_OVRD_EN(1),

    /* TXDIG_CTRL configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR'] */
    .txdig_ctrl = XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL(1) | XCVR_TX_DIG_TXDIG_CTRL_PFC_EN(1),
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /************ xcvr_msk_1mbps_config ************/
    /***********************************************/
};

const xcvr_mode_datarate_config_t xcvr_msk_500kbps_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /*********** xcvr_msk_500kbps_config ***********/
    /***********************************************/
    .radio_mode    = MSK,
    .data_rate     = DR_1MBPS,
    .alt_data_rate = DR_500KBPS,

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
    .fsk_cfg0 = GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_1M(0x12) | GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_2M(0x11) |
                GEN4PHY_FSK_CFG0_HAM_CHK_LOW_PWR(0) | GEN4PHY_FSK_CFG0_MSK2FSK_SEED(1) | GEN4PHY_FSK_CFG0_MSK_EN(1),

    /* FSK_CFG1 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .fsk_cfg1 = GEN4PHY_FSK_CFG1_OVERH(0x040) | GEN4PHY_FSK_CFG1_OVERH_INV(0x100) | GEN4PHY_FSK_CFG1_SYNCTSCALE(0xF),

    /* FSK_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM'] */
    .fsk_cfg2 = GEN4PHY_FSK_CFG2_MAG_THRESH_1M(0x0038) | GEN4PHY_FSK_CFG2_MAG_THRESH_HI_1M(0x0088),

    /* FSK_CFG3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_cfg3 = GEN4PHY_FSK_CFG3_MAG_THRESH_2M(0x002C) | GEN4PHY_FSK_CFG3_MAG_THRESH_HI_2M(0x0064),

    /* FSK_PD_CFG0 configuration, dependencies: ['MD', 'MD'] */
    .fsk_pd_cfg0 = GEN4PHY_FSK_PD_CFG0_PD_IIR_ALPHA(0xFF) | GEN4PHY_FSK_PD_CFG0_PREAMBLE_T_SCALE(0xC),

    /* FSK_PD_CFG1 configuration, dependencies: ['MD'] */
    .fsk_pd_cfg1 = GEN4PHY_FSK_PD_CFG1_PREAMBLE_PATTERN(0x55),

    /* FSK_PD_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_pd_cfg2 = GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_1M(0xF0) | GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_2M(0xEB),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[0] = GEN4PHY_FSK_PD_PH_REF0(0x02) | GEN4PHY_FSK_PD_PH_REF1(0x03) | GEN4PHY_FSK_PD_PH_REF2(0x02) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[1] = GEN4PHY_FSK_PD_PH_REF0(0x3E) | GEN4PHY_FSK_PD_PH_REF1(0x3D) | GEN4PHY_FSK_PD_PH_REF2(0x3E) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PT configuration, dependencies: ['MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .fsk_pt = GEN4PHY_FSK_PT_AGC_TIMEOUT(0x0160),

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
    .pll = XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVM(4) | XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVM_DRS(4),

    /* TX_DAC_PA configuration, dependencies: ['COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM'] */
    .tx_dac_pa = XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_CFBK_DRS(1),

    /* XCVR_MISC configs */
    /*********************/

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[0] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x300) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[1] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x000) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[0] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000120),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[1] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000000),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[0] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000120),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[1] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000000),

    /* XCVR_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM'] */
    .xcvr_ctrl = XCVR_MISC_XCVR_CTRL_DATA_RATE_DRS(2),

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
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x013B13B),
#else
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x0100000),
#endif /* RF_OSC_26MHZ == 1 */

/* XCVR_RX_DIG configs */
/***********************/

/* ACQ_FILT_0_3 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3 = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0003) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0000) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0xFFF8) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0xFFF7),
#else
    .acq_filt_0_3         = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0003) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0001) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0xFFF6) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0x0003),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_0_3_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0000) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x0000),
#else
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0000) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x0000),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11 = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x0066) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x00C9),
#else
    .acq_filt_10_11     = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0xFFFA) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x0140),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x0053) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x00E5),
#else
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0xFFB2) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x0167),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7 = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x0006) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x001A) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0x0011) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xFFE7),
#else
    .acq_filt_4_7       = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x0015) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0xFFED) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0xFFE2) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0x0036),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0xFFFC) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x0005) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0x0016) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0x0005),
#else
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x0001) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x000A) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0xFFE8) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0x0000),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9 = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0xFFCA) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFFF5),
#else
    .acq_filt_8_9     = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0x001F) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFF8F),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0xFFD2) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0xFFD5),
#else
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0x0048) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0xFFAC),
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
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(2) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x03B) |
#else
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(3) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x030) |
#endif /* RF_OSC_26MHZ == 1 */
             XCVR_RX_DIG_CTRL0_RX_ACQ_FILT_LEN(1),

/* CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(3) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x027),
#else
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(4) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x020),
#endif /* RF_OSC_26MHZ == 1 */

    /* DCOC_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM',
       'COM', 'COM', 'MD+DR', 'DR', 'COM', 'COM', 'MD+DR', 'DR'] */
    .dcoc_ctrl0 = XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_ORDER(1) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFII(0x6) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFIIP(1) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQ(0x5) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQP(1),

    /* DCOC_CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
    .dcoc_ctrl0_drs = XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFII(6) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFIIP(1) |
                      XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQ(5) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQP(1),

    /* DEMOD_FILT_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0x0002) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0xFFF6),

    /* DEMOD_FILT_0_1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1_drs = XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H0(0x0001) | XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H1(0xFFF5),

    /* DEMOD_FILT_2_4 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0xFFF3) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x008A) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x0116),

    /* DEMOD_FILT_2_4_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4_drs = XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H2(0xFFF6) | XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H3(0x008B) |
                          XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H4(0x0110),

    /* RCCAL_CTRL0 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR']
     */
    .rccal_ctrl0 = XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE(2) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS(3) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_CCODE_OFFSET(0x00) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS(1) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CMPOUT_INV(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CODE_OFFSET(0x0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_SMPL_DLY(0),

    /* XCVR_TX_DIG configs */
    /***********************/

    /* DATARATE_CONFIG_FILTER_CTRL configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR',
       'MD+DR'] */
    .datarate_config_filter_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_FIR_FILTER_OVRD(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FILTER_OVRD_EN(1) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_FILTER_OVRD(0) |
#if RF_OSC_26MHZ == 1
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#else
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#endif /* RF_OSC_26MHZ == 1 */
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_FILTER_OVRD(0),

/* DATARATE_CONFIG_FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1D80) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0240),
#else
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1DE0) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0200),
#endif /* RF_OSC_26MHZ == 1 */

/* DATARATE_CONFIG_GFSK_CTRL configuration, dependencies: ['MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0279),
#else
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0203),
#endif /* RF_OSC_26MHZ == 1 */

/* FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .fsk_ctrl = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x1B0D) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x04F2),
#else
    .fsk_ctrl                  = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x1BF9) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0406),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x0048) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x0059),
#else
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x0019) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x002C),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_2_3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x006B) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x007D),
#else
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x0046) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x0069),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_4_5 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x008D) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x009A),
#else
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x0091) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x00B8),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_6_7 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x00A4) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x00A9),
#else
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x00D8) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x00EA),
#endif /* RF_OSC_26MHZ == 1 */

    /* GFSK_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .gfsk_ctrl = XCVR_TX_DIG_GFSK_CTRL_GFSK_COEFF_MAN(0) |
#if RF_OSC_26MHZ == 1
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x04F2),
#else
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x0406),
#endif /* RF_OSC_26MHZ == 1 */

    /* IMAGE_FILTER_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .image_filter_ctrl = XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FILTER_OVRD_EN(1),

    /* TXDIG_CTRL configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR'] */
    .txdig_ctrl = XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL(1) | XCVR_TX_DIG_TXDIG_CTRL_PFC_EN(1),
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /*********** xcvr_msk_500kbps_config ***********/
    /***********************************************/
};

const xcvr_mode_datarate_config_t xcvr_msk_250kbps_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /*********** xcvr_msk_250kbps_config ***********/
    /***********************************************/
    .radio_mode    = MSK,
    .data_rate     = DR_1MBPS,
    .alt_data_rate = DR_250KBPS,

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
    .fsk_cfg0 = GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_1M(0x11) | GEN4PHY_FSK_CFG0_AA_ACQ_1_2_3_THRESH_2M(0x11) |
                GEN4PHY_FSK_CFG0_HAM_CHK_LOW_PWR(0) | GEN4PHY_FSK_CFG0_MSK2FSK_SEED(1) | GEN4PHY_FSK_CFG0_MSK_EN(1),

    /* FSK_CFG1 configuration, dependencies: ['MD', 'MD', 'MD'] */
    .fsk_cfg1 = GEN4PHY_FSK_CFG1_OVERH(0x040) | GEN4PHY_FSK_CFG1_OVERH_INV(0x100) | GEN4PHY_FSK_CFG1_SYNCTSCALE(0xF),

    /* FSK_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM'] */
    .fsk_cfg2 = GEN4PHY_FSK_CFG2_MAG_THRESH_1M(0x0038) | GEN4PHY_FSK_CFG2_MAG_THRESH_HI_1M(0x0088),

    /* FSK_CFG3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_cfg3 = GEN4PHY_FSK_CFG3_MAG_THRESH_2M(0x0020) | GEN4PHY_FSK_CFG3_MAG_THRESH_HI_2M(0x004C),

    /* FSK_PD_CFG0 configuration, dependencies: ['MD', 'MD'] */
    .fsk_pd_cfg0 = GEN4PHY_FSK_PD_CFG0_PD_IIR_ALPHA(0xFF) | GEN4PHY_FSK_PD_CFG0_PREAMBLE_T_SCALE(0xC),

    /* FSK_PD_CFG1 configuration, dependencies: ['MD'] */
    .fsk_pd_cfg1 = GEN4PHY_FSK_PD_CFG1_PREAMBLE_PATTERN(0x55),

    /* FSK_PD_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .fsk_pd_cfg2 = GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_1M(0xB8) | GEN4PHY_FSK_PD_CFG2_PD_THRESH_ACQ_1_3_2M(0xEB),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[0] = GEN4PHY_FSK_PD_PH_REF0(0x02) | GEN4PHY_FSK_PD_PH_REF1(0x03) | GEN4PHY_FSK_PD_PH_REF2(0x02) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PD_PH configuration, dependencies: ['MD', 'MD', 'MD', 'MD'] */
    .fsk_pd_ph[1] = GEN4PHY_FSK_PD_PH_REF0(0x3E) | GEN4PHY_FSK_PD_PH_REF1(0x3D) | GEN4PHY_FSK_PD_PH_REF2(0x3E) |
                    GEN4PHY_FSK_PD_PH_REF3(0x00),

    /* FSK_PT configuration, dependencies: ['MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .fsk_pt = GEN4PHY_FSK_PT_AGC_TIMEOUT(0x0100),

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
    .pll = XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVM(4) | XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVM_DRS(0),

    /* TX_DAC_PA configuration, dependencies: ['COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM'] */
    .tx_dac_pa = XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_CFBK_DRS(1),

    /* XCVR_MISC configs */
    /*********************/

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[0] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x300) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_ADDR configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .ips_fo_addr[1] =
        XCVR_MISC_IPS_FO_ADDR_ADDR(0x000) | XCVR_MISC_IPS_FO_ADDR_ENTRY_RX(0) | XCVR_MISC_IPS_FO_ADDR_ENTRY_TX(0),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[0] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000120),

    /* IPS_FO_DRS0_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs0_data[1] = XCVR_MISC_IPS_FO_DRS0_DATA_DRS0_DATA(0x00000000),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[0] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000120),

    /* IPS_FO_DRS1_DATA configuration, dependencies: ['MD+DR'] */
    .ips_fo_drs1_data[1] = XCVR_MISC_IPS_FO_DRS1_DATA_DRS1_DATA(0x00000000),

    /* XCVR_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM'] */
    .xcvr_ctrl = XCVR_MISC_XCVR_CTRL_DATA_RATE_DRS(3),

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
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x013B13B),
#else
    .data_rate_ovrd_ctrl2 = XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL2_NUM_OFFSET_CFG1(0x0100000),
#endif /* RF_OSC_26MHZ == 1 */

/* XCVR_RX_DIG configs */
/***********************/

/* ACQ_FILT_0_3 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3 = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0003) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0000) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0xFFF8) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0xFFF7),
#else
    .acq_filt_0_3         = XCVR_RX_DIG_ACQ_FILT_0_3_H0(0x0003) | XCVR_RX_DIG_ACQ_FILT_0_3_H1(0x0001) |
                    XCVR_RX_DIG_ACQ_FILT_0_3_H2(0xFFF6) | XCVR_RX_DIG_ACQ_FILT_0_3_H3(0x0003),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_0_3_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0000) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x0000),
#else
    .acq_filt_0_3_drs = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x0000) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x0000) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x0000),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11 = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0x0066) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x00C9),
#else
    .acq_filt_10_11     = XCVR_RX_DIG_ACQ_FILT_10_11_H10(0xFFFA) | XCVR_RX_DIG_ACQ_FILT_10_11_H11(0x0140),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_10_11_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x005B) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x00E8),
#else
    .acq_filt_10_11_drs = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0xFFA8) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x0164),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7 = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x0006) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0x001A) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0x0011) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0xFFE7),
#else
    .acq_filt_4_7       = XCVR_RX_DIG_ACQ_FILT_4_7_H4(0x0015) | XCVR_RX_DIG_ACQ_FILT_4_7_H5(0xFFED) |
                    XCVR_RX_DIG_ACQ_FILT_4_7_H6(0xFFE2) | XCVR_RX_DIG_ACQ_FILT_4_7_H7(0x0036),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_4_7_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0xFFFD) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x0006) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0x0012) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xFFFD),
#else
    .acq_filt_4_7_drs = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0xFFFD) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x000A) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0xFFF9) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xFFEB),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9 = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0xFFCA) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFFF5),
#else
    .acq_filt_8_9     = XCVR_RX_DIG_ACQ_FILT_8_9_H8(0x001F) | XCVR_RX_DIG_ACQ_FILT_8_9_H9(0xFF8F),
#endif /* RF_OSC_26MHZ == 1 */

/* ACQ_FILT_8_9_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0xFFCF) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0xFFDD),
#else
    .acq_filt_8_9_drs = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0x003E) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0xFFCC),
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
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_CIC_RATE(2) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x03B) |
#else
    .ctrl0 = XCVR_RX_DIG_CTRL0_CIC_ORDER(1) | XCVR_RX_DIG_CTRL0_CIC_RATE(3) | XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(0x030) |
#endif /* RF_OSC_26MHZ == 1 */
             XCVR_RX_DIG_CTRL0_RX_ACQ_FILT_LEN(1),

/* CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(0) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(4) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x027),
#else
    .ctrl0_drs = XCVR_RX_DIG_CTRL0_DRS_CIC_ORDER(1) | XCVR_RX_DIG_CTRL0_DRS_CIC_RATE(5) |
                 XCVR_RX_DIG_CTRL0_DRS_DIG_MIXER_FREQ(0x020),
#endif /* RF_OSC_26MHZ == 1 */

    /* DCOC_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM',
       'COM', 'COM', 'MD+DR', 'DR', 'COM', 'COM', 'MD+DR', 'DR'] */
    .dcoc_ctrl0 = XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_ORDER(1) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFII(0x6) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFIIP(1) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQ(0x5) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQQP(1),

    /* DCOC_CTRL0_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR'] */
    .dcoc_ctrl0_drs = XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFII(6) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFIIP(1) |
                      XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQ(5) | XCVR_RX_DIG_DCOC_CTRL0_DRS_DCOC_SFQQP(1),

    /* DEMOD_FILT_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1 = XCVR_RX_DIG_DEMOD_FILT_0_1_H0(0x0002) | XCVR_RX_DIG_DEMOD_FILT_0_1_H1(0xFFF6),

    /* DEMOD_FILT_0_1_DRS configuration, dependencies: ['MD+DR', 'MD+DR'] */
    .demod_filt_0_1_drs = XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H0(0x0002) | XCVR_RX_DIG_DEMOD_FILT_0_1_DRS_H1(0xFFF5),

    /* DEMOD_FILT_2_4 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4 = XCVR_RX_DIG_DEMOD_FILT_2_4_H2(0xFFF3) | XCVR_RX_DIG_DEMOD_FILT_2_4_H3(0x008A) |
                      XCVR_RX_DIG_DEMOD_FILT_2_4_H4(0x0116),

    /* DEMOD_FILT_2_4_DRS configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR'] */
    .demod_filt_2_4_drs = XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H2(0xFFF5) | XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H3(0x008B) |
                          XCVR_RX_DIG_DEMOD_FILT_2_4_DRS_H4(0x0112),

    /* RCCAL_CTRL0 configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR']
     */
    .rccal_ctrl0 = XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE(2) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS(4) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_CCODE_OFFSET(0x00) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS(1) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CMPOUT_INV(0) |
                   XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_CODE_OFFSET(0x0) | XCVR_RX_DIG_RCCAL_CTRL0_RCCAL_SMPL_DLY(0),

    /* XCVR_TX_DIG configs */
    /***********************/

    /* DATARATE_CONFIG_FILTER_CTRL configuration, dependencies: ['MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR', 'MD+DR',
       'MD+DR'] */
    .datarate_config_filter_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_FIR_FILTER_OVRD(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_IMAGE_FILTER_OVRD_EN(1) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_FILTER_OVRD(0) |
#if RF_OSC_26MHZ == 1
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#else
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_GFSK_FILT_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC0_CLK_SEL(0) |
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_CLK_SEL(0) |
#endif /* RF_OSC_26MHZ == 1 */
                                   XCVR_TX_DIG_DATARATE_CONFIG_FILTER_CTRL_DATARATE_CONFIG_SYNC1_FILTER_OVRD(0),

/* DATARATE_CONFIG_FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1EA0) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0120),
#else
    .datarate_config_fsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV0(0x1EE0) |
                                XCVR_TX_DIG_DATARATE_CONFIG_FSK_CTRL_DATARATE_CONFIG_FSK_FDEV1(0x0100),
#endif /* RF_OSC_26MHZ == 1 */

/* DATARATE_CONFIG_GFSK_CTRL configuration, dependencies: ['MD+DR'] */
#if RF_OSC_26MHZ == 1
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x013D),
#else
    .datarate_config_gfsk_ctrl = XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x0101),
#endif /* RF_OSC_26MHZ == 1 */

/* FSK_CTRL configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .fsk_ctrl = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x1B0D) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x04F2),
#else
    .fsk_ctrl                  = XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_0(0x1BF9) | XCVR_TX_DIG_FSK_CTRL_FSK_FDEV_1(0x0406),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_0_1 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x0048) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x0059),
#else
    .gfsk_coeff_0_1 = XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_0(0x0019) | XCVR_TX_DIG_GFSK_COEFF_0_1_GFSK_COEFF_1(0x002C),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_2_3 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x006B) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x007D),
#else
    .gfsk_coeff_2_3 = XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_2(0x0046) | XCVR_TX_DIG_GFSK_COEFF_2_3_GFSK_COEFF_3(0x0069),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_4_5 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x008D) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x009A),
#else
    .gfsk_coeff_4_5 = XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_4(0x0091) | XCVR_TX_DIG_GFSK_COEFF_4_5_GFSK_COEFF_5(0x00B8),
#endif /* RF_OSC_26MHZ == 1 */

/* GFSK_COEFF_6_7 configuration, dependencies: ['MD+DR', 'MD+DR'] */
#if RF_OSC_26MHZ == 1
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x00A4) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x00A9),
#else
    .gfsk_coeff_6_7 = XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_6(0x00D8) | XCVR_TX_DIG_GFSK_COEFF_6_7_GFSK_COEFF_7(0x00EA),
#endif /* RF_OSC_26MHZ == 1 */

    /* GFSK_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .gfsk_ctrl = XCVR_TX_DIG_GFSK_CTRL_GFSK_COEFF_MAN(0) |
#if RF_OSC_26MHZ == 1
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x04F2),
#else
                 XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x0406),
#endif /* RF_OSC_26MHZ == 1 */

    /* IMAGE_FILTER_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .image_filter_ctrl = XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FILTER_OVRD_EN(1),

    /* TXDIG_CTRL configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR'] */
    .txdig_ctrl = XCVR_TX_DIG_TXDIG_CTRL_MODULATOR_SEL(1) | XCVR_TX_DIG_TXDIG_CTRL_PFC_EN(1),
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /*********** xcvr_msk_250kbps_config ***********/
    /***********************************************/
};

/* COMPLETE CONFIG STRUCTURES */
/*! @brief  MSK mode & datarate 1Mbps with 2Mbps alternate rate configuration. */
const xcvr_config_t xcvr_msk_1mbps_full_config = {
    .common_cfg         = &xcvr_common_config,
    .mode_data_rate_cfg = &xcvr_msk_1mbps_config,
};

/*! @brief  MSK mode & datarate 1Mbps with 500Kbps alternate rate configuration. */
const xcvr_config_t xcvr_msk_500kbps_full_config = {
    .common_cfg         = &xcvr_common_config,
    .mode_data_rate_cfg = &xcvr_msk_500kbps_config,
};

/*! @brief  MSK mode & datarate 1Mbps with 250Kbps alternate rate configuration. */
const xcvr_config_t xcvr_msk_250kbps_full_config = {
    .common_cfg         = &xcvr_common_config,
    .mode_data_rate_cfg = &xcvr_msk_250kbps_config,
};
