/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2018-2020,2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "nxp_xcvr_common_config.h"

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
/*! @brief Configuration structure for settings that are common across all modes and data rates. */
const xcvr_common_config_t xcvr_common_config = {
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /************** xcvr_common_config *************/
    /***********************************************/

    /* GEN4PHY configs */
    /*******************/

    /* DMD_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .dmd_ctrl0 =
        GEN4PHY_DMD_CTRL0_DEMOD_MOD(3) | GEN4PHY_DMD_CTRL0_DREP_SCALE_FREQ(0x04) | GEN4PHY_DMD_CTRL0_DREP_SINE_EN(0) |
        GEN4PHY_DMD_CTRL0_FED_ACT_WIN(1) | GEN4PHY_DMD_CTRL0_FED_ERR_SCALE(3) | GEN4PHY_DMD_CTRL0_FERR_TRK_EN(1) |
        GEN4PHY_DMD_CTRL0_REPEAT_FACTOR(1) | GEN4PHY_DMD_CTRL0_TED_ACT_WIN(1) | GEN4PHY_DMD_CTRL0_TERR_TRK_EN(1),

    /* DMD_CTRL1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .dmd_ctrl1 = GEN4PHY_DMD_CTRL1_FED_IDLE_WIN(0x008) | GEN4PHY_DMD_CTRL1_FED_IMM_MEAS_EN(1) |
                 GEN4PHY_DMD_CTRL1_TED_ERR_SCALE(0x04) | GEN4PHY_DMD_CTRL1_TED_IDLE_WIN(0x004) |
                 GEN4PHY_DMD_CTRL1_TED_IMM_MEAS_EN(1) | GEN4PHY_DMD_CTRL1_TTRK_INT_RANGE(0x0A),

    /* FSK_CFG0 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'MD+DR', 'MD', 'MD'] */
    .fsk_cfg0 =
        GEN4PHY_FSK_CFG0_AA_OUT_SEL(1) | GEN4PHY_FSK_CFG0_BLE_NTW_ADR_THR(0) | GEN4PHY_FSK_CFG0_FSK_BIT_INVERT(0),

    /* FSK_CFG2 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM'] */
    .fsk_cfg2 = GEN4PHY_FSK_CFG2_MAG_WIN(0x04),

    /* FSK_FAD_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .fsk_fad_cfg = GEN4PHY_FSK_FAD_CFG_WIN_FAD_SEARCH_PD(0x00) | GEN4PHY_FSK_FAD_CFG_WIN_FAD_WAIT_PD(0x00) |
                   GEN4PHY_FSK_FAD_CFG_WIN_FAD_WAIT_SYNCH(0x00) | GEN4PHY_FSK_FAD_CFG_WIN_SEARCH_PD(0x00),

    /* FSK_FAD_CTRL configuration, dependencies: ['COM'] */
    .fsk_fad_ctrl = GEN4PHY_FSK_FAD_CTRL_FAD_EN(0),

    /* FSK_PT configuration, dependencies: ['MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .fsk_pt = GEN4PHY_FSK_PT_BYPASS_WITH_RSSI(0) | GEN4PHY_FSK_PT_COND_AA_BUFF_EN(0) |
              GEN4PHY_FSK_PT_COND_SIG_PRST_EN(0) | GEN4PHY_FSK_PT_POW_TH_BYPASS(0),

    /* MISC configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .misc = GEN4PHY_MISC_DMA_PAGE_SEL(0) | GEN4PHY_MISC_DTEST_MUX_EN(0) | GEN4PHY_MISC_ECO1_RSVD(0x00) |
            GEN4PHY_MISC_ECO2_RSVD(0x0) | GEN4PHY_MISC_PHY_CLK_CTRL(0x03FF) | GEN4PHY_MISC_PHY_CLK_ON(1) |
            GEN4PHY_MISC_RSSI_CORR_TH(0xA0),

    /* PREPHY_MISC configuration, dependencies: ['COM', 'COM'] */
    .prephy_misc = GEN4PHY_PREPHY_MISC_BUFF_PTR_GFSK(0x0C) | GEN4PHY_PREPHY_MISC_BUFF_PTR_LR(0x10),

    /* SM_CFG configuration, dependencies: ['MD', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .sm_cfg = GEN4PHY_SM_CFG_ACQ_MODE(2) | GEN4PHY_SM_CFG_AGC_FRZ_ON_PD_FOUND_ACQ1_LR(0) |
              GEN4PHY_SM_CFG_EARLY_PD_TIMEOUT(0x24) | GEN4PHY_SM_CFG_EN_PHY_SM_EXT_RST(0) |
#if RF_OSC_26MHZ == 1
              GEN4PHY_SM_CFG_PH_BUFF_PTR_SYM(5),
#else
              GEN4PHY_SM_CFG_PH_BUFF_PTR_SYM(3),
#endif /* RF_OSC_26MHZ == 1 */

    /* XCVR_ANALOG configs */
    /***********************/

    /* LDO_0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .ldo_0 = XCVR_ANALOG_LDO_0_BG_FORCE(0) | XCVR_ANALOG_LDO_0_LDOTRIM_TRIM_VREF(0) |
             XCVR_ANALOG_LDO_0_LDO_CAL_BYPASS(0) | XCVR_ANALOG_LDO_0_LDO_CAL_FORCE(0) |
             XCVR_ANALOG_LDO_0_LDO_CAL_PLAT_BUMP(2) | XCVR_ANALOG_LDO_0_LDO_LV_BYPASS(0) |
             XCVR_ANALOG_LDO_0_LDO_LV_TRIM(0) | XCVR_ANALOG_LDO_0_LDO_PLL_BYPASS(0) |
             XCVR_ANALOG_LDO_0_LDO_PLL_FORCE(0) | XCVR_ANALOG_LDO_0_LDO_PLL_PLAT_BUMP(2) |
             XCVR_ANALOG_LDO_0_LDO_RXTXHF_BYPASS(0) | XCVR_ANALOG_LDO_0_LDO_RXTXHF_FORCE(0) |
             XCVR_ANALOG_LDO_0_LDO_RXTXHF_PLAT_BUMP(2) | XCVR_ANALOG_LDO_0_LDO_RXTXLF_BYPASS(0) |
             XCVR_ANALOG_LDO_0_LDO_RXTXLF_FORCE(0) | XCVR_ANALOG_LDO_0_LDO_RXTXLF_PLAT_BUMP(2) |
             XCVR_ANALOG_LDO_0_LDO_VCO_BYPASS(0) | XCVR_ANALOG_LDO_0_LDO_VCO_FORCE(0) |
             XCVR_ANALOG_LDO_0_LDO_VCO_PLAT_BUMP(2),

    /* LDO_1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .ldo_1 = XCVR_ANALOG_LDO_1_LDO_ANT_BYPASS(0) | XCVR_ANALOG_LDO_1_LDO_ANT_HIZ(0) |
             XCVR_ANALOG_LDO_1_LDO_ANT_REF_SEL(0) | XCVR_ANALOG_LDO_1_LDO_ANT_TRIM(5),

    /* PLL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'DR', 'DR', 'COM'] */
    .pll = XCVR_ANALOG_PLL_PLL_FCAL_EN_STATIC_RES(0) | XCVR_ANALOG_PLL_PLL_PD_EN_VPD_PULLDN(0) |
           XCVR_ANALOG_PLL_PLL_PD_EN_VPD_PULLUP(0) | XCVR_ANALOG_PLL_PLL_PD_TRIM_FCAL_BIAS(2) |
           XCVR_ANALOG_PLL_PLL_VCO_EN_PKDET(0) |
#if RF_OSC_26MHZ == 1
           XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVT(7),
#else
           XCVR_ANALOG_PLL_PLL_VCO_TRIM_KVT(4),
#endif /* RF_OSC_26MHZ == 1 */

    /* RX_0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM'] */
    .rx_0 = XCVR_ANALOG_RX_0_ADC_INVERT_CLK(0) | XCVR_ANALOG_RX_0_ADC_TRIM(1) | XCVR_ANALOG_RX_0_RX_LNA_ITRIM(2) |
            XCVR_ANALOG_RX_0_RX_LNA_PLAT_FORCE_START(0) | XCVR_ANALOG_RX_0_RX_MIX_VBIAS(0),

    /* RX_1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM'] */
    .rx_1 = XCVR_ANALOG_RX_1_CBPF_TRIM_I(0) | XCVR_ANALOG_RX_1_CBPF_TRIM_Q(0) |
            XCVR_ANALOG_RX_1_CBPF_TRIM_SHORT_DCBIAS(2) | XCVR_ANALOG_RX_1_CBPF_TYPE(1) |
            XCVR_ANALOG_RX_1_CBPF_VCM_TRIM(2),

    /* TX_DAC_PA configuration, dependencies: ['COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM'] */
    .tx_dac_pa = XCVR_ANALOG_TX_DAC_PA_DAC_INVERT_CLK(0) | XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_CFBK(1) |
                 XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_IBIAS(0) | XCVR_ANALOG_TX_DAC_PA_DAC_TRIM_RLOAD(0) |
                 XCVR_ANALOG_TX_DAC_PA_TX_PA_VBIAS(0),

    /* XO_DIST configuration, dependencies: ['COM', 'COM', 'COM'] */
    .xo_dist = XCVR_ANALOG_XO_DIST_XO_DIST_BYPASS(0) | XCVR_ANALOG_XO_DIST_XO_DIST_FLIP(0) |
               XCVR_ANALOG_XO_DIST_XO_DIST_TRIM(0),

    /* XCVR_MISC configs */
    /*********************/

    /* LDO_TRIM_0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM'] */
    .ldo_trim_0 = XCVR_MISC_LDO_TRIM_0_LDO_CAL_TRIMSEL_OVRD(0) | XCVR_MISC_LDO_TRIM_0_LDO_PLL_TRIMSEL_OVRD(0) |
                  XCVR_MISC_LDO_TRIM_0_LDO_PLL_TRIM_OFFSET(0) | XCVR_MISC_LDO_TRIM_0_LDO_RXTXHF_TRIMSEL_OVRD(0) |
                  XCVR_MISC_LDO_TRIM_0_LDO_RXTXHF_TRIM_OFFSET(0) | XCVR_MISC_LDO_TRIM_0_LDO_RXTXLF_TRIM_OFFSET(0) |
                  XCVR_MISC_LDO_TRIM_0_LDO_SAMPLE_TRIMSEL_OVRD_EN(0) | XCVR_MISC_LDO_TRIM_0_LDO_TRIM_CMPOUT_INV(0) |
                  XCVR_MISC_LDO_TRIM_0_LDO_TRIM_SAMPLE_OVRD(0) | XCVR_MISC_LDO_TRIM_0_LDO_TRIM_SMPL_DLY(2) |
                  XCVR_MISC_LDO_TRIM_0_LDO_VCO_TRIMSEL_OVRD(0) | XCVR_MISC_LDO_TRIM_0_LDO_VCO_TRIM_OFFSET(0),

    /* LDO_TRIM_1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .ldo_trim_1 = XCVR_MISC_LDO_TRIM_1_LDO_PLL_TRIM_OVRD(0) | XCVR_MISC_LDO_TRIM_1_LDO_PLL_TRIM_OVRD_EN(0) |
                  XCVR_MISC_LDO_TRIM_1_LDO_RXTXHF_TRIM_OVRD(0) | XCVR_MISC_LDO_TRIM_1_LDO_RXTXHF_TRIM_OVRD_EN(0) |
                  XCVR_MISC_LDO_TRIM_1_LDO_RXTXLF_TRIM_OVRD(0) | XCVR_MISC_LDO_TRIM_1_LDO_RXTXLF_TRIM_OVRD_EN(0) |
                  XCVR_MISC_LDO_TRIM_1_LDO_VCO_TRIM_OVRD(40) | XCVR_MISC_LDO_TRIM_1_LDO_VCO_TRIM_OVRD_EN(1),

    /* XCVR_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM'] */
    .xcvr_ctrl = XCVR_MISC_XCVR_CTRL_DATA_RATE(1) | XCVR_MISC_XCVR_CTRL_DEMOD_SEL(1) | XCVR_MISC_XCVR_CTRL_FO_RX_EN(0) |
                 XCVR_MISC_XCVR_CTRL_FO_TX_EN(1) | XCVR_MISC_XCVR_CTRL_LL_CFG_CAPT_DIS(0) |
                 XCVR_MISC_XCVR_CTRL_TOF_RX_SEL(0) |
#if RF_OSC_26MHZ == 1
                 XCVR_MISC_XCVR_CTRL_REF_CLK_FREQ(1) |
#else
                 XCVR_MISC_XCVR_CTRL_REF_CLK_FREQ(0) |
#endif /* RF_OSC_26MHZ == 1 */
                 XCVR_MISC_XCVR_CTRL_TOF_TX_SEL(0),

    /* XCVR_PLL_DIG configs */
    /************************/

    /* BALUN_RX configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .balun_rx = XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_0(0) | XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_1(0) |
                XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_2(0) | XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_3(0) |
                XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_4(0) | XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_5(0) |
                XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_6(0) | XCVR_PLL_DIG_BALUN_RX_TUNING_RANGE_7(0),

    /* BALUN_TX configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .balun_tx = XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_0(4) | XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_1(4) |
                XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_2(3) | XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_3(1) |
                XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_4(4) | XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_5(3) |
                XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_6(1) | XCVR_PLL_DIG_BALUN_TX_TUNING_RANGE_7(0),

    /* CHAN_MAP configuration, dependencies: ['MD', 'COM', 'COM', 'COM', 'COM'] */
    .chan_map = XCVR_PLL_DIG_CHAN_MAP_BMR(0) | XCVR_PLL_DIG_CHAN_MAP_CHANNEL_NUM_OVRD(0x0000) |
                XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD(0) | XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_EN(0),

    /* CHAN_MAP_EXT configuration, dependencies: ['COM', 'DR'] */
    .chan_map_ext = XCVR_PLL_DIG_CHAN_MAP_EXT_CTUNE_TGT_OFFSET(0),

    /* CTUNE_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM'] */
    .ctune_ctrl = XCVR_PLL_DIG_CTUNE_CTRL_CTUNE_ADJUST(0xC),

    /* DELAY_MATCH configuration, dependencies: ['COM', 'COM', 'COM'] */
    .delay_match = XCVR_PLL_DIG_DELAY_MATCH_HPM_INTEGER_DELAY(0x01) | XCVR_PLL_DIG_DELAY_MATCH_HPM_SDM_DELAY(0x01) |
                   XCVR_PLL_DIG_DELAY_MATCH_LPM_SDM_DELAY(0x04),

    /* HPMCAL_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .hpmcal_ctrl = XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE(0) | XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_BUMPED(0) |
                   XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_COUNT_SCALE(0) |
                   XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_FACTOR_MANUAL(0x0000) | XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_SKIP(0) |
                   XCVR_PLL_DIG_HPMCAL_CTRL_HP_CAL_DISABLE(0),

    /* HPM_BUMP configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .hpm_bump = XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_CAL(0) | XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_TX(1) |
                XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_CAL(2) | XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_TX(2),

    /* HPM_CAL_TIMING configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .hpm_cal_timing =
        XCVR_PLL_DIG_HPM_CAL_TIMING_HPM_CAL1_SETTLE_TIME(0) | XCVR_PLL_DIG_HPM_CAL_TIMING_HPM_CAL2_SETTLE_TIME(0) |
        XCVR_PLL_DIG_HPM_CAL_TIMING_HPM_CTUNE_SETTLE_TIME(0) | XCVR_PLL_DIG_HPM_CAL_TIMING_HPM_VCO_MOD_DELAY(0),

    /* HPM_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM'] */
    .hpm_ctrl = XCVR_PLL_DIG_HPM_CTRL_HPM_CAL_INVERT(0) | XCVR_PLL_DIG_HPM_CTRL_HPM_CAL_TIME(0) |
                XCVR_PLL_DIG_HPM_CTRL_HPM_DTH_EN(1) | XCVR_PLL_DIG_HPM_CTRL_HPM_DTH_SCL(0) |
                XCVR_PLL_DIG_HPM_CTRL_HPM_INTEGER_INVERT(0) | XCVR_PLL_DIG_HPM_CTRL_HPM_LFSR_SIZE(4) |
                XCVR_PLL_DIG_HPM_CTRL_HPM_MOD_IN_INVERT(0) | XCVR_PLL_DIG_HPM_CTRL_HPM_SCALE(0) |
                XCVR_PLL_DIG_HPM_CTRL_HPM_SDM_IN_DISABLE(0) | XCVR_PLL_DIG_HPM_CTRL_HPM_SDM_IN_MANUAL(0x0000) |
                XCVR_PLL_DIG_HPM_CTRL_HPM_SDM_OUT_INVERT(0),

    /* HPM_SDM_RES configuration, dependencies: ['COM', 'COM'] */
    .hpm_sdm_res = XCVR_PLL_DIG_HPM_SDM_RES_HPM_COUNT_ADJUST(0x00) | XCVR_PLL_DIG_HPM_SDM_RES_HPM_DENOM(0x00FF),

    /* LOCK_DETECT configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .lock_detect = XCVR_PLL_DIG_LOCK_DETECT_CTUNE_LDF_LEV(0x08) | XCVR_PLL_DIG_LOCK_DETECT_FCAL_HOLD_EN(0) |
                   XCVR_PLL_DIG_LOCK_DETECT_FREQ_COUNT_GO(0) | XCVR_PLL_DIG_LOCK_DETECT_FREQ_COUNT_TIME(0) |
                   XCVR_PLL_DIG_LOCK_DETECT_FTF_RX_THRSH(0x21) | XCVR_PLL_DIG_LOCK_DETECT_FTF_TX_THRSH(0x06) |
                   XCVR_PLL_DIG_LOCK_DETECT_FTW_TXRX(7),

    /* LPM_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .lpm_ctrl = XCVR_PLL_DIG_LPM_CTRL_HPM_CAL_SCALE(0x0A) | XCVR_PLL_DIG_LPM_CTRL_LPM_DISABLE(0) |
                XCVR_PLL_DIG_LPM_CTRL_LPM_DTH_SCL(0x08) | XCVR_PLL_DIG_LPM_CTRL_LPM_D_CTRL(0) |
                XCVR_PLL_DIG_LPM_CTRL_LPM_D_OVRD(1) | XCVR_PLL_DIG_LPM_CTRL_LPM_SCALE(0x08) |
                XCVR_PLL_DIG_LPM_CTRL_LPM_SDM_INV(0) | XCVR_PLL_DIG_LPM_CTRL_LPM_SDM_USE_NEG(1) |
                XCVR_PLL_DIG_LPM_CTRL_PLL_LD_DISABLE(0) | XCVR_PLL_DIG_LPM_CTRL_PLL_LD_MANUAL(0x00),

    /* LPM_SDM_CTRL1 configuration, dependencies: ['COM', 'COM', 'COM'] */
    .lpm_sdm_ctrl1 = XCVR_PLL_DIG_LPM_SDM_CTRL1_HPM_ARRAY_BIAS(0x00) | XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_INTG(0x26) |
                     XCVR_PLL_DIG_LPM_SDM_CTRL1_SDM_MAP_DISABLE(0),

    /* MOD_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .mod_ctrl = XCVR_PLL_DIG_MOD_CTRL_HPM_MOD_DISABLE(0) | XCVR_PLL_DIG_MOD_CTRL_HPM_MOD_MANUAL(0x00) |
                XCVR_PLL_DIG_MOD_CTRL_HPM_SDM_OUT_DISABLE(1) | XCVR_PLL_DIG_MOD_CTRL_HPM_SDM_OUT_MANUAL(0) |
                XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(0x0000) | XCVR_PLL_DIG_MOD_CTRL_MOD_DISABLE(0),

    /* XCVR_RX_DIG configs */
    /***********************/

    /* AGC_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_ctrl = XCVR_RX_DIG_AGC_CTRL_AGC_DELTA_SLOW_EN(0) | XCVR_RX_DIG_AGC_CTRL_AGC_DELTA_SLOW_STEP(1) |
                XCVR_RX_DIG_AGC_CTRL_AGC_FAST_EN(1) | XCVR_RX_DIG_AGC_CTRL_AGC_FAST_STEP_UP_EN(0) |
                XCVR_RX_DIG_AGC_CTRL_AGC_HOLD_EN(2) | XCVR_RX_DIG_AGC_CTRL_AGC_SLOW_EN(1) |
                XCVR_RX_DIG_AGC_CTRL_AGC_UNHOLD_FEAT_EN(1) | XCVR_RX_DIG_AGC_CTRL_AGC_WBD_AUTO_DIS_CFG(0) |
                XCVR_RX_DIG_AGC_CTRL_AGC_WBD_EN(0) | XCVR_RX_DIG_AGC_CTRL_AGC_WBD_GAIN_LIMIT_EN(0) |
                XCVR_RX_DIG_AGC_CTRL_AGC_WBD_STEP1_DUAL_CLIP_EN(0) | XCVR_RX_DIG_AGC_CTRL_AGC_WBD_STEP1_SZ(2) |
                XCVR_RX_DIG_AGC_CTRL_AGC_WBD_STEP2_DUAL_CLIP_EN(0) | XCVR_RX_DIG_AGC_CTRL_AGC_WBD_STEP2_SZ(1) |
                XCVR_RX_DIG_AGC_CTRL_AGC_WBD_THR1(0xA) | XCVR_RX_DIG_AGC_CTRL_AGC_WBD_THR2(0xE),

    /* AGC_CTRL_STAT configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM']
     */
    .agc_ctrl_stat =
        XCVR_RX_DIG_AGC_CTRL_STAT_AGC_CALC_MAG_IN_FRZ(0) | XCVR_RX_DIG_AGC_CTRL_STAT_AGC_FREEZE_EN(1) |
        XCVR_RX_DIG_AGC_CTRL_STAT_AGC_GAIN_IDX_STORE(0) | XCVR_RX_DIG_AGC_CTRL_STAT_AGC_INIT_IDX(0xB) |
        XCVR_RX_DIG_AGC_CTRL_STAT_AGC_MAX_IDX(0) | XCVR_RX_DIG_AGC_CTRL_STAT_AGC_PHY_FREEZE_TRIG_SEL(0) |
        XCVR_RX_DIG_AGC_CTRL_STAT_AGC_PHY_HOLD_TRIG_SEL(0) | XCVR_RX_DIG_AGC_CTRL_STAT_AGC_SOFT_RST_GAIN_SEL(0) |
        XCVR_RX_DIG_AGC_CTRL_STAT_AGC_SOFT_RST_SRC_SEL(1) | XCVR_RX_DIG_AGC_CTRL_STAT_AGC_UNFREEZE_FEAT_EN(1),

    /* AGC_IDX0_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx0_gain_cfg = XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_ANT_EN_RLOAD_0(0) |
                         XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_CBPF_GAIN_0(0) | XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_LNA_ATTN_0(3) |
                         XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_LNA_HATTN_0(1) |
                         XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_LNA_HGAIN_0(0x00) |
                         XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_LNA_LGAIN_0(1) | XCVR_RX_DIG_AGC_IDX0_GAIN_CFG_LNA_RTRIM_0(1),

    /* AGC_IDX0_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx0_gain_val =
        XCVR_RX_DIG_AGC_IDX0_GAIN_VAL_LINEAR_GAIN_0(0x001) | XCVR_RX_DIG_AGC_IDX0_GAIN_VAL_LOG_GAIN_0(0x3F1),

    /* AGC_IDX0_THR configuration, dependencies: ['COM'] */
    .agc_idx0_thr = XCVR_RX_DIG_AGC_IDX0_THR_STEP_UP_THR_0(0x01E),

    /* AGC_IDX10_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx10_gain_cfg =
        XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_ANT_EN_RLOAD_10(0) | XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_CBPF_GAIN_10(1) |
        XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_LNA_ATTN_10(0) | XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_LNA_HATTN_10(0) |
        XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_LNA_HGAIN_10(0x25) | XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_LNA_LGAIN_10(0) |
        XCVR_RX_DIG_AGC_IDX10_GAIN_CFG_LNA_RTRIM_10(1),

    /* AGC_IDX10_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx10_gain_val =
        XCVR_RX_DIG_AGC_IDX10_GAIN_VAL_LINEAR_GAIN_10(0x6C4) | XCVR_RX_DIG_AGC_IDX10_GAIN_VAL_LOG_GAIN_10(0x0EB),

    /* AGC_IDX10_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx10_thr =
        XCVR_RX_DIG_AGC_IDX10_THR_STEP_DOWN_THR_10(0x096) | XCVR_RX_DIG_AGC_IDX10_THR_STEP_UP_THR_10(0x01E),

    /* AGC_IDX11_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx11_gain_cfg =
        XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_ANT_EN_RLOAD_11(0) | XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_CBPF_GAIN_11(1) |
        XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_LNA_ATTN_11(0) | XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_LNA_HATTN_11(0) |
        XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_LNA_HGAIN_11(0x3F) | XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_LNA_LGAIN_11(0) |
        XCVR_RX_DIG_AGC_IDX11_GAIN_CFG_LNA_RTRIM_11(1),

    /* AGC_IDX11_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx11_gain_val =
        XCVR_RX_DIG_AGC_IDX11_GAIN_VAL_LINEAR_GAIN_11(0xA6B) | XCVR_RX_DIG_AGC_IDX11_GAIN_VAL_LOG_GAIN_11(0x0FA),

    /* AGC_IDX11_THR configuration, dependencies: ['COM'] */
    .agc_idx11_thr = XCVR_RX_DIG_AGC_IDX11_THR_STEP_DOWN_THR_11(0x078),

    /* AGC_IDX1_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx1_gain_cfg = XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_ANT_EN_RLOAD_1(0) |
                         XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_CBPF_GAIN_1(0) | XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_LNA_ATTN_1(3) |
                         XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_LNA_HATTN_1(0) |
                         XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_LNA_HGAIN_1(0x00) |
                         XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_LNA_LGAIN_1(3) | XCVR_RX_DIG_AGC_IDX1_GAIN_CFG_LNA_RTRIM_1(1),

    /* AGC_IDX1_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx1_gain_val =
        XCVR_RX_DIG_AGC_IDX1_GAIN_VAL_LINEAR_GAIN_1(0x003) | XCVR_RX_DIG_AGC_IDX1_GAIN_VAL_LOG_GAIN_1(0x00B),

    /* AGC_IDX1_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx1_thr = XCVR_RX_DIG_AGC_IDX1_THR_STEP_DOWN_THR_1(0x15E) | XCVR_RX_DIG_AGC_IDX1_THR_STEP_UP_THR_1(0x01E),

    /* AGC_IDX2_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx2_gain_cfg = XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_ANT_EN_RLOAD_2(0) |
                         XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_CBPF_GAIN_2(1) | XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_LNA_ATTN_2(2) |
                         XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_LNA_HATTN_2(0) |
                         XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_LNA_HGAIN_2(0x00) |
                         XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_LNA_LGAIN_2(2) | XCVR_RX_DIG_AGC_IDX2_GAIN_CFG_LNA_RTRIM_2(1),

    /* AGC_IDX2_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx2_gain_val =
        XCVR_RX_DIG_AGC_IDX2_GAIN_VAL_LINEAR_GAIN_2(0x006) | XCVR_RX_DIG_AGC_IDX2_GAIN_VAL_LOG_GAIN_2(0x026),

    /* AGC_IDX2_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx2_thr = XCVR_RX_DIG_AGC_IDX2_THR_STEP_DOWN_THR_2(0x0C8) | XCVR_RX_DIG_AGC_IDX2_THR_STEP_UP_THR_2(0x01E),

    /* AGC_IDX3_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx3_gain_cfg = XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_ANT_EN_RLOAD_3(0) |
                         XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_CBPF_GAIN_3(1) | XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_LNA_ATTN_3(1) |
                         XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_LNA_HATTN_3(0) |
                         XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_LNA_HGAIN_3(0x00) |
                         XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_LNA_LGAIN_3(2) | XCVR_RX_DIG_AGC_IDX3_GAIN_CFG_LNA_RTRIM_3(1),

    /* AGC_IDX3_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx3_gain_val =
        XCVR_RX_DIG_AGC_IDX3_GAIN_VAL_LINEAR_GAIN_3(0x00B) | XCVR_RX_DIG_AGC_IDX3_GAIN_VAL_LOG_GAIN_3(0x03C),

    /* AGC_IDX3_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx3_thr = XCVR_RX_DIG_AGC_IDX3_THR_STEP_DOWN_THR_3(0x0C8) | XCVR_RX_DIG_AGC_IDX3_THR_STEP_UP_THR_3(0x01E),

    /* AGC_IDX4_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx4_gain_cfg = XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_ANT_EN_RLOAD_4(0) |
                         XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_CBPF_GAIN_4(1) | XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_LNA_ATTN_4(0) |
                         XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_LNA_HATTN_4(0) |
                         XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_LNA_HGAIN_4(0x00) |
                         XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_LNA_LGAIN_4(1) | XCVR_RX_DIG_AGC_IDX4_GAIN_CFG_LNA_RTRIM_4(1),

    /* AGC_IDX4_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx4_gain_val =
        XCVR_RX_DIG_AGC_IDX4_GAIN_VAL_LINEAR_GAIN_4(0x018) | XCVR_RX_DIG_AGC_IDX4_GAIN_VAL_LOG_GAIN_4(0x057),

    /* AGC_IDX4_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx4_thr = XCVR_RX_DIG_AGC_IDX4_THR_STEP_DOWN_THR_4(0x0C8) | XCVR_RX_DIG_AGC_IDX4_THR_STEP_UP_THR_4(0x01E),

    /* AGC_IDX5_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx5_gain_cfg = XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_ANT_EN_RLOAD_5(0) |
                         XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_CBPF_GAIN_5(1) | XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_LNA_ATTN_5(0) |
                         XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_LNA_HATTN_5(0) |
                         XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_LNA_HGAIN_5(0x00) |
                         XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_LNA_LGAIN_5(3) | XCVR_RX_DIG_AGC_IDX5_GAIN_CFG_LNA_RTRIM_5(1),

    /* AGC_IDX5_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx5_gain_val =
        XCVR_RX_DIG_AGC_IDX5_GAIN_VAL_LINEAR_GAIN_5(0x02E) | XCVR_RX_DIG_AGC_IDX5_GAIN_VAL_LOG_GAIN_5(0x06D),

    /* AGC_IDX5_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx5_thr = XCVR_RX_DIG_AGC_IDX5_THR_STEP_DOWN_THR_5(0x0FA) | XCVR_RX_DIG_AGC_IDX5_THR_STEP_UP_THR_5(0x01E),

    /* AGC_IDX6_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx6_gain_cfg = XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_ANT_EN_RLOAD_6(0) |
                         XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_CBPF_GAIN_6(1) | XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_LNA_ATTN_6(0) |
                         XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_LNA_HATTN_6(0) |
                         XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_LNA_HGAIN_6(0x01) |
                         XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_LNA_LGAIN_6(0) | XCVR_RX_DIG_AGC_IDX6_GAIN_CFG_LNA_RTRIM_6(1),

    /* AGC_IDX6_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx6_gain_val =
        XCVR_RX_DIG_AGC_IDX6_GAIN_VAL_LINEAR_GAIN_6(0x061) | XCVR_RX_DIG_AGC_IDX6_GAIN_VAL_LOG_GAIN_6(0x087),

    /* AGC_IDX6_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx6_thr = XCVR_RX_DIG_AGC_IDX6_THR_STEP_DOWN_THR_6(0x12C) | XCVR_RX_DIG_AGC_IDX6_THR_STEP_UP_THR_6(0x01E),

    /* AGC_IDX7_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx7_gain_cfg = XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_ANT_EN_RLOAD_7(0) |
                         XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_CBPF_GAIN_7(1) | XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_LNA_ATTN_7(0) |
                         XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_LNA_HATTN_7(0) |
                         XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_LNA_HGAIN_7(0x03) |
                         XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_LNA_LGAIN_7(0) | XCVR_RX_DIG_AGC_IDX7_GAIN_CFG_LNA_RTRIM_7(1),

    /* AGC_IDX7_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx7_gain_val =
        XCVR_RX_DIG_AGC_IDX7_GAIN_VAL_LINEAR_GAIN_7(0x0C8) | XCVR_RX_DIG_AGC_IDX7_GAIN_VAL_LOG_GAIN_7(0x0A0),

    /* AGC_IDX7_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx7_thr = XCVR_RX_DIG_AGC_IDX7_THR_STEP_DOWN_THR_7(0x12C) | XCVR_RX_DIG_AGC_IDX7_THR_STEP_UP_THR_7(0x01E),

    /* AGC_IDX8_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx8_gain_cfg = XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_ANT_EN_RLOAD_8(0) |
                         XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_CBPF_GAIN_8(1) | XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_LNA_ATTN_8(0) |
                         XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_LNA_HATTN_8(0) |
                         XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_LNA_HGAIN_8(0x07) |
                         XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_LNA_LGAIN_8(0) | XCVR_RX_DIG_AGC_IDX8_GAIN_CFG_LNA_RTRIM_8(1),

    /* AGC_IDX8_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx8_gain_val =
        XCVR_RX_DIG_AGC_IDX8_GAIN_VAL_LINEAR_GAIN_8(0x184) | XCVR_RX_DIG_AGC_IDX8_GAIN_VAL_LOG_GAIN_8(0x0B7),

    /* AGC_IDX8_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx8_thr = XCVR_RX_DIG_AGC_IDX8_THR_STEP_DOWN_THR_8(0x12C) | XCVR_RX_DIG_AGC_IDX8_THR_STEP_UP_THR_8(0x01E),

    /* AGC_IDX9_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .agc_idx9_gain_cfg = XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_ANT_EN_RLOAD_9(0) |
                         XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_CBPF_GAIN_9(1) | XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_LNA_ATTN_9(0) |
                         XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_LNA_HATTN_9(0) |
                         XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_LNA_HGAIN_9(0x11) |
                         XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_LNA_LGAIN_9(0) | XCVR_RX_DIG_AGC_IDX9_GAIN_CFG_LNA_RTRIM_9(1),

    /* AGC_IDX9_GAIN_VAL configuration, dependencies: ['COM', 'COM'] */
    .agc_idx9_gain_val =
        XCVR_RX_DIG_AGC_IDX9_GAIN_VAL_LINEAR_GAIN_9(0x34B) | XCVR_RX_DIG_AGC_IDX9_GAIN_VAL_LOG_GAIN_9(0x0D2),

    /* AGC_IDX9_THR configuration, dependencies: ['COM', 'COM'] */
    .agc_idx9_thr = XCVR_RX_DIG_AGC_IDX9_THR_STEP_DOWN_THR_9(0x096) | XCVR_RX_DIG_AGC_IDX9_THR_STEP_UP_THR_9(0x01E),

    /* AGC_MIS_GAIN_CFG configuration, dependencies: ['COM', 'COM', 'COM'] */
    .agc_mis_gain_cfg = XCVR_RX_DIG_AGC_MIS_GAIN_CFG_LNA_HATTN_IN_TX_MODE(0) |
                        XCVR_RX_DIG_AGC_MIS_GAIN_CFG_LNA_RTRIM_IN_DCOC_CAL(1) |
                        XCVR_RX_DIG_AGC_MIS_GAIN_CFG_LNA_RTRIM_IN_TX_MODE(0),

    /* AGC_THR_FAST configuration, dependencies: ['COM', 'COM'] */
    .agc_thr_fast =
        XCVR_RX_DIG_AGC_THR_FAST_STEP_DOWN_THR_FAST(0x190) | XCVR_RX_DIG_AGC_THR_FAST_STEP_UP_THR_FAST(0x00F),

    /* AGC_THR_FAST_DRS configuration, dependencies: ['COM', 'COM'] */
    .agc_thr_fast_drs =
        XCVR_RX_DIG_AGC_THR_FAST_DRS_STEP_DOWN_THR_FAST(0x190) | XCVR_RX_DIG_AGC_THR_FAST_DRS_STEP_UP_THR_FAST(0x01E),

    /* AGC_THR_MIS configuration, dependencies: ['COM', 'COM'] */
    .agc_thr_mis = XCVR_RX_DIG_AGC_THR_MIS_DELTA_SLOW_THR(0x028) | XCVR_RX_DIG_AGC_THR_MIS_HOLD_MARGIN_THR(0x096),

    /* AGC_TIMING0 configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing0 = XCVR_RX_DIG_AGC_TIMING0_AGC_DELTA_SLOW_WAIT(0) | XCVR_RX_DIG_AGC_TIMING0_AGC_MAG_INIT_WAIT(0x10) |
                   XCVR_RX_DIG_AGC_TIMING0_AGC_WBD_INIT_WAIT(0x04) | XCVR_RX_DIG_AGC_TIMING0_AGC_WBD_STEP1_TIMEOUT(7) |
                   XCVR_RX_DIG_AGC_TIMING0_AGC_WBD_STEP2_TIMEOUT(0x06),

    /* AGC_TIMING1 configuration, dependencies: ['MD+DR', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .agc_timing1 = XCVR_RX_DIG_AGC_TIMING1_AGC_WBD_DUAL_CLIP_TIMEOUT(0x6) |
                   XCVR_RX_DIG_AGC_TIMING1_AGC_WBD_STEP1_DUAL_CLIP_WAIT(1) |
                   XCVR_RX_DIG_AGC_TIMING1_AGC_WBD_STEP2_DUAL_CLIP_WAIT(1) |
                   XCVR_RX_DIG_AGC_TIMING1_AGC_WBD_STEP2_WAIT(0x10),

    /* CTRL0 configuration, dependencies: ['COM', 'MD+DR', 'MD+DR', 'MD+DR', 'COM', 'MD', 'COM', 'COM', 'COM', 'COM',
       'COM'] */
    .ctrl0 = XCVR_RX_DIG_CTRL0_ADC_CLIP_EN(1) | XCVR_RX_DIG_CTRL0_RX_ACQ_FILT_BYPASS(0) |
             XCVR_RX_DIG_CTRL0_RX_AGC_EN(1) | XCVR_RX_DIG_CTRL0_RX_DIG_GAIN(6) | XCVR_RX_DIG_CTRL0_RX_FSK_ZB_SEL(0) |
             XCVR_RX_DIG_CTRL0_RX_IQMC_EN(1) |
#if RF_OSC_26MHZ == 1
             XCVR_RX_DIG_CTRL0_RX_SRC_EN(1),
#else
             XCVR_RX_DIG_CTRL0_RX_SRC_EN(0),
#endif /* RF_OSC_26MHZ == 1 */

    /* CTRL1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .ctrl1 = XCVR_RX_DIG_CTRL1_RX_CFO_EST_OVRD(0) | XCVR_RX_DIG_CTRL1_RX_CFO_EST_OVRD_EN(0) |
             XCVR_RX_DIG_CTRL1_RX_DEMOD_FILT_BYPASS(0) | XCVR_RX_DIG_CTRL1_RX_FRAC_CORR_OVRD(0) |
             XCVR_RX_DIG_CTRL1_RX_FRAC_CORR_OVRD_EN(0) | XCVR_RX_DIG_CTRL1_RX_HIGH_RES_NORM_EN(0) |
             XCVR_RX_DIG_CTRL1_RX_NB_NORM_EN(1) | XCVR_RX_DIG_CTRL1_RX_SAMPLE_BUF_BYPASS(0),

    /* DCOC_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'MD+DR', 'COM', 'COM', 'COM',
       'COM', 'COM', 'MD+DR', 'DR', 'COM', 'COM', 'MD+DR', 'DR'] */
    .dcoc_ctrl0 = XCVR_RX_DIG_DCOC_CTRL0_DCOC_ADC_OFFSET_OVRD_EN(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_CAL_USE_OFFSET(0) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_CBPF_HIZ_OVRD(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_CBPF_HIZ_SHORT_OVRD_EN(0) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_CBPF_SHORT_OVRD(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_CBPF_STL_TIME(3) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_OVRD_EN(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_I_CAL_POL(0) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_PULSE_CAPCODE(0) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_Q_CAL_POL(1) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SAR_STL_TIME(3) | XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFIQ(1) |
                  XCVR_RX_DIG_DCOC_CTRL0_DCOC_SFQI(1),

    /* DCOC_CTRL1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .dcoc_ctrl1 = XCVR_RX_DIG_DCOC_CTRL1_DCOC_ICBPF_OFFSET(0x00) | XCVR_RX_DIG_DCOC_CTRL1_DCOC_ILNA_OFFSET(0x00) |
                  XCVR_RX_DIG_DCOC_CTRL1_DCOC_QCBPF_OFFSET(0x00) | XCVR_RX_DIG_DCOC_CTRL1_DCOC_QLNA_OFFSET(0x00),

    /* NB_RSSI_CTRL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .nb_rssi_ctrl0 = XCVR_RX_DIG_NB_RSSI_CTRL0_KEEP_RSSI_RESULT_NB(1) | XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_ADJ_NB(0xED) |
                     XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_IIR_WAIT_NB(2) | XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_IIR_WT_NB(2) |
                     XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_M_WINDOW_NB(0x4) | XCVR_RX_DIG_NB_RSSI_CTRL0_RSSI_N_WINDOW_NB(0x4) |
                     XCVR_RX_DIG_NB_RSSI_CTRL0_SNR_ADJ_NB(0x07),

    /* NB_RSSI_CTRL1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .nb_rssi_ctrl1 = XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_BIAS(0x0) | XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_SENS_ADJ(0x0) |
                     XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_RSSI_WEIGHT(0) | XCVR_RX_DIG_NB_RSSI_CTRL1_LQI_SNR_WEIGHT(0x0),

    /* RSSI_GLOBAL_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM',
       'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .rssi_global_ctrl =
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_CCA1_ED_FROM_NB(1) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CCA1_ED_EN(0) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CONT_MEAS_OVRD(0) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_CONT_MEAS_OVRD_EN(0) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_AA_MATCH_OVRD(0) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_AA_MATCH_OVRD_EN(0) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_EN(1) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_INPUT_SEL(0) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_RSSI_PA_AA_MATCH_SEL(1) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_NB_SNR_LQI_ENABLE(1) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_WB_CCA1_ED_EN(0) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_WB_CONT_MEAS_OVRD(0) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_WB_CONT_MEAS_OVRD_EN(0) |
        XCVR_RX_DIG_RSSI_GLOBAL_CTRL_WB_RSSI_EN(1) | XCVR_RX_DIG_RSSI_GLOBAL_CTRL_WB_RSSI_INPUT_SEL(0),

    /* WB_RSSI_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .wb_rssi_ctrl = XCVR_RX_DIG_WB_RSSI_CTRL_KEEP_RSSI_RESULT_WB(1) | XCVR_RX_DIG_WB_RSSI_CTRL_RSSI_ADJ_WB(0x00) |
                    XCVR_RX_DIG_WB_RSSI_CTRL_RSSI_DB_EN_WB(1) | XCVR_RX_DIG_WB_RSSI_CTRL_RSSI_F_WINDOW_WB(3) |
                    XCVR_RX_DIG_WB_RSSI_CTRL_RSSI_M_WINDOW_WB(4) | XCVR_RX_DIG_WB_RSSI_CTRL_RSSI_N_WINDOW_WB(2),

    /* XCVR_TSM configs */
    /********************/

    /* CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM']
     */
    .ctrl = XCVR_TSM_CTRL_ABORT_ON_CTUNE(0) | XCVR_TSM_CTRL_ABORT_ON_FREQ_TARG(0) | XCVR_TSM_CTRL_BKPT(0xFF) |
            XCVR_TSM_CTRL_FORCE_RX_EN(0) | XCVR_TSM_CTRL_FORCE_TX_EN(0) | XCVR_TSM_CTRL_RX_ABORT_DIS(0) |
            XCVR_TSM_CTRL_TSM_IRQ0_EN(0) | XCVR_TSM_CTRL_TSM_IRQ1_EN(0) | XCVR_TSM_CTRL_TSM_LL_INHIBIT(0) |
            XCVR_TSM_CTRL_TSM_SOFT_RESET(0) | XCVR_TSM_CTRL_TX_ABORT_DIS(0),

    /* END_OF_SEQ configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .end_of_seq = XCVR_TSM_END_OF_SEQ_END_OF_RX_WD(0x5A) | XCVR_TSM_END_OF_SEQ_END_OF_RX_WU(0x58) |
                  XCVR_TSM_END_OF_SEQ_END_OF_TX_WD(0x72) | XCVR_TSM_END_OF_SEQ_END_OF_TX_WU(0x70),

    /* FAST_CTRL1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .fast_ctrl1 = XCVR_TSM_FAST_CTRL1_FAST_RX2TX_EN(0) | XCVR_TSM_FAST_CTRL1_FAST_RX2TX_START(0x00) |
                  XCVR_TSM_FAST_CTRL1_FAST_RX_WU_EN(0) | XCVR_TSM_FAST_CTRL1_FAST_TX_WU_EN(0) |
                  XCVR_TSM_FAST_CTRL1_PWRSAVE_RX_WU_EN(0) | XCVR_TSM_FAST_CTRL1_PWRSAVE_TX_WU_EN(0) |
                  XCVR_TSM_FAST_CTRL1_PWRSAVE_WU_CLEAR(0),

    /* FAST_CTRL2 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .fast_ctrl2 = XCVR_TSM_FAST_CTRL2_FAST_DEST_RX(0x66) | XCVR_TSM_FAST_CTRL2_FAST_DEST_TX(0x42) |
                  XCVR_TSM_FAST_CTRL2_FAST_START_RX(0x28) | XCVR_TSM_FAST_CTRL2_FAST_START_TX(0x08),

    /* RECYCLE_COUNT configuration, dependencies: ['COM', 'COM'] */
    .recycle_count = XCVR_TSM_RECYCLE_COUNT_RECYCLE_COUNT0(0x56) | XCVR_TSM_RECYCLE_COUNT_RECYCLE_COUNT1(0x12),

    /* TIMING00 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing00 = XCVR_TSM_TIMING00_RF_ACTIVE_RX_HI(0xFF) | XCVR_TSM_TIMING00_RF_ACTIVE_RX_LO(0xFF) |
                XCVR_TSM_TIMING00_RF_ACTIVE_TX_HI(0xFF) | XCVR_TSM_TIMING00_RF_ACTIVE_TX_LO(0xFF),

    /* TIMING01 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing01 = XCVR_TSM_TIMING01_RF_STATUS_RX_HI(0xFF) | XCVR_TSM_TIMING01_RF_STATUS_RX_LO(0xFF) |
                XCVR_TSM_TIMING01_RF_STATUS_TX_HI(0xFF) | XCVR_TSM_TIMING01_RF_STATUS_TX_LO(0xFF),

    /* TIMING02 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing02 = XCVR_TSM_TIMING02_RF_PRIORITY_RX_HI(0xFF) | XCVR_TSM_TIMING02_RF_PRIORITY_RX_LO(0xFF) |
                XCVR_TSM_TIMING02_RF_PRIORITY_TX_HI(0xFF) | XCVR_TSM_TIMING02_RF_PRIORITY_TX_LO(0xFF),

    /* TIMING03 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing03 = XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_HI(0xFF) | XCVR_TSM_TIMING03_IRQ0_START_TRIG_RX_LO(0xFF) |
                XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_HI(0xFF) | XCVR_TSM_TIMING03_IRQ0_START_TRIG_TX_LO(0xFF),

    /* TIMING04 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing04 = XCVR_TSM_TIMING04_IRQ1_STOP_TRIG_RX_HI(0xFF) | XCVR_TSM_TIMING04_IRQ1_STOP_TRIG_RX_LO(0xFF) |
                XCVR_TSM_TIMING04_IRQ1_STOP_TRIG_TX_HI(0xFF) | XCVR_TSM_TIMING04_IRQ1_STOP_TRIG_TX_LO(0xFF),

    /* TIMING05 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing05 = XCVR_TSM_TIMING05_GPIO0_TRIG_EN_RX_HI(0xFF) | XCVR_TSM_TIMING05_GPIO0_TRIG_EN_RX_LO(0xFF) |
                XCVR_TSM_TIMING05_GPIO0_TRIG_EN_TX_HI(0xFF) | XCVR_TSM_TIMING05_GPIO0_TRIG_EN_TX_LO(0xFF),

    /* TIMING06 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing06 = XCVR_TSM_TIMING06_GPIO1_TRIG_EN_RX_HI(0xFF) | XCVR_TSM_TIMING06_GPIO1_TRIG_EN_RX_LO(0xFF) |
                XCVR_TSM_TIMING06_GPIO1_TRIG_EN_TX_HI(0xFF) | XCVR_TSM_TIMING06_GPIO1_TRIG_EN_TX_LO(0xFF),

    /* TIMING07 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing07 = XCVR_TSM_TIMING07_GPIO2_TRIG_EN_RX_HI(0xFF) | XCVR_TSM_TIMING07_GPIO2_TRIG_EN_RX_LO(0xFF) |
                XCVR_TSM_TIMING07_GPIO2_TRIG_EN_TX_HI(0xFF) | XCVR_TSM_TIMING07_GPIO2_TRIG_EN_TX_LO(0xFF),

    /* TIMING08 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing08 = XCVR_TSM_TIMING08_GPIO3_TRIG_EN_RX_HI(0xFF) | XCVR_TSM_TIMING08_GPIO3_TRIG_EN_RX_LO(0xFF) |
                XCVR_TSM_TIMING08_GPIO3_TRIG_EN_TX_HI(0xFF) | XCVR_TSM_TIMING08_GPIO3_TRIG_EN_TX_LO(0xFF),

    /* TIMING09 configuration, dependencies: ['COM', 'COM'] */
    .timing09 = XCVR_TSM_TIMING09_DCOC_GAIN_CFG_EN_RX_HI(0x00) | XCVR_TSM_TIMING09_DCOC_GAIN_CFG_EN_RX_LO(0x55),

    /* TIMING10 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing10 = XCVR_TSM_TIMING10_LDO_CAL_EN_RX_HI(0x08) | XCVR_TSM_TIMING10_LDO_CAL_EN_RX_LO(0x11) |
                XCVR_TSM_TIMING10_LDO_CAL_EN_TX_HI(0x08) | XCVR_TSM_TIMING10_LDO_CAL_EN_TX_LO(0x11),

    /* TIMING11 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing11 = XCVR_TSM_TIMING11_PLL_DIG_EN_RX_HI(0x13) | XCVR_TSM_TIMING11_PLL_DIG_EN_RX_LO(0x59) |
                XCVR_TSM_TIMING11_PLL_DIG_EN_TX_HI(0x13) | XCVR_TSM_TIMING11_PLL_DIG_EN_TX_LO(0x71),

    /* TIMING12 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing12 = XCVR_TSM_TIMING12_SIGMA_DELTA_EN_RX_HI(0x19) | XCVR_TSM_TIMING12_SIGMA_DELTA_EN_RX_LO(0x59) |
                XCVR_TSM_TIMING12_SIGMA_DELTA_EN_TX_HI(0x4F) | XCVR_TSM_TIMING12_SIGMA_DELTA_EN_TX_LO(0x71),

    /* TIMING13 configuration, dependencies: ['COM', 'COM'] */
    .timing13 = XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_HI(0x2D) | XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_LO(0x55),

    /* TIMING14 configuration, dependencies: ['COM', 'COM'] */
    .timing14 = XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI(0x6F) | XCVR_TSM_TIMING14_TX_DIG_EN_TX_LO(0x71),

    /* TIMING15 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing15 = XCVR_TSM_TIMING15_FREQ_TARG_LD_EN_RX_HI(0x56) | XCVR_TSM_TIMING15_FREQ_TARG_LD_EN_RX_LO(0x59) |
                XCVR_TSM_TIMING15_FREQ_TARG_LD_EN_TX_HI(0x6F) | XCVR_TSM_TIMING15_FREQ_TARG_LD_EN_TX_LO(0x71),

    /* TIMING16 configuration, dependencies: ['COM', 'COM'] */
    .timing16 = XCVR_TSM_TIMING16_RX_INIT_RX_HI(0x56) | XCVR_TSM_TIMING16_RX_INIT_RX_LO(0x57),

    /* TIMING17 configuration, dependencies: ['COM', 'COM'] */
    .timing17 = XCVR_TSM_TIMING17_RX_DIG_EN_RX_HI(0x56) | XCVR_TSM_TIMING17_RX_DIG_EN_RX_LO(0x59),

    /* TIMING18 configuration, dependencies: ['COM', 'COM'] */
    .timing18 = XCVR_TSM_TIMING18_RX_PHY_EN_RX_HI(0x56) | XCVR_TSM_TIMING18_RX_PHY_EN_RX_LO(0x59),

    /* TIMING19 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing19 = XCVR_TSM_TIMING19_SEQ_BG_PUP_IBG_CAL_RX_HI(0x00) | XCVR_TSM_TIMING19_SEQ_BG_PUP_IBG_CAL_RX_LO(0x11) |
                XCVR_TSM_TIMING19_SEQ_BG_PUP_IBG_CAL_TX_HI(0x00) | XCVR_TSM_TIMING19_SEQ_BG_PUP_IBG_CAL_TX_LO(0x11),

    /* TIMING20 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing20 = XCVR_TSM_TIMING20_SEQ_LDOTRIM_PUP_RX_HI(0x00) | XCVR_TSM_TIMING20_SEQ_LDOTRIM_PUP_RX_LO(0x11) |
                XCVR_TSM_TIMING20_SEQ_LDOTRIM_PUP_TX_HI(0x00) | XCVR_TSM_TIMING20_SEQ_LDOTRIM_PUP_TX_LO(0x11),

    /* TIMING21 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing21 = XCVR_TSM_TIMING21_SEQ_LDO_CAL_PUP_RX_HI(0x00) | XCVR_TSM_TIMING21_SEQ_LDO_CAL_PUP_RX_LO(0x19) |
                XCVR_TSM_TIMING21_SEQ_LDO_CAL_PUP_TX_HI(0x00) | XCVR_TSM_TIMING21_SEQ_LDO_CAL_PUP_TX_LO(0x11),

    /* TIMING22 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing22 = XCVR_TSM_TIMING22_SEQ_BG_FC_RX_HI(0x00) | XCVR_TSM_TIMING22_SEQ_BG_FC_RX_LO(0x56) |
                XCVR_TSM_TIMING22_SEQ_BG_FC_TX_HI(0x00) | XCVR_TSM_TIMING22_SEQ_BG_FC_TX_LO(0x6F),

    /* TIMING23 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing23 = XCVR_TSM_TIMING23_SEQ_LDO_GANG_FC_RX_HI(0x00) | XCVR_TSM_TIMING23_SEQ_LDO_GANG_FC_RX_LO(0x56) |
                XCVR_TSM_TIMING23_SEQ_LDO_GANG_FC_TX_HI(0x00) | XCVR_TSM_TIMING23_SEQ_LDO_GANG_FC_TX_LO(0x6F),

    /* TIMING24 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing24 = XCVR_TSM_TIMING24_SEQ_LDO_GANG_PUP_RX_HI(0x00) | XCVR_TSM_TIMING24_SEQ_LDO_GANG_PUP_RX_LO(0x59) |
                XCVR_TSM_TIMING24_SEQ_LDO_GANG_PUP_TX_HI(0x00) | XCVR_TSM_TIMING24_SEQ_LDO_GANG_PUP_TX_LO(0x71),

    /* TIMING25 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing25 = XCVR_TSM_TIMING25_SEQ_LDO_LV_PUP_RX_HI(0x00) | XCVR_TSM_TIMING25_SEQ_LDO_LV_PUP_RX_LO(0x5A) |
                XCVR_TSM_TIMING25_SEQ_LDO_LV_PUP_TX_HI(0x00) | XCVR_TSM_TIMING25_SEQ_LDO_LV_PUP_TX_LO(0x72),

    /* TIMING26 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing26 = XCVR_TSM_TIMING26_SEQ_BG_PUP_RX_HI(0x00) | XCVR_TSM_TIMING26_SEQ_BG_PUP_RX_LO(0x5A) |
                XCVR_TSM_TIMING26_SEQ_BG_PUP_TX_HI(0x00) | XCVR_TSM_TIMING26_SEQ_BG_PUP_TX_LO(0x72),

    /* TIMING27 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing27 = XCVR_TSM_TIMING27_SEQ_BG_PUP_IBG_ANT_RX_HI(0x00) | XCVR_TSM_TIMING27_SEQ_BG_PUP_IBG_ANT_RX_LO(0x5A) |
                XCVR_TSM_TIMING27_SEQ_BG_PUP_IBG_ANT_TX_HI(0x00) | XCVR_TSM_TIMING27_SEQ_BG_PUP_IBG_ANT_TX_LO(0x72),

    /* TIMING28 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing28 =
        XCVR_TSM_TIMING28_SEQ_BG_PUP_IBG_XO_DIST_RX_HI(0x00) | XCVR_TSM_TIMING28_SEQ_BG_PUP_IBG_XO_DIST_RX_LO(0x5A) |
        XCVR_TSM_TIMING28_SEQ_BG_PUP_IBG_XO_DIST_TX_HI(0x00) | XCVR_TSM_TIMING28_SEQ_BG_PUP_IBG_XO_DIST_TX_LO(0x72),

    /* TIMING29 configuration, dependencies: ['COM', 'COM'] */
    .timing29 = XCVR_TSM_TIMING29_SEQ_BG_PUP_IBG_TX_TX_HI(0x00) | XCVR_TSM_TIMING29_SEQ_BG_PUP_IBG_TX_TX_LO(0x72),

    /* TIMING30 configuration, dependencies: ['COM', 'COM'] */
    .timing30 = XCVR_TSM_TIMING30_SEQ_BG_PUP_IBG_RX_RX_HI(0x00) | XCVR_TSM_TIMING30_SEQ_BG_PUP_IBG_RX_RX_LO(0x5A),

    /* TIMING31 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing31 = XCVR_TSM_TIMING31_SEQ_TSM_ISO_B_2P4GHZ_RX_HI(0x08) |
                XCVR_TSM_TIMING31_SEQ_TSM_ISO_B_2P4GHZ_RX_LO(0x59) |
                XCVR_TSM_TIMING31_SEQ_TSM_ISO_B_2P4GHZ_TX_HI(0x08) | XCVR_TSM_TIMING31_SEQ_TSM_ISO_B_2P4GHZ_TX_LO(0x71),

    /* TIMING32 configuration, dependencies: ['COM', 'COM'] */
    .timing32 = XCVR_TSM_TIMING32_SEQ_RCCAL_PUP_RX_HI(0x11) | XCVR_TSM_TIMING32_SEQ_RCCAL_PUP_RX_LO(0x19),

    /* TIMING33 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing33 = XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_RX_HI(0x11) | XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_RX_LO(0x23) |
                XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_TX_HI(0x11) | XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_TX_LO(0x5A),

    /* TIMING34 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing34 = XCVR_TSM_TIMING34_SEQ_PD_PUP_RX_HI(0x11) | XCVR_TSM_TIMING34_SEQ_PD_PUP_RX_LO(0x59) |
                XCVR_TSM_TIMING34_SEQ_PD_PUP_TX_HI(0x11) | XCVR_TSM_TIMING34_SEQ_PD_PUP_TX_LO(0x71),

    /* TIMING35 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing35 = XCVR_TSM_TIMING35_SEQ_VCO_PUP_RX_HI(0x11) | XCVR_TSM_TIMING35_SEQ_VCO_PUP_RX_LO(0x59) |
                XCVR_TSM_TIMING35_SEQ_VCO_PUP_TX_HI(0x11) | XCVR_TSM_TIMING35_SEQ_VCO_PUP_TX_LO(0x71),

    /* TIMING36 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing36 = XCVR_TSM_TIMING36_SEQ_XO_DIST_EN_RX_HI(0x11) | XCVR_TSM_TIMING36_SEQ_XO_DIST_EN_RX_LO(0x59) |
                XCVR_TSM_TIMING36_SEQ_XO_DIST_EN_TX_HI(0x11) | XCVR_TSM_TIMING36_SEQ_XO_DIST_EN_TX_LO(0x71),

    /* TIMING37 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing37 =
        XCVR_TSM_TIMING37_SEQ_XO_DIST_EN_CLK_REF_RX_HI(0x11) | XCVR_TSM_TIMING37_SEQ_XO_DIST_EN_CLK_REF_RX_LO(0x59) |
        XCVR_TSM_TIMING37_SEQ_XO_DIST_EN_CLK_REF_TX_HI(0x11) | XCVR_TSM_TIMING37_SEQ_XO_DIST_EN_CLK_REF_TX_LO(0x71),

    /* TIMING38 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing38 = XCVR_TSM_TIMING38_SEQ_XO_EN_CLK_2P4G_RX_HI(0x11) | XCVR_TSM_TIMING38_SEQ_XO_EN_CLK_2P4G_RX_LO(0x59) |
                XCVR_TSM_TIMING38_SEQ_XO_EN_CLK_2P4G_TX_HI(0x11) | XCVR_TSM_TIMING38_SEQ_XO_EN_CLK_2P4G_TX_LO(0x71),

    /* TIMING39 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing39 = XCVR_TSM_TIMING39_SEQ_XO_DIST_EN_CLK_ADCDAC_RX_HI(0x2B) |
                XCVR_TSM_TIMING39_SEQ_XO_DIST_EN_CLK_ADCDAC_RX_LO(0x59) |
                XCVR_TSM_TIMING39_SEQ_XO_DIST_EN_CLK_ADCDAC_TX_HI(0x11) |
                XCVR_TSM_TIMING39_SEQ_XO_DIST_EN_CLK_ADCDAC_TX_LO(0x71),

    /* TIMING40 configuration, dependencies: ['COM', 'COM'] */
    .timing40 = XCVR_TSM_TIMING40_SEQ_DAC_PUP_TX_HI(0x11) | XCVR_TSM_TIMING40_SEQ_DAC_PUP_TX_LO(0x71),

    /* TIMING41 configuration, dependencies: ['COM', 'COM'] */
    .timing41 = XCVR_TSM_TIMING41_SEQ_VCO_EN_HPM_TX_HI(0x11) | XCVR_TSM_TIMING41_SEQ_VCO_EN_HPM_TX_LO(0x71),

    /* TIMING42 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing42 = XCVR_TSM_TIMING42_SEQ_LO_PUP_VLO_FBK_RX_HI(0x12) | XCVR_TSM_TIMING42_SEQ_LO_PUP_VLO_FBK_RX_LO(0x59) |
                XCVR_TSM_TIMING42_SEQ_LO_PUP_VLO_FBK_TX_HI(0x12) | XCVR_TSM_TIMING42_SEQ_LO_PUP_VLO_FBK_TX_LO(0x71),

    /* TIMING43 configuration, dependencies: ['COM', 'COM'] */
    .timing43 = XCVR_TSM_TIMING43_SEQ_LO_PUP_VLO_RX_RX_HI(0x12) | XCVR_TSM_TIMING43_SEQ_LO_PUP_VLO_RX_RX_LO(0x59),

    /* TIMING44 configuration, dependencies: ['COM', 'COM'] */
    .timing44 = XCVR_TSM_TIMING44_SEQ_LO_PUP_VLO_RXDRV_RX_HI(0x12) | XCVR_TSM_TIMING44_SEQ_LO_PUP_VLO_RXDRV_RX_LO(0x59),

    /* TIMING45 configuration, dependencies: ['COM', 'COM'] */
    .timing45 = XCVR_TSM_TIMING45_SEQ_LO_PUP_VLO_TX_TX_HI(0x12) | XCVR_TSM_TIMING45_SEQ_LO_PUP_VLO_TX_TX_LO(0x71),

    /* TIMING46 configuration, dependencies: ['COM', 'COM'] */
    .timing46 = XCVR_TSM_TIMING46_SEQ_LO_PUP_VLO_TXDRV_TX_HI(0x6F) | XCVR_TSM_TIMING46_SEQ_LO_PUP_VLO_TXDRV_TX_LO(0x71),

    /* TIMING47 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing47 = XCVR_TSM_TIMING47_SEQ_DIVN_PUP_RX_HI(0x13) | XCVR_TSM_TIMING47_SEQ_DIVN_PUP_RX_LO(0x59) |
                XCVR_TSM_TIMING47_SEQ_DIVN_PUP_TX_HI(0x57) | XCVR_TSM_TIMING47_SEQ_DIVN_PUP_TX_LO(0x71),

    /* TIMING48 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing48 = XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI(0x23) | XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_LO(0x59) |
                XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_TX_HI(0x5A) | XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_TX_LO(0x71),

    /* TIMING49 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing49 = XCVR_TSM_TIMING49_SEQ_PD_EN_PD_DRV_RX_HI(0x23) | XCVR_TSM_TIMING49_SEQ_PD_EN_PD_DRV_RX_LO(0x59) |
                XCVR_TSM_TIMING49_SEQ_PD_EN_PD_DRV_TX_HI(0x5A) | XCVR_TSM_TIMING49_SEQ_PD_EN_PD_DRV_TX_LO(0x71),

    /* TIMING50 configuration, dependencies: ['COM', 'COM'] */
    .timing50 = XCVR_TSM_TIMING50_SEQ_CBPF_EN_DCOC_RX_HI(0x2B) | XCVR_TSM_TIMING50_SEQ_CBPF_EN_DCOC_RX_LO(0x59),

    /* TIMING51 configuration, dependencies: ['COM', 'COM'] */
    .timing51 = XCVR_TSM_TIMING51_SEQ_RX_GANG_PUP_RX_HI(0x2B) | XCVR_TSM_TIMING51_SEQ_RX_GANG_PUP_RX_LO(0x59),

    /* TIMING52 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .timing52 = XCVR_TSM_TIMING52_SEQ_SPARE3_RX_HI(0xFF) | XCVR_TSM_TIMING52_SEQ_SPARE3_RX_LO(0xFF) |
                XCVR_TSM_TIMING52_SEQ_SPARE3_TX_HI(0xFF) | XCVR_TSM_TIMING52_SEQ_SPARE3_TX_LO(0xFF),

/* WU_LATENCY configuration, dependencies: ['COM', 'COM'] */
#if RF_OSC_26MHZ == 1
    .wu_latency = XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY(0x06) |
#else
    .wu_latency = XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY(0x06) |
#endif /* RF_OSC_26MHZ == 1 */
                  XCVR_TSM_WU_LATENCY_TX_DATAPATH_LATENCY(0x06),

    /* XCVR_TX_DIG configs */
    /***********************/

    /* DATA_PADDING_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .data_padding_ctrl = XCVR_TX_DIG_DATA_PADDING_CTRL_CTE_DATA(1) | XCVR_TX_DIG_DATA_PADDING_CTRL_DATA_PADDING_SEL(0) |
                         XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY(0x01) | XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN(1) |
                         XCVR_TX_DIG_DATA_PADDING_CTRL_RAMP_DN_PAD_EN(0) |
                         XCVR_TX_DIG_DATA_PADDING_CTRL_TX_CAPTURE_POL(0),

    /* DATA_PADDING_CTRL_1 configuration, dependencies: ['COM', 'COM', 'COM'] */
    .data_padding_ctrl_1 = XCVR_TX_DIG_DATA_PADDING_CTRL_1_PA_PUP_ADJ(0x1) |
                           XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY(0x04) |
                           XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(0x04),

    /* DATA_PADDING_CTRL_2 configuration, dependencies: ['COM', 'COM'] */
    .data_padding_ctrl_2 =
        XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_MFDEV(0x0000) | XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_PFDEV(0x0000),

    /* GFSK_CTRL configuration, dependencies: ['COM', 'MD+DR', 'MD+DR'] */
    .gfsk_ctrl = XCVR_TX_DIG_GFSK_CTRL_BT_EQ_OR_GTR_ONE(0),

    /* IMAGE_FILTER_CTRL configuration, dependencies: ['COM', 'MD+DR', 'COM', 'COM', 'COM', 'COM'] */
    .image_filter_ctrl = XCVR_TX_DIG_IMAGE_FILTER_CTRL_FREQ_WORD_ADJ(0x0000) |
                         XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FIR_FILTER_OVRD(0) |
                         XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_FIR_FILTER_SEL(0) |
                         XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_SYNC0_FILTER_OVRD(0) |
                         XCVR_TX_DIG_IMAGE_FILTER_CTRL_IMAGE_SYNC1_FILTER_OVRD(0),

    /* PA_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM', 'COM', 'COM'] */
    .pa_ctrl = XCVR_TX_DIG_PA_CTRL_EARLY_WU_COMPLETE(0) | XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL(1) |
               XCVR_TX_DIG_PA_CTRL_PA_TGT_POWER(0x26) | XCVR_TX_DIG_PA_CTRL_TGT_PWR_SRC(0) |
               XCVR_TX_DIG_PA_CTRL_TX_PA_PUP_OVRD(0) | XCVR_TX_DIG_PA_CTRL_TX_PA_PUP_OVRD_EN(0),

    /* PA_RAMP_TBL0 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .pa_ramp_tbl0 = XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP0(0x01) | XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP1(0x02) |
                    XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP2(0x04) | XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP3(0x06),

    /* PA_RAMP_TBL1 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .pa_ramp_tbl1 = XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP4(0x09) | XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP5(0x0C) |
                    XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP6(0x10) | XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP7(0x14),

    /* PA_RAMP_TBL2 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .pa_ramp_tbl2 = XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP10(0x21) | XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP11(0x26) |
                    XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP8(0x18) | XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP9(0x1C),

    /* PA_RAMP_TBL3 configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .pa_ramp_tbl3 = XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP12(0x2C) | XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP13(0x32) |
                    XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP14(0x38) | XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP15(0x3C),

    /* SWITCH_TX_CTRL configuration, dependencies: ['COM', 'COM', 'COM', 'COM'] */
    .switch_tx_ctrl = XCVR_TX_DIG_SWITCH_TX_CTRL_SWITCH_FIR_SEL(0) | XCVR_TX_DIG_SWITCH_TX_CTRL_SWITCH_GFSK_COEFF(0) |
                      XCVR_TX_DIG_SWITCH_TX_CTRL_SWITCH_MOD(0) | XCVR_TX_DIG_SWITCH_TX_CTRL_SWITCH_TGT_PWR(0x00),

    /* TXDIG_CTRL configuration, dependencies: ['COM', 'COM', 'MD+DR', 'MD+DR'] */
    .txdig_ctrl = XCVR_TX_DIG_TXDIG_CTRL_DATA_STREAM_SEL(0) | XCVR_TX_DIG_TXDIG_CTRL_INV_DATA_OUT(0),
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /************** xcvr_common_config *************/
    /***********************************************/
};
