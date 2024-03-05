/*
 * Copyright 2018-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */

#ifndef NXP_XCVR_MODE_CONFIG_H
/* clang-format off */
#define NXP_XCVR_MODE_CONFIG_H
/* clang-format on */

#include "fsl_device_registers.h"

/*!
 * @addtogroup configs Radio Configuration Files
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief XCVR common configure structure
 */
typedef struct tag_xcvr_common_config_t
{
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /************ tag_xcvr_common_config ***********/
    /***********************************************/
    /* GEN4PHY registers */
    uint32_t dmd_ctrl0;
    uint32_t dmd_ctrl1;
    uint32_t fsk_cfg0;
    uint32_t fsk_cfg2;
    uint32_t fsk_fad_cfg;
    uint32_t fsk_fad_ctrl;
    uint32_t fsk_pt;
    uint32_t misc;
    uint32_t prephy_misc;
    uint32_t rtt_ctrl;
    uint32_t sm_cfg;
    /* XCVR_ANALOG registers */
    uint32_t ldo_0;
    uint32_t ldo_1;
    uint32_t pll;
    uint32_t rx_0;
    uint32_t rx_1;
    uint32_t tx_dac_pa;
    uint32_t xo_dist;
    /* XCVR_MISC registers */
    uint32_t ldo_trim_0;
    uint32_t ldo_trim_1;
    uint32_t xcvr_ctrl;
    /* XCVR_PLL_DIG registers */
    uint32_t chan_map;
    uint32_t chan_map_ext;
    uint32_t data_rate_ovrd_ctrl1;
    uint32_t delay_match;
    uint32_t hpmcal_ctrl;
    uint32_t hpm_bump;
    uint32_t hpm_cal_timing;
    uint32_t hpm_ctrl;
    uint32_t hpm_sdm_res;
    uint32_t lock_detect;
    uint32_t lpm_ctrl;
    uint32_t lpm_sdm_ctrl1;
    uint32_t mod_ctrl;
    uint32_t pll_datarate_ctrl;
    uint32_t tuning_cap_rx_ctrl;
    uint32_t tuning_cap_tx_ctrl;
    /* XCVR_RX_DIG registers */
    uint32_t agc_ctrl;
    uint32_t agc_ctrl_stat;
    uint32_t agc_idx0_gain_cfg;
    uint32_t agc_idx0_gain_val;
    uint32_t agc_idx0_thr;
    uint32_t agc_idx10_gain_cfg;
    uint32_t agc_idx10_gain_val;
    uint32_t agc_idx10_thr;
    uint32_t agc_idx11_gain_cfg;
    uint32_t agc_idx11_gain_val;
    uint32_t agc_idx11_thr;
    uint32_t agc_idx1_gain_cfg;
    uint32_t agc_idx1_gain_val;
    uint32_t agc_idx1_thr;
    uint32_t agc_idx2_gain_cfg;
    uint32_t agc_idx2_gain_val;
    uint32_t agc_idx2_thr;
    uint32_t agc_idx3_gain_cfg;
    uint32_t agc_idx3_gain_val;
    uint32_t agc_idx3_thr;
    uint32_t agc_idx4_gain_cfg;
    uint32_t agc_idx4_gain_val;
    uint32_t agc_idx4_thr;
    uint32_t agc_idx5_gain_cfg;
    uint32_t agc_idx5_gain_val;
    uint32_t agc_idx5_thr;
    uint32_t agc_idx6_gain_cfg;
    uint32_t agc_idx6_gain_val;
    uint32_t agc_idx6_thr;
    uint32_t agc_idx7_gain_cfg;
    uint32_t agc_idx7_gain_val;
    uint32_t agc_idx7_thr;
    uint32_t agc_idx8_gain_cfg;
    uint32_t agc_idx8_gain_val;
    uint32_t agc_idx8_thr;
    uint32_t agc_idx9_gain_cfg;
    uint32_t agc_idx9_gain_val;
    uint32_t agc_idx9_thr;
    uint32_t agc_mis_gain_cfg;
    uint32_t agc_thr_fast;
    uint32_t agc_thr_fast_drs;
    uint32_t agc_thr_mis;
    uint32_t agc_timing0;
    uint32_t agc_timing0_drs;
    uint32_t agc_timing1;
    uint32_t agc_timing2;
    uint32_t ctrl0;
    uint32_t ctrl1;
    uint32_t dcoc_ctrl0;
    uint32_t dcoc_ctrl1;
    uint32_t nb_rssi_ctrl0;
    uint32_t nb_rssi_ctrl1;
    uint32_t rssi_global_ctrl;
    uint32_t wb_rssi_ctrl;
    /* XCVR_TSM registers */
    uint32_t ctrl;
    uint32_t end_of_seq;
    uint32_t fast_ctrl1;
    uint32_t fast_ctrl2;
    uint32_t fast_ctrl3;
    uint32_t recycle_count;
    uint32_t timing00;
    uint32_t timing01;
    uint32_t timing02;
    uint32_t timing03;
    uint32_t timing04;
    uint32_t timing05;
    uint32_t timing06;
    uint32_t timing07;
    uint32_t timing08;
    uint32_t timing09;
    uint32_t timing10;
    uint32_t timing11;
    uint32_t timing12;
    uint32_t timing13;
    uint32_t timing14;
    uint32_t timing15;
    uint32_t timing16;
    uint32_t timing17;
    uint32_t timing18;
    uint32_t timing19;
    uint32_t timing20;
    uint32_t timing21;
    uint32_t timing22;
    uint32_t timing23;
    uint32_t timing24;
    uint32_t timing25;
    uint32_t timing26;
    uint32_t timing27;
    uint32_t timing28;
    uint32_t timing29;
    uint32_t timing30;
    uint32_t timing31;
    uint32_t timing32;
    uint32_t timing33;
    uint32_t timing34;
    uint32_t timing35;
    uint32_t timing36;
    uint32_t timing37;
    uint32_t timing38;
    uint32_t timing39;
    uint32_t timing40;
    uint32_t timing41;
    uint32_t timing42;
    uint32_t timing43;
    uint32_t timing44;
    uint32_t timing45;
    uint32_t timing46;
    uint32_t timing47;
    uint32_t timing48;
    uint32_t timing49;
    uint32_t timing50;
    uint32_t timing51;
    uint32_t timing52;
    uint32_t wu_latency;
    /* XCVR_TX_DIG registers */
    uint32_t data_padding_ctrl;
    uint32_t data_padding_ctrl_1;
    uint32_t image_filter_ctrl;
    uint32_t pa_ctrl;
    uint32_t pa_ramp_tbl0;
    uint32_t pa_ramp_tbl1;
    uint32_t pa_ramp_tbl2;
    uint32_t pa_ramp_tbl3;
    uint32_t switch_tx_ctrl;
    uint32_t txdig_ctrl;
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /************ tag_xcvr_common_config ***********/
    /***********************************************/
} xcvr_common_config_t;

/*! @brief XCVR demod_wave specific configure structure (varies by radio mode) */
typedef struct tag_xcvr_demod_wave_t
{
    uint32_t dmd_wave_reg0;
    uint32_t dmd_wave_reg1;
    uint32_t dmd_wave_reg2;
} xcvr_demod_wave_t;

/*!
 * @brief XCVR modeXdatarate specific configure structure (varies by radio mode AND data rate)
 * This structure is used to store all of the XCVR settings which are dependent upon both radio mode and data rate. It
 * is used as an overlay on top of the xcvr_mode_config_t structure to supply definitions which are either not in that
 * table or which must be overridden for data rate.
 */
typedef struct tag_xcvr_mode_datarate_config_t
{
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /******** tag_xcvr_mode_datarate_config ********/
    /***********************************************/
    radio_mode_t radio_mode;
    data_rate_t data_rate;
    data_rate_t alt_data_rate;
    /* GEN4PHY registers */
    xcvr_demod_wave_t demod_wave[8];
    uint32_t fsk_cfg0;
    uint32_t fsk_cfg1;
    uint32_t fsk_pd_cfg0;
    uint32_t fsk_pd_cfg1;
    uint32_t fsk_pd_cfg2;
    uint32_t fsk_pd_ph[2];
    uint32_t fsk_pt;
    uint32_t lr_aa_cfg;
    uint32_t lr_pd_cfg;
    uint32_t lr_pd_ph[4];
    uint32_t rtt_ref;
    uint32_t sm_cfg;
    /* RADIO_CTRL registers */
    uint32_t ll_ctrl;
    /* XCVR_ANALOG registers */
    uint32_t tx_dac_pa;
    /* XCVR_MISC registers */
    uint32_t ips_fo_addr[2];
    uint32_t ips_fo_drs0_data[2];
    uint32_t ips_fo_drs1_data[2];
    uint32_t xcvr_ctrl;
    /* XCVR_PLL_DIG registers */
    uint32_t chan_map;
    uint32_t chan_map_ext;
    uint32_t data_rate_ovrd_ctrl2;
    uint32_t delay_match;
    uint32_t hpmcal_ctrl;
    uint32_t hpm_bump;
    uint32_t hpm_sdm_res;
    /* XCVR_RX_DIG registers */
    uint32_t acq_filt_0_3;
    uint32_t acq_filt_0_3_drs;
    uint32_t acq_filt_10_11;
    uint32_t acq_filt_10_11_drs;
    uint32_t acq_filt_4_7;
    uint32_t acq_filt_4_7_drs;
    uint32_t acq_filt_8_9;
    uint32_t acq_filt_8_9_drs;
    uint32_t agc_idx11_gain_cfg;
    uint32_t agc_idx11_gain_val;
    uint32_t agc_timing0;
    uint32_t agc_timing0_drs;
    uint32_t agc_timing1;
    uint32_t agc_timing1_drs;
    uint32_t agc_timing2;
    uint32_t agc_timing2_drs;
    uint32_t ctrl0;
    uint32_t ctrl0_drs;
    uint32_t dcoc_ctrl0;
    uint32_t dcoc_ctrl0_drs;
    uint32_t demod_filt_0_1;
    uint32_t demod_filt_0_1_drs;
    uint32_t demod_filt_2_4;
    uint32_t demod_filt_2_4_drs;
    uint32_t rccal_ctrl0;
    uint32_t rssi_global_ctrl;
    /* XCVR_TX_DIG registers */
    uint32_t datarate_config_filter_ctrl;
    uint32_t datarate_config_fsk_ctrl;
    uint32_t datarate_config_gfsk_ctrl;
    uint32_t data_padding_ctrl;
    uint32_t data_padding_ctrl_1;
    uint32_t data_padding_ctrl_2;
    uint32_t fsk_ctrl;
    uint32_t gfsk_coeff_0_1;
    uint32_t gfsk_coeff_2_3;
    uint32_t gfsk_coeff_4_5;
    uint32_t gfsk_coeff_6_7;
    uint32_t gfsk_ctrl;
    uint32_t image_filter_ctrl;
    uint32_t pa_ctrl;
    uint32_t txdig_ctrl;
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /******** tag_xcvr_mode_datarate_config ********/
    /***********************************************/
} xcvr_mode_datarate_config_t;

/*!
 * @brief Coding & CRCW specific configure structure (varies by data rate)
 * This structure is used to store all of the RBME settings which are dependent upon the protocol coding and CRC. It is
 * used in addition to the XCVR overall configurations in ::xcvr_config_t to fully specify the settings for radio
 * operation.
 */
typedef struct tag_xcvr_coding_config_t
{
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /************ tag_xcvr_coding_config ***********/
    /***********************************************/
    coding_t coding_mode;
    /* GEN4PHY registers */
    uint32_t dmd_ctrl1;
    /* RBME registers */
    uint32_t crcw_cfg;
    uint32_t crcw_cfg2;
    uint32_t crcw_cfg3;
    uint32_t crc_init;
    uint32_t crc_phr_sz;
    uint32_t crc_poly;
    uint32_t crc_xor_out;
    uint32_t fcp_cfg;
    uint32_t fec_bsz_ov_b4sp;
    uint32_t fec_cfg1;
    uint32_t fec_cfg2;
    uint32_t frame_over_sz;
    uint32_t npayl_over_sz;
    uint32_t pkt_sz;
    uint32_t spread_cfg;
    uint32_t whiten_cfg;
    uint32_t whiten_poly;
    uint32_t whiten_sz_thr;
    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /************ tag_xcvr_coding_config ***********/
    /***********************************************/
} xcvr_coding_config_t;

/*!
 * @brief XCVR overall configuration structure
 * This structure is used to store all of the XCVR settings pointers to simplify passing configurations to the XCVR
 * config functions. This structure and child structures specifies the transceiver settings but does not specify any
 * CRC, Whitening, or coding settings. These are handled in the
 * ::xcvr_coding_config_t structure. Both are required to configure the radio into an operational mode.
 */
typedef struct
{
    const xcvr_common_config_t *common_cfg;
    const xcvr_mode_datarate_config_t *mode_data_rate_cfg;
} xcvr_config_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Performs all register setup for XCVR initialization or mode change.
 *
 * This function sets up all of the registers in the XCVR module with user-defined settings.
 *
 * @param[in] xcvr_config  Pointer to a pointer to the combined (common and mode dependent) XCVR settings structure.
 * @return Status of the call.
 */
xcvrStatus_t XCVR_RadioGenRegSetup(const xcvr_config_t *xcvr_config);

/*!
 * @brief Initializes the RBME block.
 *
 * This function initializes the RBME module with user-defined settings to configure coding, CRC, whitening, etc.
 *
 * @param[in] rbme  Pointer to the RBME settings structure for coding, CRC, whitening setup.
 * @return Status of the call.
 */
xcvrStatus_t XCVR_RadioGenRBMESetup(const xcvr_coding_config_t *rbme);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* NXP_XCVR_MODE_CONFIG_H */
