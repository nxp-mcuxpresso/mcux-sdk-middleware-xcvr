/*
 * Copyright 2020-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_lcl_ctrl.h"
#include "nxp_xcvr_lcl_step_mgr.h"
#include "nxp_xcvr_common_config.h"

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450) && (RF_OSC_26MHZ == 0) /* RSM only supported on Gen 4.5 radios & with 32MHZ RF_OSC */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RSM_STATE_IDLE (0x0UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)   /*!< Idle state */
#define RSM_STATE_DLY (0x1UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)    /*!< Delay state. Used only for SQTE. */
#define RSM_STATE_EXT_TX (0x2UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Extend TX. Used only for PDE state. */
#define RSM_STATE_EXT_RX (0x3UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Extend RX. Used only for PDE state. */
#define RSM_STATE_WU (0x4UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)     /*!< Warmup. Used only for SQTE state. */
#define RSM_STATE_DT_TX (0x5UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)  /*!< Packet TX. Used only for SQTE state. */
#define RSM_STATE_DT_RX (0x6UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)  /*!< Packet RX. Used only for SQTE state. */
#define RSM_STATE_DT_RX_SYNC \
    (0x7UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Packet RX Sync. Used only for SQTE state. */
#define RSM_STATE_FM_TX \
    (0x8UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Frequency Measurement TX. Used only for SQTE state. */
#define RSM_STATE_FM_RX \
    (0x9UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Frequency Measurement RX. Used only for SQTE state. */
#define RSM_STATE_PH_TX (0xAUL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Phase Measurement TX state. */
#define RSM_STATE_PH_RX (0xBUL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Phase Measurement RX state. */
#define RSM_STATE_IP1_RX2TX \
    (0xCUL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Interlude Period 1 RX2TX). Used only for SQTE state. */
#define RSM_STATE_IP1_TX2RX \
    (0xDUL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Interlude Period 1 TX2RX. Used only for SQTE state. */
#define RSM_STATE_S_RX2RX \
    (0xEUL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Short Period RX2RX. Used only for SQTE state. */
#define RSM_STATE_S_TX2TX \
    (0xFUL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Short Period TX2TX. Used only for SQTE state. */
#define RSM_STATE_IP2_RX2TX (0x10UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Interlude Period 2 RX2TX state. */
#define RSM_STATE_IP2_TX2RX (0x11UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT) /*!< Interlude Period 2 TX2RX state. */
#define RSM_STATE_FC_RX2TX (0x12UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)  /*!< Frequency Change RX2TX state. */
#define RSM_STATE_FC_TX2RX (0x13UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)  /*!< Frequency Change TX2RX state. */
#define RSM_STATE_WD (0x14UL << XCVR_MISC_RSM_CSR_RSM_STATE_SHIFT)        /*!< Warmdown state */

#define TXWU_MASK (0xFFU)
#define TXWU_SHIFT (0U)
#define TXWD_MASK (0xFF00U)
#define TXWD_SHIFT (8U)
#define RXWU_MASK (0xFF0000U)
#define RXWU_SHIFT (16U)
#define RXWD_MASK (0xFF000000U)
#define RXWD_SHIFT (24U)
#define TXWU(x) (((uint32_t)(((uint32_t)(x)) << TXWU_SHIFT)) & TXWU_MASK)
#define TXWD(x) (((uint32_t)(((uint32_t)(x)) << TXWD_SHIFT)) & TXWD_MASK)
#define RXWU(x) (((uint32_t)(((uint32_t)(x)) << RXWU_SHIFT)) & RXWU_MASK)
#define RXWD(x) (((uint32_t)(((uint32_t)(x)) << RXWD_SHIFT)) & RXWD_MASK)

#define RX_SETTLE_LAT_2MBPS_SQTE (3U)
#define RX_SETTLE_LAT_1MBPS_SQTE (5U)
#define RX_SETTLE_LAT_PDE (5U)

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  /*KW47 supports longer T_PM values */
#define T_PM0_MAX (655U) /*!< Max value for T_PM0 for HADM usage.  */
#else
#define T_PM0_MAX (640U) /*!< Max value for T_PM0 for HADM usage.  */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

#define IQ_SAMPLES_PER_USEC_1MBPS                                                                                     \
    4U /*!< Number of IQ samples per usec in 1Mbps data rate, used for checking averaging window for integer multiple \
          of samples length */
#define IQ_SAMPLES_PER_USEC_2MBPS                                                                                     \
    8U /*!< Number of IQ samples per usec in 2Mbps data rate, used for checking averaging window for integer multiple \
          of samples length */
#define F_2442_CUBED 14562534888L; /* 2442^3 Used in HPM_CAL interpolation curve */
#if defined(KW45_A0_SUPPORT) && (KW45_A0_SUPPORT > 0)
#error "A0 revision of KW45 is no longer supported."
#endif

#define NADM_WA_IF_1_7MHZ (0)
#define NADM_WA_OVERDROOP (1)
/*******************************************************************************
 * Variables
 ******************************************************************************/
static struct
{
    uint32_t tsm_ovrd0;
    uint32_t tsm_ovrd1;
    uint32_t tsm_ovrd2;
    uint32_t tsm_ovrd3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
    uint32_t tsm_ovrd4;
#endif
    uint32_t tx_dig_data_padding_ctrl;
    uint32_t tx_dig_data_padding_ctrl_1;
    uint32_t tx_dig_pa_ctrl;
} xcvr_settings;

xcvr_lcl_hpm_cal_interp_t hpm_cal_2442_data;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
/* TQI register settings for 1Mbps and 2Mbps rates */
xcvr_lcl_tqi_setting_tbl_t tqi_1mbps_settings = 
{
    .t_slot_10usec_tqi = {.iq_depth=0U, .mag_depth=3U, .t1=0x20U, .t2=0x20U},
    .t_slot_20usec_tqi = {.iq_depth=1U, .mag_depth=3U, .t1=0x20U, .t2=0x20U},
    .t_slot_40usec_tqi = {.iq_depth=2U, .mag_depth=3U, .t1=0x20U, .t2=0x20U},
};

xcvr_lcl_tqi_setting_tbl_t tqi_2mbps_settings = 
{
    .t_slot_10usec_tqi = {.iq_depth=1U, .mag_depth=3U, .t1=0x20U, .t2=0x20U},
    .t_slot_20usec_tqi = {.iq_depth=2U, .mag_depth=3U, .t1=0x20U, .t2=0x20U},
    .t_slot_40usec_tqi = {.iq_depth=3U, .mag_depth=3U, .t1=0x20U, .t2=0x20U},
};
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)
void RSM_INT_IRQHandler(void);
#endif /* defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)) */

#if !defined(GCOV_DO_COVERAGE) /* local except when testing code coverage */
/*!
 * @brief Function to check the length of RSM sequences.
 *
 * This function provides a common length check for multiple functions which program RSM buffers in Packet RAM.
 *
 * @param length the length of the array of info.
 * @param maxlen the maximum length allowed.
 *
 * @return The result of the length check.
 *
 */
static inline xcvrLclStatus_t XCVR_LCL_RsmCheckSeqLen(uint16_t length, uint16_t maxlen);

/*!
 * @brief Function to validate the chosen dma duration against the active rate and averaging window selection.
 *
 * This function checks that the dma duration is an integer multiple of the averaging window to ensure a consistent
 * sample capture.
 *
 * @param dma_duration The DMA mask duration in usec.
 * @param rate The operating data rate.
 * @param avg_win The selected averaging window length.
 *
 * @return True if the selected dma duration is a valid choice for the rate and averaging window length.
 *
 */
static bool XCVR_LCL_RsmCheckDmaDuration(uint16_t dma_duration,
                                         XCVR_RSM_SQTE_RATE_T rate,
                                         XCVR_RSM_AVG_WIN_LEN_T avg_win);

/*!
 * @brief Function to validate the chosen dma mask within the selected rsm_settings.
 *
 * This function checks that the dma duration is an integer multiple of the averaging window to ensure a consistent
 * sample capture.
 *
 * @param rsm_settings_ptr The RSM configuration settings.
 * @param status_in The status of any checks prior to this routine, so the status can be cumlative.
 *
 * @return True if the configured dma mask is a valid choice for the rate and averaging window length.
 *
 */
static xcvrLclStatus_t XCVR_LCL_RsmCheckDmaMask(const xcvr_lcl_rsm_config_t *rsm_settings_ptr,
                                                xcvrLclStatus_t status_in);

/*!
 * @brief Function to calculate the an ADC offset for use in DCOC_CTRL2  for a manual DCOC calibration prior to RSM operations.
 *
 * This function calculates a total ADC offset value using a combination of the base ADC offset and also the digital residual correction.
 * It is used to support the XCVR_LCL_SetupManualDcoc() routine which calculates this offset for both I and Q channels.
 *
 * @param adc_offset_s7 - the ADC base offset value taken from the DCOC_STAT register. Signed (2's complement) 7 bit value stored in a uint8_t.
 * @param dig_corr_s8 - the digital DC residual correction value taken from the DCOC_DIG_CORR_RESULT register. Signed 8 bits but stored in a uint8_t.
 *
 * @return The signed 8 bit (int8_t) value to be used in DCOC_CTRL2 register for manual DCOC.
 *
 */
static uint8_t XCVR_LCL_CalcAdcOffset(uint8_t adc_offset_s7, uint8_t dig_corr_s8);
#endif /* !defined(GCOV_DO_COVERAGE) */

static inline void WAIT_RSM_IDLE(void)
{
    /* Wait for RSM to return to IDLE before performing other RSM operations */
    while (((XCVR_MISC->RSM_CSR & XCVR_MISC_RSM_CSR_RSM_STATE_MASK) != RSM_STATE_IDLE))
    {
        volatile uint32_t temp_csr = XCVR_MISC->RSM_CSR; /* MISRA doesn't like the routine without any side effect */
        (void)temp_csr;
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/* define repeated status check to re-use throughout the RSM API */
#if !defined(GCOV_DO_COVERAGE) /* local except when testing code coverage */
static inline xcvrLclStatus_t XCVR_LCL_RsmCheckSeqLen(uint16_t length, uint16_t maxlen)
#else
xcvrLclStatus_t XCVR_LCL_RsmCheckSeqLen(uint16_t length, uint16_t maxlen)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* check for sequences > the maximum length */
    if (length > maxlen)
    {
        status = gXcvrLclStatusInvalidLength;
    }
    /* check for sequences < the minimum length */
    if (length < XCVR_RSM_MIN_SEQ_LEN)
    {
        status = gXcvrLclStatusInvalidLength;
    }
    return status;
}

#if !defined(GCOV_DO_COVERAGE) /* local except when testing code coverage */
static bool XCVR_LCL_RsmCheckDmaDuration(uint16_t dma_duration,
                                         XCVR_RSM_SQTE_RATE_T rate,
                                         XCVR_RSM_AVG_WIN_LEN_T avg_win)
#else
bool XCVR_LCL_RsmCheckDmaDuration(uint16_t dma_duration, XCVR_RSM_SQTE_RATE_T rate, XCVR_RSM_AVG_WIN_LEN_T avg_win)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    static const uint16_t avg_win_samples[8] = {0U, 4U, 8U, 16U, 32U, 64U, 128U, 256U};
    bool valid_duration                      = false;
    /* Calculate how many IQ samples in the duration, based on the data rate */
    uint16_t num_samples_in_duration;
    if (rate == XCVR_RSM_RATE_2MBPS)
    {
        num_samples_in_duration = dma_duration * IQ_SAMPLES_PER_USEC_2MBPS;
    }
    else
    {
        num_samples_in_duration = dma_duration * IQ_SAMPLES_PER_USEC_1MBPS;
    }

    /* Check that the calculated number of samples is an integer multiple of the averaging window (if it is in use) */
    if (avg_win == XCVR_RSM_AVG_WIN_DISABLED)
    {
        valid_duration = true; /* no averaging window case has no restrictions */
    }
    else
    {
        /* check that the # of samples is greater than the duration AND that there are no lower bits set */
        /* if lower bits are set then a divide would have a remainder and the number of samples would not be an integer
         * multiple */
        if ((num_samples_in_duration >= avg_win_samples[avg_win]) &&
            ((num_samples_in_duration & (uint16_t)((avg_win_samples[avg_win] - 1U))) ==
             0U)) /* -1 works because avg_win_samples is a power of two and the check above removes the zero case */
        {
            valid_duration = true;
        }
        else
        {
            valid_duration = false;
        }
    }

    return valid_duration;
}

#if !defined(GCOV_DO_COVERAGE) /* local except when testing code coverage */
static xcvrLclStatus_t XCVR_LCL_RsmCheckDmaMask(const xcvr_lcl_rsm_config_t *rsm_settings_ptr,
                                                xcvrLclStatus_t status_in)
#else
xcvrLclStatus_t XCVR_LCL_RsmCheckDmaMask(const xcvr_lcl_rsm_config_t *rsm_settings_ptr, xcvrLclStatus_t status_in)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    xcvrLclStatus_t status = status_in;
    /* No NULLPTR check since this code is only used when the pointers have already been checked. */

    /* If RSM dma mask is used, check validity of RSM_DMA_DUR (only used for SQTE) and other aspects */
    if (rsm_settings_ptr->use_rsm_dma_mask)
    {
        if (!XCVR_LCL_RsmCheckDmaDuration(rsm_settings_ptr->rsm_dma_dur_pm, rsm_settings_ptr->rate,
                                          rsm_settings_ptr->averaging_win))
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        uint8_t rx_settling_latency =
            (uint8_t)(((xcvr_lcl_tsm_generic_config.WU_LATENCY) & XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_MASK) >>
                      XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_SHIFT);
        uint16_t phase_meas_time = ((uint16_t)rsm_settings_ptr->t_pm0);
        /* Check that DMA duration for PM */
        if (rsm_settings_ptr->rsm_dma_dur_pm >
            (phase_meas_time - ((uint16_t)(rsm_settings_ptr->rsm_dma_dly_pm) - (uint16_t)rx_settling_latency)))
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        /* Check that DMA duration for FM */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
        uint16_t t_fm0 = (uint16_t)((xcvr_lcl_rsm_generic_config.RSM_CTRL1 & XCVR_MISC_RSM_CTRL1_RSM_T_FM0_MASK) >>
                                    XCVR_MISC_RSM_CTRL1_RSM_T_FM0_SHIFT);
        uint16_t freq_meas_time = ((uint16_t)t_fm0 * T_FM_INCMT + T_FM_INCMT);
#else
        uint16_t t_fm0 = (uint16_t)((xcvr_lcl_rsm_generic_config.RSM_CTRL5 & XCVR_MISC_RSM_CTRL5_RSM_T_FM_MASK) >>
                                    XCVR_MISC_RSM_CTRL5_RSM_T_FM_SHIFT);
        uint16_t freq_meas_time = ((uint16_t)t_fm0 * T_FM_INCMT);
#endif

        if (rsm_settings_ptr->rsm_dma_dur_fm_ext >
            (freq_meas_time - ((uint16_t)(rsm_settings_ptr->rsm_dma_dly_fm_ext) - (uint16_t)rx_settling_latency)))
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        /* Check that DMA PM delay cover at least RX_SETTLING_LATENCY */
        if (rsm_settings_ptr->rsm_dma_dly_pm < rx_settling_latency)
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        /* Check that DMA FM delay cover at least RX_SETTLING_LATENCY */
        if (rsm_settings_ptr->rsm_dma_dly_fm_ext < rx_settling_latency)
        {
            status = gXcvrLclStatusInvalidArgs;
        }
    }

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* Only applies to KW47 */
// TODO: add DMA duration check for KW47 
#else
    /* Independent of whether RSM dma mask is used, check validity of RSM_DMA_DUR0 (only used for both SQTE and PDE) */
    if (!XCVR_LCL_RsmCheckDmaDuration(rsm_settings_ptr->rsm_dma_dur_fm_ext, rsm_settings_ptr->rate,
                                      rsm_settings_ptr->averaging_win))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
#endif

    return status;
}

xcvrLclStatus_t XCVR_LCL_ValidateRsmSettings(const xcvr_lcl_rsm_config_t *rsm_settings_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (rsm_settings_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Track args check by using all statements that evaluate to false (0 value) if param is ok */
        /* Sum up every status check and compare to zero at the end. If not == zero then return invalid args result */
        uint32_t tmp_logic_sum = 0U; 
        /* Only support SQTE mode of RSM */
        tmp_logic_sum += (uint32_t)((rsm_settings_ptr->op_mode != XCVR_RSM_SQTE_MODE) && 
                                    (rsm_settings_ptr->op_mode != XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE));

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
        /* Sniffer mode not supported on KW45 */
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->sniffer_mode_en == true);
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)   */

        /* Verify the rate is valid */
        tmp_logic_sum += (uint32_t)(rsm_settings_ptr->rate >= XCVR_RSM_RATE_INVALID);

#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->rtt_len >= XCVR_RSM_RTT_LEN_INVALID);

#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */

        /* Check settings that apply to both modes with no special meaning */
        tmp_logic_sum +=  (uint32_t)((rsm_settings_ptr->num_steps > XCVR_RSM_OVERALL_MAX_SEQ_LEN) ||
            (rsm_settings_ptr->num_steps < XCVR_RSM_MIN_SEQ_LEN));

        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->trig_sel >= XCVR_RSM_TRIG_INVALID);

        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->use_rsm_dma_mask && (rsm_settings_ptr->num_ant_path > 1U));


        /* This max value accounts for the HADM requirement to program T_PM1 to T_PM0+10 */
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->t_pm0 > T_PM0_MAX);

        /* Check T_FC vs min and max values and also divisible by required modulo */
        tmp_logic_sum +=  (uint32_t)((rsm_settings_ptr->t_fc < T_FC_MIN) 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
                                      || (rsm_settings_ptr->t_fc > T_FC_MAX) || 
                                      ((rsm_settings_ptr->t_fc % T_FC_MODULO) != 0U)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  */
                                      );

        /* Check T_IP1 vs min and max values and also divisible by required modulo */
        tmp_logic_sum +=  (uint32_t)((rsm_settings_ptr->t_ip1 < T_IP_MIN) 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
                                     || (rsm_settings_ptr->t_ip1 > T_IP_MAX) || 
                                      ((rsm_settings_ptr->t_ip1 % T_IP_MODULO) != 0U)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  */
                                      );

        /* Check T_IP2 vs min and max values and also divisible by required modulo */
        tmp_logic_sum +=  (uint32_t)((rsm_settings_ptr->t_ip2 < T_IP_MIN) 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
                                    || (rsm_settings_ptr->t_ip2 > T_IP_MAX) || 
                                    ((rsm_settings_ptr->t_ip2 % T_IP_MODULO) != 0U)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  */
                                    );

        /* Check T_PM vs min and max values and also divisible by required modulo */
        tmp_logic_sum +=  (uint32_t)((rsm_settings_ptr->t_pm0 > T_PM_MAX) || 
                                     (rsm_settings_ptr->t_pm0 < T_PM_MIN) 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
                                    || ((rsm_settings_ptr->t_pm0 % T_PM_MODULO) != 0U)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  */
                                    );

        /* IQ out select */
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->iq_out_sel>= XCVR_RSM_IQ_OUT_ERROR);

        /* KW47 settings check */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        /* Mode 0 timeout setting limit */
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->mode0_timeout_usec >= 4096U);
        /* PCT averaging win */
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->pct_averaging_win >= XCVR_RSM_PCT_AVG_WIN_ERROR);
        /* PA ramp time */
        tmp_logic_sum +=  (uint32_t)(rsm_settings_ptr->pa_ramp_time >= XCVR_RSM_PA_RAMP_ERROR);
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */

        if (tmp_logic_sum != 0U)  /* If any of the above logic statements is true then the argument is invalid */
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        status = XCVR_LCL_RsmCheckDmaMask(rsm_settings_ptr,
                                          status); /* Updates current status variable, can only invalidate status */
    }

    return status;
}

// TODO: check these constants still apply to KW47
/* Constants used for init that may need different treatment in some situations */
#define RSM_PA_PAD_DLY_VAL                                                                                  \
    0UL /* Amount to delay PA ramping from default case; Applied to XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY - \
           CONNRF-1142 */
#define TX_DIG_RAMP_UP_DLY \
    (6UL - RSM_PA_PAD_DLY_VAL) /* Programming value for XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY */
#if (RSM_PA_PAD_DLY_VAL > 6UL)
#error "PAD_DLY_VAL cannot exceed 6."
#endif

void XCVR_LCL_RsmPLLInit(XCVR_RSM_SQTE_RATE_T rate)
{
    uint32_t temp;
    
    /* RSM PLL overrides go here */
    temp = XCVR_PLL_DIG->CHAN_MAP;
    temp &= ~(XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_MASK |
              XCVR_PLL_DIG_CHAN_MAP_CHANNEL_NUM_OVRD_MASK); /* CHANNEL_NUM_OVRD must be zero for RSM operation */
    temp |= XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD(2U); /* RSM asserts the OVRD_EN directly through hardware logic, SW need not set it */
    XCVR_PLL_DIG->CHAN_MAP = temp;
    
    /* Overrides to keep ADC on in both RX and TX to have consistent voltages on LDOs */
    XCVR_TSM->OVRD3 |= (XCVR_TSM_OVRD3_SEQ_ADC_PUP_OVRD_MASK | XCVR_TSM_OVRD3_SEQ_ADC_PUP_OVRD_EN_MASK);
    XCVR_TSM->OVRD2 |= (XCVR_TSM_OVRD2_SEQ_XO_DIST_EN_CLK_ADCDAC_OVRD_MASK |
                                        XCVR_TSM_OVRD2_SEQ_XO_DIST_EN_CLK_ADCDAC_OVRD_EN_MASK |
                                        XCVR_TSM_OVRD2_SEQ_BG_PUP_IBG_RX_OVRD_MASK | 
                                        XCVR_TSM_OVRD2_SEQ_BG_PUP_IBG_RX_OVRD_EN_MASK);

    /* Tuning cap overrides to prevent amplitude discontinuities during HADM */
    XCVR_PLL_DIG->TUNING_CAP_TX_CTRL =
        XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_0(7U) | XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_1(7U) |
        XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_2(7U) | XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_3(7U) |
        XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_4(7U) | XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_5(7U) |
        XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_6(7U) | XCVR_PLL_DIG_TUNING_CAP_TX_CTRL_TUNING_RANGE_7(7U);
    XCVR_PLL_DIG->TUNING_CAP_RX_CTRL =
        XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_0(0U) | XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_1(0U) |
        XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_2(0U) | XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_3(0U) |
        XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_4(0U) | XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_5(0U) |
        XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_6(0U) | XCVR_PLL_DIG_TUNING_CAP_RX_CTRL_TUNING_RANGE_7(0U);

    /* this is not needed, as long as RSM_HPM_CAL = 1. For the dynamic selection: RSM_HPM_CAL = 0, HPM_DYNAMIC_SEL=1, HPM_DYNAMIC_RX_PKT_TABLE=1, HPM_DYNAMIC_RX_TONE_TABLE=0  */ 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* KW47 always uses HPM CAL from PKT RAM */
        temp = XCVR_PLL_DIG->HPM_CTRL;
        temp |= (XCVR_PLL_DIG_HPM_CTRL_HPM_DYNAMIC_RX_PKT_TABLE_MASK |
                    XCVR_PLL_DIG_HPM_CTRL_HPM_DYNAMIC_SEL_MASK);
        XCVR_PLL_DIG->HPM_CTRL = temp; 
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

#if(0) // disabled since this is overriden by IPS_FO by defaul ble config to use DRS feature.
        temp = XCVR_PLL_DIG->HPM_SDM_RES;
        temp &= ~(XCVR_PLL_DIG_HPM_SDM_RES_HPM_COUNT_ADJUST_MASK);
        temp |= XCVR_PLL_DIG_HPM_SDM_RES_HPM_COUNT_ADJUST(0U);  /* This is also configured by IPS_FO.... need to disable it */
        XCVR_PLL_DIG->HPM_SDM_RES = temp;
#endif
    
        /* Settings for 1Mpbs operation */
        temp = XCVR_TX_DIG->GFSK_CTRL;
        temp &= ~(XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV_MASK);
        temp |= XCVR_TX_DIG_GFSK_CTRL_GFSK_FDEV(0x200); /* Default: 0x400 */
        XCVR_TX_DIG->GFSK_CTRL = temp;
        
        temp = XCVR_PLL_DIG->LPM_CTRL;
        temp &= ~(XCVR_PLL_DIG_LPM_CTRL_LPM_SCALE_MASK | XCVR_PLL_DIG_LPM_CTRL_HPM_CAL_SCALE_MASK);
        temp |= XCVR_PLL_DIG_LPM_CTRL_LPM_SCALE(0x9); /* Default: 0x8 */
        temp |= XCVR_PLL_DIG_LPM_CTRL_HPM_CAL_SCALE(0xAU);
        XCVR_PLL_DIG->LPM_CTRL = temp;
        
        temp = XCVR_PLL_DIG->DELAY_MATCH;
        temp &= ~(XCVR_PLL_DIG_DELAY_MATCH_LPM_SDM_DELAY_MASK | XCVR_PLL_DIG_DELAY_MATCH_HPM_INTEGER_DELAY_MASK);
        temp |=  XCVR_PLL_DIG_DELAY_MATCH_LPM_SDM_DELAY(2U) | XCVR_PLL_DIG_DELAY_MATCH_HPM_INTEGER_DELAY(5U);
        XCVR_PLL_DIG->DELAY_MATCH = temp;

        /* 1Mbps config: IF compensation improvement [CONNRF_1163_IF_COMP] */
        XCVR_PLL_DIG->HPM_BUMP = XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_TX(2U) | XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_CAL(2U) |
                                 XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_TX(2U) | XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_CAL(2U) |
                                 XCVR_PLL_DIG_HPM_BUMP_PLL_VCO_TRIM_KVM_TX(2U) | XCVR_PLL_DIG_HPM_BUMP_PLL_VCO_TRIM_KVM_CAL(2U);
    
    
        /* Settings for 2 Mbps operation */
        temp = XCVR_TX_DIG->DATARATE_CONFIG_GFSK_CTRL;
        temp &=  ~(XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV_MASK);
        temp |=  XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x400); /* Default: 0x400 */
        XCVR_TX_DIG->DATARATE_CONFIG_GFSK_CTRL = temp;    

        /* 2Mbps config:  IF compensation improvement [CONNRF_1163_IF_COMP] */
        temp = XCVR_PLL_DIG->PLL_DATARATE_CTRL;
        temp &= ~(XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_TX_DRS_MASK | XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_CAL_DRS_MASK
                | XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_TX_DRS_MASK | XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_CAL_DRS_MASK);
        temp |= ( XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_TX_DRS(6U)
                | XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_CAL_DRS(6U)
                | XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_TX_DRS(2U) 
                | XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_CAL_DRS(2U));
        XCVR_PLL_DIG->PLL_DATARATE_CTRL = temp;
        
        XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL1 = ( XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_LPM_SCALE_CFG1(0x9)  /* Default: 0x8 */
                                             | XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_CAL_SCALE_CFG1(0xAU)
                                             | XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_TX_CFG1(2U) 
                                             | XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_CAL_CFG1(2U));

        /* Define the frequency push on the VCO */
        temp = XCVR_PLL_DIG->MOD_CTRL;
        temp &= ~(XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL_MASK);
        if (rate == XCVR_RSM_RATE_1MBPS) {
            temp |= XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(0x1950U);
        }
        else {
            temp |= XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(0x15F8U); /* 0x15F8U is the best setting for -1.5 MHz@LO with HPM_CAL_SCALE_CFG1=0x9 */
        }
        XCVR_PLL_DIG->MOD_CTRL = temp;
    
        /* IF compensation improvement [CONNRF_1163_IF_COMP] */
        XCVR_PLL_DIG->LPM_SDM_CTRL1 |= XCVR_PLL_DIG_LPM_SDM_CTRL1_HPM_ARRAY_BIAS(63U);

#if defined(NADM_WORKAROUND_IF_1_7MHZ)  &&  (NADM_WORKAROUND_IF_1_7MHZ ==1)
        /* Set IF to 1.7MHz */
        XCVR_PLL_DIG->CHAN_MAP_EXT &= ~(XCVR_PLL_DIG_CHAN_MAP_EXT_NUM_OFFSET_MASK);
        XCVR_PLL_DIG->CHAN_MAP_EXT |= (XCVR_PLL_DIG_CHAN_MAP_EXT_NUM_OFFSET(1835008U));
#endif /* NADM_WORKAROUND_IF_1_7MHZ */
}

xcvrLclStatus_t XCVR_LCL_RsmPLLBackup(rsm_reg_backup_t *reg_backup_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (reg_backup_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
#if (0)        /* This code belongs in XCVR_LCL_RsmRegBackup */
        /* RSM and LCL registers backup - these registers only apply to RSM & LCL, they may be able to be retained during other radio modes */
        reg_backup_ptr->XCVR_MISC_DMA_CTRL = XCVR_MISC->DMA_CTRL;
        
        reg_backup_ptr->XCVR_MISC_LCL_CFG0 = XCVR_MISC->LCL_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_CFG1 = XCVR_MISC->LCL_CFG1;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG0 = XCVR_MISC->LCL_TX_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG1 = XCVR_MISC->LCL_TX_CFG1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_MISC_RSM_CTRL6 = XCVR_MISC->RSM_CTRL6;
#else
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG2 = XCVR_MISC->LCL_TX_CFG2;
#endif
        reg_backup_ptr->XCVR_MISC_LCL_RX_CFG0 = XCVR_MISC->LCL_RX_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_RX_CFG1 = XCVR_MISC->LCL_RX_CFG1;
        reg_backup_ptr->XCVR_MISC_LCL_RX_CFG2 = XCVR_MISC->LCL_RX_CFG2;
        reg_backup_ptr->XCVR_MISC_LCL_PM_MSB = XCVR_MISC->LCL_PM_MSB;
        reg_backup_ptr->XCVR_MISC_LCL_PM_LSB = XCVR_MISC->LCL_PM_LSB;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL0 = XCVR_MISC->LCL_GPIO_CTRL0;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL1 = XCVR_MISC->LCL_GPIO_CTRL1;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL2 = XCVR_MISC->LCL_GPIO_CTRL2;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL3 = XCVR_MISC->LCL_GPIO_CTRL3;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL4 = XCVR_MISC->LCL_GPIO_CTRL4;
        reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_DELAY = XCVR_MISC->DMA_MASK_DELAY;
        reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_PERIOD = XCVR_MISC->DMA_MASK_PERIOD;
        
        reg_backup_ptr->XCVR_MISC_RSM_CTRL0 = XCVR_MISC->RSM_CTRL0;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL1 = XCVR_MISC->RSM_CTRL1;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL2 = XCVR_MISC->RSM_CTRL2;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL3 = XCVR_MISC->RSM_CTRL3;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL4 = XCVR_MISC->RSM_CTRL4;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_MISC_RSM_CTRL5 = XCVR_MISC->RSM_CTRL5;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL7 = XCVR_MISC->RSM_CTRL7;
        
        reg_backup_ptr->XCVR_MISC_RSM_INT_ENABLE = XCVR_MISC->RSM_INT_ENABLE;
#endif
#endif /* (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))         */
        
        /* PLL registers backup only - these registers need backup if corrupting PLL registers */
        reg_backup_ptr->XCVR_PLL_DIG_CHAN_MAP    = XCVR_PLL_DIG->CHAN_MAP;
        reg_backup_ptr->XCVR_PLL_DIG_HPM_CTRL    = XCVR_PLL_DIG->HPM_CTRL;
        reg_backup_ptr->XCVR_TX_DIG_GFSK_CTRL    = XCVR_TX_DIG->GFSK_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_LPM_CTRL    = XCVR_PLL_DIG->LPM_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_LPM_SDM_CTRL1 = XCVR_PLL_DIG->LPM_SDM_CTRL1;
        reg_backup_ptr->XCVR_PLL_DIG_DELAY_MATCH = XCVR_PLL_DIG->DELAY_MATCH;
        reg_backup_ptr->XCVR_PLL_DIG_HPM_SDM_RES = XCVR_PLL_DIG->HPM_SDM_RES;
        reg_backup_ptr->XCVR_PLL_DIG_HPM_BUMP    = XCVR_PLL_DIG->HPM_BUMP;
        reg_backup_ptr->XCVR_PLL_DIG_MOD_CTRL    = XCVR_PLL_DIG->MOD_CTRL;
        reg_backup_ptr->XCVR_PLL_OFFSET_CTRL =
            XCVR_PLL_DIG->PLL_OFFSET_CTRL; /* this register is backed up because CFO compensation can corrupt it */
        reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_TX_CTRL = XCVR_PLL_DIG->TUNING_CAP_TX_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_RX_CTRL = XCVR_PLL_DIG->TUNING_CAP_RX_CTRL;
         /* TSM Overrides controlling the PLL signals, so included in PLL backup/restore */
        reg_backup_ptr->XCVR_TSM_OVRD0 = XCVR_TSM->OVRD0;
        reg_backup_ptr->XCVR_TSM_OVRD1 = XCVR_TSM->OVRD1;
        reg_backup_ptr->XCVR_TSM_OVRD2 = XCVR_TSM->OVRD2;
        reg_backup_ptr->XCVR_TSM_OVRD3 = XCVR_TSM->OVRD3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_TSM_OVRD4 = XCVR_TSM->OVRD4;
#endif


    }
    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmPLLRestore(const rsm_reg_backup_t *reg_backup_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (reg_backup_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
#if (0)        /* This code belongs in XCVR_LCL_RsmRegBackup */
        /* RSM and LCL registers restore - these registers only apply to RSM & LCL, they may be able to be retained during other radio modes */
        XCVR_MISC->DMA_CTRL = reg_backup_ptr->XCVR_MISC_DMA_CTRL; 
        
        XCVR_MISC->LCL_CFG0 = reg_backup_ptr->XCVR_MISC_LCL_CFG0;
        XCVR_MISC->LCL_CFG1 = reg_backup_ptr->XCVR_MISC_LCL_CFG1;
        XCVR_MISC->LCL_TX_CFG0 = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG0;
        XCVR_MISC->LCL_TX_CFG1 = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_MISC->RSM_CTRL6 = reg_backup_ptr->XCVR_MISC_RSM_CTRL6;
#else
        XCVR_MISC->LCL_TX_CFG2 = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG2;
#endif
        XCVR_MISC->LCL_RX_CFG0 = reg_backup_ptr->XCVR_MISC_LCL_RX_CFG0;
        XCVR_MISC->LCL_RX_CFG1 = reg_backup_ptr->XCVR_MISC_LCL_RX_CFG1;
        XCVR_MISC->LCL_RX_CFG2 = reg_backup_ptr->XCVR_MISC_LCL_RX_CFG2;
        XCVR_MISC->LCL_PM_MSB = reg_backup_ptr->XCVR_MISC_LCL_PM_MSB;
        XCVR_MISC->LCL_PM_LSB = reg_backup_ptr->XCVR_MISC_LCL_PM_LSB;
        XCVR_MISC->LCL_GPIO_CTRL0 = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL0;
        XCVR_MISC->LCL_GPIO_CTRL1 = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL1;
        XCVR_MISC->LCL_GPIO_CTRL2 = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL2;
        XCVR_MISC->LCL_GPIO_CTRL3 = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL3;
        XCVR_MISC->LCL_GPIO_CTRL4 = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL4;
        XCVR_MISC->DMA_MASK_DELAY = reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_DELAY;
        XCVR_MISC->DMA_MASK_PERIOD = reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_PERIOD;
        
        XCVR_MISC->RSM_CTRL0 = reg_backup_ptr->XCVR_MISC_RSM_CTRL0;
        XCVR_MISC->RSM_CTRL1 = reg_backup_ptr->XCVR_MISC_RSM_CTRL1;
        XCVR_MISC->RSM_CTRL2 = reg_backup_ptr->XCVR_MISC_RSM_CTRL2;
        XCVR_MISC->RSM_CTRL3 = reg_backup_ptr->XCVR_MISC_RSM_CTRL3;
        XCVR_MISC->RSM_CTRL4 = reg_backup_ptr->XCVR_MISC_RSM_CTRL4;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_MISC->RSM_CTRL5 = reg_backup_ptr->XCVR_MISC_RSM_CTRL5;
        XCVR_MISC->RSM_CTRL7 = reg_backup_ptr->XCVR_MISC_RSM_CTRL7;
        
        XCVR_MISC->RSM_INT_ENABLE = reg_backup_ptr->XCVR_MISC_RSM_INT_ENABLE;
#endif
#endif /* (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))         */

        
        /* PLL registers restore - these registers need restore if corrupting PLL registers */
        
        /* restore all changed registers from backup */
        XCVR_PLL_DIG->CHAN_MAP    = reg_backup_ptr->XCVR_PLL_DIG_CHAN_MAP;
        XCVR_PLL_DIG->HPM_CTRL    = reg_backup_ptr->XCVR_PLL_DIG_HPM_CTRL;
        XCVR_TX_DIG->GFSK_CTRL    = reg_backup_ptr->XCVR_TX_DIG_GFSK_CTRL;
        XCVR_PLL_DIG->LPM_CTRL    = reg_backup_ptr->XCVR_PLL_DIG_LPM_CTRL;
        XCVR_PLL_DIG->LPM_SDM_CTRL1 = reg_backup_ptr->XCVR_PLL_DIG_LPM_SDM_CTRL1;
        XCVR_PLL_DIG->DELAY_MATCH = reg_backup_ptr->XCVR_PLL_DIG_DELAY_MATCH;
        XCVR_PLL_DIG->HPM_SDM_RES = reg_backup_ptr->XCVR_PLL_DIG_HPM_SDM_RES;
        XCVR_PLL_DIG->HPM_BUMP    = reg_backup_ptr->XCVR_PLL_DIG_HPM_BUMP;
        XCVR_PLL_DIG->MOD_CTRL    = reg_backup_ptr->XCVR_PLL_DIG_MOD_CTRL;
        XCVR_PLL_DIG->PLL_OFFSET_CTRL =
            reg_backup_ptr
                ->XCVR_PLL_OFFSET_CTRL; /* this register is restored because CFO compensation can corrupt it */
        XCVR_PLL_DIG->TUNING_CAP_TX_CTRL = reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_TX_CTRL;
        XCVR_PLL_DIG->TUNING_CAP_RX_CTRL = reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_RX_CTRL;
        /* TSM Overrides controlling the PLL signals, so included in PLL backup/restore */
        XCVR_TSM->OVRD0 = reg_backup_ptr ->XCVR_TSM_OVRD0; 
        XCVR_TSM->OVRD1 = reg_backup_ptr ->XCVR_TSM_OVRD1; 
        XCVR_TSM->OVRD2 = reg_backup_ptr ->XCVR_TSM_OVRD2; 
        XCVR_TSM->OVRD3 = reg_backup_ptr ->XCVR_TSM_OVRD3; 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_TSM->OVRD4 = reg_backup_ptr ->XCVR_TSM_OVRD4; 
#endif
    }
    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmInit(const xcvr_lcl_rsm_config_t *rsm_settings_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (rsm_settings_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
#if  defined(XCVR_SKIP_RSM_SETTINGS_CHECK) && (XCVR_SKIP_RSM_SETTINGS_CHECK == 0)
    else
    {
        /* Verify the settings within the structure are valid */
        status = XCVR_LCL_ValidateRsmSettings(
            rsm_settings_ptr); /* Separate routine allows easier testing of this validation step */
    }
#endif /* defined(XCVR_SKIP_RSM_SETTINGS_CHECK) && (XCVR_SKIP_RSM_SETTINGS_CHECK == 0) */
    if (status == gXcvrLclStatusSuccess)
    {
        bool rate_is_2mbps =
            (rsm_settings_ptr->rate == XCVR_RSM_RATE_2MBPS); /* True == 2Mbps rate; False == 1Mbps rate */
        bool is_sqte_mode =
            ((rsm_settings_ptr->op_mode == XCVR_RSM_SQTE_MODE)  ||
            (rsm_settings_ptr->op_mode == XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE)); /* Will need to use this test frequently */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  /* Stable phase test workaround for KW45 */
        bool is_sqte_stable_phase_mode =
            (rsm_settings_ptr->op_mode == XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE); /* Flags custom settings for stable phase testing for KW45 workaround */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  */
        
        uint32_t temp;
        /* ************** */
        /* Setup LCL for ranging */
        /* ************** */
        /* Setup LCL_CTRL for antenna switching & DMA durations */
        /* Setup XCVR_DMA to capture phase and measurement samples  */
        /* Setup DSB to support the XCVR_DMA sample capture */

        /* ************** */
        /* Setup PHY RTT for ranging */
        /* ************** */
        if (is_sqte_mode) /* RTT programming only applies to SQTE mode */
        {
            XCVR_2P4GHZ_PHY->RTT_CTRL |= GEN4PHY_RTT_CTRL_EN_HIGH_ACC_RTT_MASK; /* Enable the PHY RTT function */
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
            if (rsm_settings_ptr->rtt_len == XCVR_RSM_SQTE_PN64)
            {
                XCVR_2P4GHZ_PHY->RTT_CTRL |= GEN4PHY_RTT_CTRL_RTT_SEQ_LEN_MASK; /* Select 64 bit PN sequences */
                XCVR_RX_DIG->CTRL1 &= ~(XCVR_RX_DIG_CTRL1_DIS_WB_NORM_AA_FOUND_MASK); /* Must be cleared for 64 bit PNs */
            }
            else
            {
                XCVR_RX_DIG->CTRL1 |= XCVR_RX_DIG_CTRL1_DIS_WB_NORM_AA_FOUND_MASK; /* Must be set for 32 bit PNs */
            }
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */
            /* the 1Mbps case is programmed in the normal settings configuration process */
            if (rate_is_2mbps)
            {
                /* FM REF for GFSK BT=0.5, h=0.5 modulation - Mihai */
                XCVR_2P4GHZ_PHY->RTT_REF = GEN4PHY_RTT_REF_FM_REF_010(0x3AU) | GEN4PHY_RTT_REF_FM_REF_110(0x5EU) |
                                           GEN4PHY_RTT_REF_FM_REF_111(0x80U);
                XCVR_2P4GHZ_PHY->RTT_CTRL &= ~(GEN4PHY_RTT_CTRL_HA_RTT_THRESHOLD_MASK);
                XCVR_2P4GHZ_PHY->RTT_CTRL |= GEN4PHY_RTT_CTRL_HA_RTT_THRESHOLD(0x166);
            }
        }
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* KW47 demod control */
        temp = XCVR_2P4GHZ_PHY->DMD_CTRL1;
        temp &= ~(GEN4PHY_DMD_CTRL1_TTRK_INT_RANGE_MASK);
        temp |= GEN4PHY_DMD_CTRL1_TTRK_INT_RANGE(2U);
        XCVR_2P4GHZ_PHY->DMD_CTRL1 = temp;
        temp = XCVR_2P4GHZ_PHY->DMD_CTRL2;
        temp &= ~(GEN4PHY_DMD_CTRL2_WAIT_DMD_CLKEN_ADJ_MASK);
        temp |= GEN4PHY_DMD_CTRL2_WAIT_DMD_CLKEN_ADJ(6U);
        XCVR_2P4GHZ_PHY->DMD_CTRL2 = temp;       
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */
        /* ************** */
        /* Setup PLL for ranging */
        /* ************** */
        /* NOTE: This code assumes t_ip1 and t_ip2 config structures both use identical HPM CAL enable bit settings
         * (which is the case in the initial coding */
        bool do_hpm_cal = ((xcvr_lcl_rsm_generic_config.RSM_CTRL3 & XCVR_MISC_RSM_CTRL3_RSM_HPM_CAL_MASK) ==
                           0U); /* RSM isn't providing HPM CAL values */
        XCVR_LCL_RsmPLLInit(rsm_settings_ptr->rate);
        (void)status; /* Failure status won't affect the flow of the program, will just get passed as a return */
        if (do_hpm_cal)
        {
            XCVR_PLL_DIG->HPM_CTRL |=
                XCVR_PLL_DIG_HPM_CTRL_RX_HPM_CAL_EN_MASK; /* RSM is not providing the HPM CAL values so PLL must perform
                                                             calibration. */
        }
        else
        {
            XCVR_PLL_DIG->HPM_CTRL &= ~(XCVR_PLL_DIG_HPM_CTRL_RX_HPM_CAL_EN_MASK); /* RSM forces HPM CAL values so PLL
                                                                                      does not perform calibration.  */
        }

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* KW47 Inline Phase Return */
        if (rsm_settings_ptr->enable_inpr)
        {
            /* Inline Phase Return enable - full version */
            temp = XCVR_PLL_DIG->LPM_SDM_CTRL2;
            temp &=  ~(XCVR_PLL_DIG_LPM_SDM_CTRL2_INPR_DT_MASK);
            temp |= (XCVR_PLL_DIG_LPM_SDM_CTRL2_EN_INPR_MASK |  /* Enable Inline Phase Return */
                            XCVR_PLL_DIG_LPM_SDM_CTRL2_EN_INPR_RX_NORM_MASK |
                            XCVR_PLL_DIG_LPM_SDM_CTRL2_INPR_DT(2U));
            XCVR_PLL_DIG->LPM_SDM_CTRL2 = temp;
            temp = XCVR_PLL_DIG->LPM_SDM_CTRL3;
            temp &=  ~(XCVR_PLL_DIG_LPM_SDM_CTRL3_INPR_CORR_INV_MASK);
            temp |= (XCVR_PLL_DIG_LPM_SDM_CTRL3_INPR_TX_TQI_DIS_MASK); 
            XCVR_PLL_DIG->LPM_SDM_CTRL3 = temp;
        }
        else
        {
            XCVR_PLL_DIG->LPM_SDM_CTRL2 &=  ~(XCVR_PLL_DIG_LPM_SDM_CTRL2_EN_INPR_MASK); /* Disable Inline Phase Return */
        }
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

        /* ************** */
        /* Setup TXDIG for ranging */
        /* ************** */
        /* save for later restore */
        xcvr_settings.tx_dig_data_padding_ctrl   = XCVR_TX_DIG->DATA_PADDING_CTRL;
        xcvr_settings.tx_dig_data_padding_ctrl_1 = XCVR_TX_DIG->DATA_PADDING_CTRL_1;
        xcvr_settings.tx_dig_pa_ctrl             = XCVR_TX_DIG->PA_CTRL;
        
        temp = xcvr_settings.tx_dig_pa_ctrl;
        temp &= ~(XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL_MASK);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* KW47 supports improved PA ramping */
        uint8_t pa_ramp_sel_value = rsm_settings_ptr->pa_ramp_time;
        switch (pa_ramp_sel_value)
        {
            case XCVR_RSM_PA_RAMP_0_USEC:
            case XCVR_RSM_PA_RAMP_1_USEC:
            case XCVR_RSM_PA_RAMP_2_USEC:
            case XCVR_RSM_PA_RAMP_4_USEC:
                /* Ensure PA ramp table is correct by reprogramming to default. */
#if (1)
                XCVR_TX_DIG->PA_RAMP_TBL0 = xcvr_common_config.pa_ramp_tbl0 & ~(XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP0_MASK); /* Forces first value to zero always */
#else
                XCVR_TX_DIG->PA_RAMP_TBL0 = xcvr_common_config.pa_ramp_tbl0; /* Leave first value as programmed (to 1) */
#endif
                XCVR_TX_DIG->PA_RAMP_TBL1 = xcvr_common_config.pa_ramp_tbl1;
                XCVR_TX_DIG->PA_RAMP_TBL2 = xcvr_common_config.pa_ramp_tbl2;
                XCVR_TX_DIG->PA_RAMP_TBL3 = xcvr_common_config.pa_ramp_tbl3;
                break;
            case XCVR_RSM_PA_RAMP_3_USEC:
                /* Program new PA ramp table to make 3usec ramping work. */
                XCVR_TX_DIG->PA_RAMP_TBL0 =  XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP0(0x00) | 
                                                                    XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP1(0x03) |
                                                                    XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP2(0x07) | 
                                                                    XCVR_TX_DIG_PA_RAMP_TBL0_PA_RAMP3(0x0B);
                XCVR_TX_DIG->PA_RAMP_TBL1 = XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP4(0x10) | 
                                                                    XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP5(0x15) |
                                                                    XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP6(0x1A) | 
                                                                    XCVR_TX_DIG_PA_RAMP_TBL1_PA_RAMP7(0x20);
                XCVR_TX_DIG->PA_RAMP_TBL2 = XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP8(0x26) | 
                                                                    XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP9(0x2C) |
                                                                    XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP10(0x33) | 
                                                                    XCVR_TX_DIG_PA_RAMP_TBL2_PA_RAMP11(0x3A);
                XCVR_TX_DIG->PA_RAMP_TBL3 = XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP12(0x3E) |  /* Max PA value for last 4 steps of the table */
                                                                    XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP13(0x3E) |  /* Max PA value for last 4 steps of the table */
                                                                    XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP14(0x3E) |  /* Max PA value for last 4 steps of the table */
                                                                    XCVR_TX_DIG_PA_RAMP_TBL3_PA_RAMP15(0x3E);   /* Max PA value for last 4 steps of the table */
                pa_ramp_sel_value = XCVR_RSM_PA_RAMP_4_USEC; /* TX DIG bitfield gets the 4usec ramping value since it will count 4usec */
                break;
            default:
                status |= gXcvrLclStatusInvalidArgs;
                break;
        }
        temp |= XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL(pa_ramp_sel_value);
#else
        temp &= ~(XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL_MASK);
        temp |= XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL(2U); /* MFDEV padding */
#endif
        XCVR_TX_DIG->PA_CTRL = temp;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* KW47 specific padding settings for modulation bandwidth testing */
        temp = xcvr_settings.tx_dig_data_padding_ctrl;
        temp &= ~(XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_MASK | XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN_MASK |
                  XCVR_TX_DIG_DATA_PADDING_CTRL_DATA_PADDING_SEL_MASK);
        temp |= (XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY(RSM_PA_PAD_DLY_VAL) |
                XCVR_TX_DIG_DATA_PADDING_CTRL_DATA_PADDING_SEL(2U) |  /* 2 = negative Fdev ramp; 1 = positive Fdev ramp */
                 XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN_MASK); /* Adjust PA RAMP timing using PAD_DLY */
        XCVR_TX_DIG->DATA_PADDING_CTRL = temp;

        temp = xcvr_settings.tx_dig_data_padding_ctrl_1;
        temp &= ~(XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY_MASK |
                  XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_MASK);
        temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY(TX_DIG_RAMP_UP_DLY);
        if (rate_is_2mbps)
        {
            temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(TX_DATA_FLUSH_DLY_2MBPS); /* Setup for 2Mbps rate */
        }
        else
        {
            temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(TX_DATA_FLUSH_DLY_1MBPS); /* Setup for 1Mbps rate */
        }
        XCVR_TX_DIG->DATA_PADDING_CTRL_1 = temp;
        XCVR_TX_DIG->DATA_PADDING_CTRL_2 =XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_MFDEV(0x1DCDU) |
                                          XCVR_TX_DIG_DATA_PADDING_CTRL_2_DATA_PAD_PFDEV(0x233U);
#else
        temp = xcvr_settings.tx_dig_data_padding_ctrl;
        temp &= ~(XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_MASK | XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN_MASK |
                  XCVR_TX_DIG_DATA_PADDING_CTRL_DATA_PADDING_SEL_MASK);
        temp |= (XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY(RSM_PA_PAD_DLY_VAL) |
                 XCVR_TX_DIG_DATA_PADDING_CTRL_PAD_DLY_EN_MASK); /* Adjust PA RAMP timing using PAD_DLY */
        XCVR_TX_DIG->DATA_PADDING_CTRL = temp;

        temp = xcvr_settings.tx_dig_data_padding_ctrl_1;
        temp &= ~(XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY_MASK |
                  XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_MASK);
        temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY(TX_DIG_RAMP_UP_DLY);
        if (rate_is_2mbps)
        {
            temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(TX_DATA_FLUSH_DLY_2MBPS); /* Setup for 2Mbps rate */
        }
        else
        {
            temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(TX_DATA_FLUSH_DLY_1MBPS); /* Setup for 1Mbps rate */
        }
        XCVR_TX_DIG->DATA_PADDING_CTRL_1 = temp;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

        /* ************** */
        /* Setup RXDIG for ranging */
        /* ************** */
        /* If RCCAL is not performed, setup the CBPF override value */
        temp = XCVR_RX_DIG->RCCAL_CTRL1;
        if (!xcvr_lcl_rsm_generic_config.do_rxdig_rccal)
        {
            temp |= XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD_EN_MASK; /* Enable override of CBPF value */
            temp &= ~(XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD_MASK);
            if (rate_is_2mbps)
            {
                temp |= XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD(0x2BU); /* Setup for 2Mbps rate */
            }
            else
            {
                temp |= XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD(0x4DU); /* Setup for 1Mbps rate */
            }
            
        }
        else
        {
            temp &= ~(XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD_EN_MASK); /* Disable override of CBPF value */
        }
        XCVR_RX_DIG->RCCAL_CTRL1 = temp;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* NADM only available on KW47 */
        /* NADM feature setup */
        XCVR_RX_DIG->NADM_CTRL = (XCVR_RX_DIG_NADM_CTRL_NADM_EN_MASK |
        XCVR_RX_DIG_NADM_CTRL_NADM_SRC(1U) |
        XCVR_RX_DIG_NADM_CTRL_NADM_DLY(4U) |
        XCVR_RX_DIG_NADM_CTRL_NADM_OFFSET(8U) |
        XCVR_RX_DIG_NADM_CTRL_NADM_DMD_LATENCY(9U) |
        XCVR_RX_DIG_NADM_CTRL_NADM_FIR_LATENCY(15U));

#if defined(NADM_WA_OVERDROOP)  &&  (NADM_WA_OVERDROOP ==1)
        if (rate_is_2mbps) /* Modify IF to avoid including DC in NADM processing */
        {        /* 2Mbps Overdroop acquistion filter */
            temp = XCVR_RX_DIG->RCCAL_CTRL0;
            temp &= ~(XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS_MASK | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS_MASK);
            temp |= (XCVR_RX_DIG_RCCAL_CTRL0_CBPF_SC_CODE_DRS(0U) | XCVR_RX_DIG_RCCAL_CTRL0_CBPF_BW_CODE_DRS(0U));
            XCVR_RX_DIG->RCCAL_CTRL0 = temp;
            XCVR_RX_DIG->ACQ_FILT_0_3_DRS = XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H0(0x09) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H1(0x02) |
                        XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H2(0x7A) | XCVR_RX_DIG_ACQ_FILT_0_3_DRS_H3(0x77);
            XCVR_RX_DIG->ACQ_FILT_4_7_DRS = XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H4(0x02) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H5(0x11) |
                        XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H6(0x0D) | XCVR_RX_DIG_ACQ_FILT_4_7_DRS_H7(0xEF);
            XCVR_RX_DIG->ACQ_FILT_8_9_DRS = XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H8(0x1D8) | XCVR_RX_DIG_ACQ_FILT_8_9_DRS_H9(0x1F0);
            XCVR_RX_DIG->ACQ_FILT_10_11_DRS = XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H10(0x039) | XCVR_RX_DIG_ACQ_FILT_10_11_DRS_H11(0x07B);
        }
#endif

#if defined(NADM_WA_IF_1_7MHZ)  &&  (NADM_WA_IF_1_7MHZ ==1)
        if (rate_is_2mbps) /* Modify IF to avoid including DC in NADM processing */
        {
            /* Set IF to 1.7MHz */
            XCVR_RX_DIG->CTRL0 &= ~(XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ_MASK);
            XCVR_RX_DIG->CTRL0 |= (XCVR_RX_DIG_CTRL0_DIG_MIXER_FREQ(56U));
        }
#endif /* NADM_WORKAROUND_IF_1_7MHZ */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */

#if (0) // DCOC is not run with generic tsm sequence. if enabled, this should be moved after saving the ovrd regs for
        // clean restore
        /* [CONNRF-1139] FIX Tx_gain not applied when TX WU runs DCOC */
        if (rsm_settings_ptr->t_fc_tsm_config->T_FC_VALUE >= 80U)
        {
            XCVR_RX_DIG->DFT_CTRL |= 1UL << (9U + XCVR_RX_DIG_DFT_CTRL_CGM_OVRD_SHIFT);
        }
#endif

        /* Setup RSSI and RXDIG output to DMA */
        temp = XCVR_RX_DIG->DFT_CTRL;
        temp &= ~(
            XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_OUT_SEL_MASK | 
            XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_MAG_OUT_SEL_MASK |
                  XCVR_RX_DIG_DFT_CTRL_DFT_RX_PH_OUT_SEL_MASK |
                  XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL_MASK);
        temp |= 
                XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_OUT_SEL(4U) |
                XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_MAG_OUT_SEL(7U) |
                XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL(rsm_settings_ptr->iq_out_sel);
        XCVR_RX_DIG->DFT_CTRL = temp;
        temp                  = XCVR_RX_DIG->CTRL1;
        temp |= XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK;
        temp &= ~(XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_MASK);
        /* Setup averager (in the same register) */
        temp |= XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN(
            rsm_settings_ptr->averaging_win); /* Setup according to input structure */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* Only applies to KW47 */
        temp |= XCVR_RX_DIG_CTRL1_RX_DFT_IQ_OUT_AVERAGED_MASK |
            XCVR_RX_DIG_CTRL1_RX_IQ_AVG_WIN_PCT(
            rsm_settings_ptr->pct_averaging_win); /* Setup according to input structure for second stage of averaging */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */
        XCVR_RX_DIG->CTRL1 = temp;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        temp = XCVR_RX_DIG->AGC_CTRL;
        temp &= ~(XCVR_RX_DIG_AGC_CTRL_AGC_HOLD_EN_MASK);  // to avoid quick changes for stable localisation 
        temp |=XCVR_RX_DIG_AGC_CTRL_AGC_HOLD_EN_MASK ;    // to avoid AGC computation at the end of a RX packet
        XCVR_RX_DIG->AGC_CTRL = temp;
#endif
        /* Setup DMA start trigger using DMA configuration APIs */

        /* TQI registers configured in XCVR_LCL_ConfigLclBlock() */

        /* ************** */
        /* Setup TSM for ranging */
        /* ************** */
        status |= XCVR_LCL_ReprogramTsmTimings(&xcvr_lcl_tsm_generic_config); /* TSM settings are stored in const config structures */

        temp = XCVR_RX_DIG->DFT_CTRL;
#if (1) /* TSM OVERRIDES for preserving PLL constant phase */
        xcvr_settings.tsm_ovrd0 = XCVR_TSM->OVRD0;
        xcvr_settings.tsm_ovrd1 = XCVR_TSM->OVRD1;
        xcvr_settings.tsm_ovrd2 = XCVR_TSM->OVRD2;
        xcvr_settings.tsm_ovrd3 = XCVR_TSM->OVRD3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        xcvr_settings.tsm_ovrd4 = XCVR_TSM->OVRD4;
#endif

        temp |= XCVR_RX_DIG_DFT_CTRL_CGM_OVRD(4); // to maintain rx_dig_mixer_clk
#endif
        /* [CONNRF-1139] FIX Tx_gain not applied when TX WU runs DCOC */
        temp |= 1UL << (9U + XCVR_RX_DIG_DFT_CTRL_CGM_OVRD_SHIFT);
        XCVR_RX_DIG->DFT_CTRL = temp;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        /* Configure RX_SETTLING_LATENCY */
        temp = XCVR_TSM->WU_LATENCY;
        temp &= ~(XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_MASK);
        if (rate_is_2mbps)
        {
            temp |= XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY(RX_SETTLING_LATENCY_2MBPS);
        }
        else
        {
            temp |= XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY(RX_SETTLING_LATENCY_1MBPS);
        }
        XCVR_TSM->WU_LATENCY = temp;
#endif

        /* ************** */
        /* Setup RSM for ranging */
        /* ************** */
                /* Customize RSM and TSM settings for 655usec long tone using 2 step sequence */
        uint8_t temp_t_fcs_usec; 
        uint8_t temp_t_fm_reg_setting; 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  /* Stable phase test workaround for KW45 */
        if (is_sqte_stable_phase_mode)
        {
            /* customize T_FC and T_FM settings for stable phase testing */
            if (rsm_settings_ptr->t_fc < 130U)
            {
                /* Shorter sequence configuration */
                temp_t_fcs_usec =rsm_settings_ptr->t_fc+15U;  /* T_FCS 15usec longer */
                temp_t_fm_reg_setting = 7U; /* Keep default T_FM */
            }
            else
            {
                /* Longer sequence configuration */
                temp_t_fcs_usec = rsm_settings_ptr->t_fc-5U;   /* T_FCS 20usec shorter then 15usec longer (ends up 5usec shorter) */
                temp_t_fm_reg_setting = 9U; /* Extend T_FM by 20usec (2 counts in the register) */
            }
        }
        else
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) */
        {
                temp_t_fcs_usec =rsm_settings_ptr->t_fc;  /* T_FCS unchanged */
                temp_t_fm_reg_setting = 7U; /* Keep default T_FM */
        }

        /* RSM_CTRL0 */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL0; /* Structure based configuration for the RSM_CTRL0 register;  */
        temp &= ~(XCVR_MISC_RSM_CTRL0_RSM_MODE_MASK | /* Zero out all fields being modified by logic */
                  XCVR_MISC_RSM_CTRL0_RSM_RATE_MASK | XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL_MASK |
                  XCVR_MISC_RSM_CTRL0_RSM_TRIG_DLY_MASK | XCVR_MISC_RSM_CTRL0_RSM_STEPS_MASK);
        /* Configure mode */
        if (!is_sqte_mode)
        {
            temp |= XCVR_MISC_RSM_CTRL0_RSM_MODE_MASK; /* PDE mode */
        }
        /* Configure rate */
        if (rate_is_2mbps)
        {
            temp |= XCVR_MISC_RSM_CTRL0_RSM_RATE_MASK; /* Setup for 2Mbps rate */
        }
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        if (rsm_settings_ptr->sniffer_mode_en)
        {
            temp &= ~(XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK); /* Turn off TX in sniffer mode */
            temp |= XCVR_MISC_RSM_CTRL0_RSM_SN_EN_MASK; /* Enable sniffer mode */
        }
#endif
        /* Configure trigger select, trigger delay and number of steps */
        temp |= (XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL((uint32_t)rsm_settings_ptr->trig_sel) |
                 XCVR_MISC_RSM_CTRL0_RSM_TRIG_DLY((uint32_t)rsm_settings_ptr->trig_delay) |
                 XCVR_MISC_RSM_CTRL0_RSM_STEPS((uint32_t)rsm_settings_ptr->num_steps));
        /* For hardware triggers, RX or TX enable can be set in this routine. For SW triggered, XCVR_LCL_RsmGo() *must* be called to set the role and trigger the start. */
        if (rsm_settings_ptr->trig_sel != XCVR_RSM_TRIG_SW)
        {
            temp |= (rsm_settings_ptr->role==XCVR_RSM_TX_MODE ? XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK : XCVR_MISC_RSM_CTRL0_RSM_RX_EN_MASK);
        }
        XCVR_MISC->RSM_CTRL0 = temp;

        /* RSM_CTRL1 */
        /* Write T_FC/T_IP values - writing the entire register so no need for stored config */
        XCVR_MISC->RSM_CTRL1 = (XCVR_MISC_RSM_CTRL1_RSM_T_FC((uint8_t)((temp_t_fcs_usec + T_RD) / T_FC_INCMT)) |
                                 XCVR_MISC_RSM_CTRL1_RSM_T_IP1((uint8_t)((rsm_settings_ptr->t_ip1 + T_RD) / T_IP_INCMT)) |
                                 XCVR_MISC_RSM_CTRL1_RSM_T_IP2((uint8_t)((rsm_settings_ptr->t_ip2 + T_RD) / T_IP_INCMT)) |
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
                                XCVR_MISC_RSM_CTRL1_RSM_T_FM0(temp_t_fm_reg_setting) |
                                XCVR_MISC_RSM_CTRL1_RSM_T_FM1(temp_t_fm_reg_setting) |
                                XCVR_MISC_RSM_CTRL1_RSM_T_S(2U) 
#else
                                /* T_FM field (collapsed to 1) is moved to RSM_CTRL5 in KW47 */
                                XCVR_MISC_RSM_CTRL1_RSM_T_S(0xAU) 
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)   */
                                );

        /* RSM_CTRL2 */
        /* Configure T_PM0 and T_PM1 fields */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL2; /* Structure based configuration for the RSM_CTRL2 register; Later
                                                      logic will configure additional fields */
        temp &= ~(XCVR_MISC_RSM_CTRL2_RSM_T_PM0_MASK | XCVR_MISC_RSM_CTRL2_RSM_T_PM1_MASK);
        /*Input T_PM setting is the T_PM total duration for mult. antenna and tone extension slot */
        uint16_t t_pm0_setting = (rsm_settings_ptr->t_pm0 / T_PM_INCMT) - ZERO_BASIS;
        temp |= XCVR_MISC_RSM_CTRL2_RSM_T_PM0((uint32_t)(t_pm0_setting)) |
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
                XCVR_MISC_RSM_CTRL2_RSM_T_PM1((uint32_t)(t_pm0_setting) + 1U);
#else
                XCVR_MISC_RSM_CTRL2_RSM_RTT_TYPE(rsm_settings_ptr->rtt_type) |
                XCVR_MISC_RSM_CTRL2_RSM_T_PM1((uint32_t)(t_pm0_setting) + 10U);
#endif
        XCVR_MISC->RSM_CTRL2 = temp;

        /* RSM_CTRL3 */
        /* Configure RSM_DT_RX_SYNC_DLY & RSM_DMA_RX_EN & RSM_DMA_DUR */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL3; /* Structure based configuration for the RSM_CTRL3 register; Later
                                                      logic will configure additional fields */
        temp &= ~(XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN_MASK | XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY_MASK |
                  XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR_MASK | XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DIS_MASK);    /* Zero out all fields being modified by logic */
        if (rate_is_2mbps)
        {
          temp |= XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY(RX_SYNC_DLY_2MBPS); /* Original setting for both 1Mbps & 2Mbps rates (now testing only for 2Mbps) */
        }
        else
        {
          temp |= XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY(RX_SYNC_DLY_1MBPS); /* Decreased by 1 for 1Mbps to test shifting sync point */
        }
        if (rsm_settings_ptr->use_rsm_dma_mask)
        {
            temp |= XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN_MASK |
                    XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR(
                        rsm_settings_ptr
                            ->rsm_dma_dur_pm); /* Setup to use the RSM DMA mask and set the duration for that mask */
        }
        else
        {
            temp &= ~(XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN_MASK | XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR_MASK); /* Setup to use the LCL DMA mask control */
            XCVR_MISC->RSM_CTRL4 = 0U; /* Clear all DMA duration and delay from RSM when LCL is in use - to prevent non-interference of RSM signals on LCL */
        }
        if (rsm_settings_ptr->disable_rx_sync)
        {
            temp |= XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DIS_MASK;
        }
        XCVR_MISC->RSM_CTRL3 = temp;

        /* RSM_CTRL4 */
        if (rsm_settings_ptr->sniffer_mode_en)
        {
            temp = 0U;
        }
        else
        {
            /* Configure RSM_DMA_DUR0 & RSM_DMA_DLY0 independent of whether using dma mask (these are always used for FCS
             * sequence)*/
            temp = XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY0(rsm_settings_ptr->rsm_dma_dly_fm_ext) |
                   XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0(rsm_settings_ptr->rsm_dma_dur_fm_ext);
            /* Configure RSM_DMA_DLY  if using dma mask */
            if (rsm_settings_ptr->use_rsm_dma_mask)
            {
                temp |= XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY(rsm_settings_ptr->rsm_dma_dly_pm);
            }
        }
        XCVR_MISC->RSM_CTRL4 = temp;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        /* RSM_CTRL5 */
        XCVR_MISC->RSM_CTRL5 = xcvr_lcl_rsm_generic_config.RSM_CTRL5; /* T_FM is the only field in this register and is always 80usec  */
        (void)temp_t_fm_reg_setting; /* Touch the variable because it isn't being used in this case */

        /* RSM_CTRL6 */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL6; /* Structure based configuration for the RSM_CTRL0 register;  */
        temp &= ~(XCVR_MISC_RSM_CTRL6_RSM_RXLAT_DIG_MASK |
                        XCVR_MISC_RSM_CTRL6_RSM_MODE0_TIMEOUT_MASK);
        temp |= (XCVR_MISC_RSM_CTRL6_RSM_RXLAT_DIG(3U) |        /* give time for latency for HARTT to complete */
                        XCVR_MISC_RSM_CTRL6_RSM_EARLY_MOD_DIS_MASK |  // disable_lpm_mod for tones and pkts 
                        XCVR_MISC_RSM_CTRL6_RSM_SKIP_RECYCLE_R2R_MASK | // skip recycle at end of pkts (skip rx_init, to avoid hartt lost and also pulse can occur on AA_det instead of maintained level)
                        XCVR_MISC_RSM_CTRL6_RSM_MODE0_TIMEOUT(rsm_settings_ptr->mode0_timeout_usec));
        XCVR_MISC->RSM_CTRL6 = temp;

        /* RSM_CTRL7 */
        // TODO: add any RSM_CTRL7customization
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL7; /* Structure based configuration for the RSM_CTRL0 register;  */
        /* Configure TIME_ALIGN for reflector role only */
        if (rsm_settings_ptr->role == XCVR_RSM_RX_MODE)
        {
            temp |= XCVR_MISC_RSM_CTRL7_RSM_TIME_ALIGN_MODE(1U);
        }
        else
        {
            temp &= ~(XCVR_MISC_RSM_CTRL7_RSM_TIME_ALIGN_MODE_MASK);
        }
        XCVR_MISC->RSM_CTRL7 = temp;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)   */




        /* Prep for 2Mbps capability */
        temp = RADIO_CTRL->RF_CTRL;
        temp |= RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_EN_MASK; /* Enable overriding the radio rate selection */
        if (rate_is_2mbps)
        {
            temp |= RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_MASK;
        }
        else
        {
            temp &= ~(RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_MASK);
        }
        RADIO_CTRL->RF_CTRL = temp;
    }

    return status;
}

void XCVR_LCL_RsmDeInit(void)
{
    /* Disable mask capability from RX DIG to allow independent IQ capture  */
    XCVR_RX_DIG->CTRL1 &= ~(XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK);
#if (0)
    /* [CONNRF-1139] Disable AGC Clock override to allow gain config update in TX after DCOC */
    XCVR_RX_DIG->DFT_CTRL &= ~(1UL << (9U + XCVR_RX_DIG_DFT_CTRL_CGM_OVRD_SHIFT));
#endif

    /* Restore Tx_dig */
    XCVR_TX_DIG->DATA_PADDING_CTRL   = xcvr_settings.tx_dig_data_padding_ctrl;
    XCVR_TX_DIG->DATA_PADDING_CTRL_1 = xcvr_settings.tx_dig_data_padding_ctrl_1;
    XCVR_TX_DIG->PA_CTRL             = xcvr_settings.tx_dig_pa_ctrl;

    /* Restore TSM Overrides */
    XCVR_TSM->OVRD0 = xcvr_settings.tsm_ovrd0;
    XCVR_TSM->OVRD1 = xcvr_settings.tsm_ovrd1;
    XCVR_TSM->OVRD2 = xcvr_settings.tsm_ovrd2;
    XCVR_TSM->OVRD3 = xcvr_settings.tsm_ovrd3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)  
    XCVR_TSM->OVRD4= xcvr_settings.tsm_ovrd4;
#endif
}

xcvrLclStatus_t XCVR_LCL_Set_TSM_FastStart(XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t *rsm_settings_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (rsm_settings_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
        /* Sniffer mode not supported on KW45 */
        if (rsm_settings_ptr->sniffer_mode_en == true)
        {
            status = gXcvrLclStatusInvalidArgs;
        }
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)   */
        uint32_t t_fc = rsm_settings_ptr->t_fc;
        uint32_t t_ip2 = rsm_settings_ptr->t_ip2;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  /* Stable phase workaround for KW45 */
        uint32_t t_ip1 = rsm_settings_ptr->t_ip1;
        if ((rsm_settings_ptr->op_mode == XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE) && (rsm_settings_ptr->t_fc >= 130U))
        {
            /* Longer sequence configuration shortens T_FC for stable phase testing */
            t_fc -= 20U;
        }
    uint32_t t_ip = (t_ip1 <= t_ip2) ? t_ip1 : t_ip2;
#else
        uint32_t t_ip = t_ip2;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)   */

    uint32_t temp = xcvr_lcl_tsm_generic_config.FAST_CTRL1;
    uint32_t fast_tx2rx_start = ((temp & XCVR_TSM_FAST_CTRL1_FAST_TX2RX_START_MASK) >> XCVR_TSM_FAST_CTRL1_FAST_TX2RX_START_SHIFT);
    uint32_t fast_rx2tx_start = ((temp & XCVR_TSM_FAST_CTRL1_FAST_RX2TX_START_MASK) >> XCVR_TSM_FAST_CTRL1_FAST_RX2TX_START_SHIFT);

    uint32_t fast_ctrl2 = xcvr_lcl_tsm_generic_config.FAST_CTRL2;
    uint32_t fast_dest_rx = ((fast_ctrl2 & XCVR_TSM_FAST_CTRL2_FAST_DEST_RX_MASK) >> XCVR_TSM_FAST_CTRL2_FAST_DEST_RX_SHIFT);
    uint32_t fast_dest_tx = ((fast_ctrl2 & XCVR_TSM_FAST_CTRL2_FAST_DEST_TX_MASK) >> XCVR_TSM_FAST_CTRL2_FAST_DEST_TX_SHIFT);

    temp = xcvr_lcl_tsm_generic_config.FAST_CTRL3;
    uint32_t fast_tx2rx_start_fc = ((temp & XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC_MASK) >> XCVR_TSM_FAST_CTRL3_FAST_TX2RX_START_FC_SHIFT);
    uint32_t fast_rx2tx_start_fc = ((temp & XCVR_TSM_FAST_CTRL3_FAST_RX2TX_START_FC_MASK) >> XCVR_TSM_FAST_CTRL3_FAST_RX2TX_START_FC_SHIFT);

    temp = xcvr_lcl_tsm_generic_config.END_OF_SEQ;
    uint32_t end_of_rx_wu = ((temp & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT);
    uint32_t end_of_tx_wu = ((temp & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT);

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        temp = xcvr_lcl_tsm_generic_config.WU_LATENCY; 
        uint32_t rx_delay = ((temp & XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_MASK) >> XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_SHIFT);
#endif  /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)   */          

    fast_ctrl2 &= ~(XCVR_TSM_FAST_CTRL2_FAST_START_RX_MASK | XCVR_TSM_FAST_CTRL2_FAST_START_TX_MASK);
    uint32_t temp_fast_rx;
    uint32_t temp_fast_tx;
    switch (role)
    {
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
#define REFL_ONLY_ADJUST  (1U) /* Subtract from REFL case and sniffer case 1 */
            case XCVR_RSM_RX_MODE:
/* Reference equations 
 *   fast_start_tx_val = _tip2 - 18 - (tx_delay -3) + fast_dest_tx_val + fast_rx2tx_start_val        - end_of_tx_wu_val + _trd; 	         	         
 *   fast_start_rx_val = _tfc - rx_delay + fast_dest_rx_val + fast_tx2rx_start_fc_val      - end_of_rx_wu_val + _trd -1;  
 */
                temp_fast_tx = t_ip + T_RD - (TX_DATA_FLUSH_DLY_1MBPS-3U) + fast_dest_tx + fast_rx2tx_start - end_of_tx_wu - 18U;
                temp_fast_rx = t_fc + T_RD - rx_delay + fast_dest_rx + fast_tx2rx_start_fc - end_of_rx_wu - REFL_ONLY_ADJUST;
                if (rsm_settings_ptr->sniffer_mode_en)
                {
/* Reference equation
 *  fast_start_rx_val = _tfc -  5 - rx_delay - tx_delay + fast_dest_rx_val + fast_tx2rx_start_fc_val      - end_of_rx_wu_val + _trd -1;
 */
                    temp_fast_rx = temp_fast_rx -TX_DATA_FLUSH_DLY_1MBPS-5U; /* Sniffer case needs to move the fast start earlier to stay within T_FC time */
                }
                break;
            case XCVR_RSM_TX_MODE:
/* Reference equations 
 * fast_start_tx_val = _tfc - 18 - (tx_delay -3) + fast_dest_tx_val + fast_rx2tx_start_fc_val      - end_of_tx_wu_val + _trd; 	         
 * fast_start_rx_val = _tip2 - rx_delay + fast_dest_rx_val + fast_tx2rx_start_val        - end_of_rx_wu_val + _trd ; 
 */
                temp_fast_tx = t_fc + T_RD - (TX_DATA_FLUSH_DLY_1MBPS-3U) + fast_dest_tx + fast_rx2tx_start_fc - end_of_tx_wu - 18U;
                temp_fast_rx = t_ip + T_RD - rx_delay + fast_dest_rx + fast_tx2rx_start - end_of_rx_wu;
                break;
#else        
        /* implement CONNRF-1142 */
        case XCVR_RSM_RX_MODE:
            temp_fast_rx = t_fc + T_RD + fast_dest_rx + fast_tx2rx_start_fc - end_of_rx_wu - 4U;
            temp_fast_tx = t_ip + T_RD + fast_dest_tx + fast_rx2tx_start - end_of_tx_wu - 18U;
            break;
        case XCVR_RSM_TX_MODE:
            temp_fast_rx = t_ip + T_RD + fast_dest_rx + fast_tx2rx_start - end_of_rx_wu - 4U;
            temp_fast_tx = t_fc + T_RD + fast_dest_tx + fast_rx2tx_start_fc - end_of_tx_wu - 18U;
            break;
#endif  /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)   */          
        default:
            status = gXcvrLclStatusFail; /* Error, default case should never be encountered */
            break;
    }
    if (status == gXcvrLclStatusSuccess)
    {
        fast_ctrl2 |= XCVR_TSM_FAST_CTRL2_FAST_START_RX(temp_fast_rx) | XCVR_TSM_FAST_CTRL2_FAST_START_TX(temp_fast_tx);
        XCVR_TSM->FAST_CTRL2 = fast_ctrl2;
    }
    }
    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmGo(XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t *rsm_settings_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    (void)rsm_settings_ptr; /* Retain the rsm_settings_ptr parameter for future possible use; (void) touches it for
                               unused parameter errors */

    /* Clear any interrupt flags that may be set so that they don't cause problems on RSM startup */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
    XCVR_MISC->RSM_CSR |=
        (XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_FC_MASK |
         XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_MASK);
#else
    XCVR_MISC->RSM_INT_STATUS = 0xFFFFFFFFU; /* Write-1-to-clear all of the status bits */
#endif

    /* Set the RSM enable according to the role */
    switch (role)
    {
        case XCVR_RSM_RX_MODE:
            XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_RX_EN_MASK; /* Enable RX */
            break;
        case XCVR_RSM_TX_MODE:
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
            if (rsm_settings_ptr->sniffer_mode_en)
            {
                status = gXcvrLclStatusFail;   /* TX is not permitted in sniffer mode */
            }
            else
            {
            XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK; /* Enable TX */
            }
#else
            XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK; /* Enable TX */
#endif
            break;
        default:
            status = gXcvrLclStatusFail; /* Error, default case should never be encountered */
            break;
    }

    return status;
}

void XCVR_LCL_RsmStopAbort(bool abort_rsm)
{
    /* Only if aborting then assert abort */
    if (abort_rsm)
    {
        XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_SW_ABORT_MASK; /* Assert the abort */
    }
    /* Remainder of the sequence is common to both ABORT and STOP */
    /* Make sure that state returns to IDLE */
    WAIT_RSM_IDLE();

    /* Clear any interrupt flags that may be set so that they don't cause problems on RSM startup */
    /* This must be done before clearing RX_EN or TX_EN bits as once those are clear these bits cannot be cleared */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
    volatile uint32_t temp_csr = XCVR_MISC->RSM_CSR;
    XCVR_MISC->RSM_CSR         = temp_csr;
#else
    XCVR_MISC->RSM_INT_STATUS = 0xFFFFFFFFU; /* Write-1-to-clear all of the status bits */
#endif

    /* Then clear the abort (no effect in STOP case) */
    XCVR_MISC->RSM_CTRL0 &= ~(XCVR_MISC_RSM_CTRL0_RSM_SW_ABORT_MASK | XCVR_MISC_RSM_CTRL0_RSM_RX_EN_MASK |
                              XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK); /* Clear the RX & TX enables */
}


#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
const uint32_t tsm_fast_descrip_comp[TSM_DESCRIP_COUNT] = 
{
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 6U, &(XCVR_TSM->END_OF_SEQ)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 53U, &(XCVR_TSM->TIMING09))
};

#if (1)
const uint32_t pll_rsm_fast_descrip_comp[PLL_DESCRIP_COUNT] = 
{
#if (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))    
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_MISC->DMA_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 17U, &(XCVR_MISC->LCL_CFG0)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 7U, &(XCVR_MISC->RSM_CTRL0)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_MISC->RSM_INT_ENABLE)),
#endif /* (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))         */
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 2U, &(XCVR_2P4GHZ_PHY->RTT_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 2U, &(XCVR_2P4GHZ_PHY->DMD_CTRL1)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 2U, &(XCVR_TX_DIG->DATA_PADDING_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_TX_DIG->GFSK_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 5U, &(XCVR_TX_DIG->PA_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 3U, &(XCVR_PLL_DIG->HPM_BUMP)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_PLL_DIG->HPM_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 3U, &(XCVR_PLL_DIG->HPM_SDM_RES)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 3U, &(XCVR_PLL_DIG->DELAY_MATCH)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_PLL_DIG->PLL_OFFSET_CTRL)),
#if (0) /* This ordering of the descriptor causes a failure in the FPGA testing when polling for completion of the DMA transfer */
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 5U, &(XCVR_TSM->OVRD0)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 2U, &(XCVR_RX_DIG->CTRL1)),
#else
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 2U, &(XCVR_RX_DIG->CTRL1)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 5U, &(XCVR_TSM->OVRD0)),
#endif
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_RX_DIG->AGC_CTRL)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(XCVR_RX_DIG->RCCAL_CTRL1)),
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 1U, &(RADIO_CTRL->RF_CTRL))
#else
extern const uint32_t pll_rsm_fast_descrip_comp[1] = 
{
    PR2IPS_DESCRIPTOR(PR2IPS_BURST_XFER, 5U, &(XCVR_TSM->OVRD0)),
#endif
};

xcvrLclStatus_t XCVR_LCL_FastBackupRestore(uint32_t * desc_ptr, uint8_t num_entries, PKT_RAM_BANK_SEL_T pkt_ram_bank, uint16_t pkt_ram_index_offset, bool restore)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    xcvrStatus_t xcvrstatus = gXcvrSuccess_c;
    /* Error checking for NULL pointer */
    if (desc_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        if (!restore)
        {
            /* Performing backup, need to prepare the PKT RAM with uncompressed descriptor entries */
            uint32_t * load_ptr = (uint32_t *)(pkt_ram_bank == TX_PKT_RAM_SEL ? (TX_PACKET_RAM_BASE) : (RX_PACKET_RAM_BASE));
            xcvrstatus = XCVR_FastPeriphDescrip_Load( desc_ptr, num_entries, &(load_ptr[pkt_ram_index_offset]));
            if (xcvrstatus != gXcvrSuccess_c)
            {
              status = gXcvrLclStatusFail;
            }
        }
        XCVR_FastPeriphReg_UpDownload_Go(pkt_ram_bank, pkt_ram_index_offset, restore);
    }
    return status;
}

#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */

xcvrLclStatus_t XCVR_LCL_GetTsmTimings(xcvr_lcl_tsm_config_t *backup_tsm_timings)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (backup_tsm_timings == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Backup TSM timings before re-configuration for RSM */
        /* CTRL register value is not modified for RSM operation. */
        /* LPPS_CTRL register value is not modified for RSM operation. */
        backup_tsm_timings->END_OF_SEQ = XCVR_TSM->END_OF_SEQ;
        backup_tsm_timings->WU_LATENCY = XCVR_TSM->WU_LATENCY;
        backup_tsm_timings->RECYCLE_COUNT = XCVR_TSM->RECYCLE_COUNT;
        backup_tsm_timings->FAST_CTRL1 = XCVR_TSM->FAST_CTRL1;
        backup_tsm_timings->FAST_CTRL2 = XCVR_TSM->FAST_CTRL2;
        backup_tsm_timings->FAST_CTRL3 = XCVR_TSM->FAST_CTRL3;
        /* TIMING00 register value is not modified for RSM operation. */
        /* TIMING01 register value is not modified for RSM operation. */
        /* TIMING02 register value is not modified for RSM operation. */
        /* TIMING03 register value is not modified for RSM operation. */
        /* TIMING04 register value is not modified for RSM operation. */
        /* TIMING05 register value is not modified for RSM operation. */
        /* TIMING06 register value is not modified for RSM operation. */
        /* TIMING07 register value is not modified for RSM operation. */
        /* TIMING08 register value is not modified for RSM operation. */
        backup_tsm_timings->TIMING09 = XCVR_TSM->TIMING09;
        backup_tsm_timings->TIMING10 = XCVR_TSM->TIMING10; 
        backup_tsm_timings->TIMING11 = XCVR_TSM->TIMING11; 
        backup_tsm_timings->TIMING12 = XCVR_TSM->TIMING12; 
        backup_tsm_timings->TIMING13 = XCVR_TSM->TIMING13; 
        backup_tsm_timings->TIMING14 = XCVR_TSM->TIMING14; 
        backup_tsm_timings->TIMING15 = XCVR_TSM->TIMING15; 
        backup_tsm_timings->TIMING16 = XCVR_TSM->TIMING16; 
        backup_tsm_timings->TIMING17 = XCVR_TSM->TIMING17; 
        backup_tsm_timings->TIMING18 = XCVR_TSM->TIMING18; 
        backup_tsm_timings->TIMING19 = XCVR_TSM->TIMING19; 
        backup_tsm_timings->TIMING20 = XCVR_TSM->TIMING20; 
        backup_tsm_timings->TIMING21 = XCVR_TSM->TIMING21; 
        backup_tsm_timings->TIMING22 = XCVR_TSM->TIMING22; 
        backup_tsm_timings->TIMING23 = XCVR_TSM->TIMING23; 
        backup_tsm_timings->TIMING24 = XCVR_TSM->TIMING24; 
        backup_tsm_timings->TIMING25 = XCVR_TSM->TIMING25; 
        backup_tsm_timings->TIMING26 = XCVR_TSM->TIMING26; 
        backup_tsm_timings->TIMING27 = XCVR_TSM->TIMING27; 
        backup_tsm_timings->TIMING28 = XCVR_TSM->TIMING28; 
        backup_tsm_timings->TIMING29 = XCVR_TSM->TIMING29; 
        backup_tsm_timings->TIMING30 = XCVR_TSM->TIMING30; 
        backup_tsm_timings->TIMING31 = XCVR_TSM->TIMING31; 
        backup_tsm_timings->TIMING32 = XCVR_TSM->TIMING32; 
        backup_tsm_timings->TIMING33 = XCVR_TSM->TIMING33; 
        backup_tsm_timings->TIMING34 = XCVR_TSM->TIMING34; 
        backup_tsm_timings->TIMING35 = XCVR_TSM->TIMING35; 
        backup_tsm_timings->TIMING36 = XCVR_TSM->TIMING36; 
        backup_tsm_timings->TIMING37 = XCVR_TSM->TIMING37; 
        backup_tsm_timings->TIMING38 = XCVR_TSM->TIMING38; 
        backup_tsm_timings->TIMING39 = XCVR_TSM->TIMING39; 
        backup_tsm_timings->TIMING40 = XCVR_TSM->TIMING40; 
        backup_tsm_timings->TIMING41 = XCVR_TSM->TIMING41; 
        backup_tsm_timings->TIMING42 = XCVR_TSM->TIMING42; 
        backup_tsm_timings->TIMING43 = XCVR_TSM->TIMING43; 
        backup_tsm_timings->TIMING44 = XCVR_TSM->TIMING44; 
        backup_tsm_timings->TIMING45 = XCVR_TSM->TIMING45; 
        backup_tsm_timings->TIMING46 = XCVR_TSM->TIMING46; 
        backup_tsm_timings->TIMING47 = XCVR_TSM->TIMING47; 
        backup_tsm_timings->TIMING48 = XCVR_TSM->TIMING48; 
        backup_tsm_timings->TIMING49 = XCVR_TSM->TIMING49; 
        backup_tsm_timings->TIMING50 = XCVR_TSM->TIMING50; 
        backup_tsm_timings->TIMING51 = XCVR_TSM->TIMING51; 
        /* TIMING52 register value is not modified for RSM operation. */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
        backup_tsm_timings->TIMING53 = XCVR_TSM->TIMING53; 
        backup_tsm_timings->TIMING54 = XCVR_TSM->TIMING54; 
        backup_tsm_timings->TIMING55 = XCVR_TSM->TIMING55; 
        backup_tsm_timings->TIMING56 = XCVR_TSM->TIMING56; 
        backup_tsm_timings->TIMING57 = XCVR_TSM->TIMING57; 
        backup_tsm_timings->TIMING58 = XCVR_TSM->TIMING58; 
        backup_tsm_timings->TIMING59 = XCVR_TSM->TIMING59; 
        backup_tsm_timings->TIMING60 = XCVR_TSM->TIMING60; 
        backup_tsm_timings->TIMING61 = XCVR_TSM->TIMING61; 
#endif        
        /* OVRD0 register value is not modified for RSM operation. */
        /* OVRD1 register value is not modified for RSM operation. */
        /* OVRD2 register value is not modified for RSM operation. */
        /* OVRD3 register value is not modified for RSM operation. */
        /* OVRD4 register value is not modified for RSM operation. */
    }

    return status;
}



xcvrLclStatus_t XCVR_LCL_ReprogramTsmTimings(const xcvr_lcl_tsm_config_t *new_tsm_timings)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (new_tsm_timings == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Program new TSM timings based  on input TSM settings and RSM configurations */

        /***********************************************/
        /*********** START OF GENERATED CODE ***********/
        /*********** xcvr_lcl_tsm_config_t **********/
        /***********************************************/
        /* CTRL register value is not modified for RSM operation. */
        /* LPPS_CTRL register value is not modified for RSM operation. */
        XCVR_TSM->END_OF_SEQ    = new_tsm_timings->END_OF_SEQ;
        XCVR_TSM->WU_LATENCY    = new_tsm_timings->WU_LATENCY;
        XCVR_TSM->RECYCLE_COUNT = new_tsm_timings->RECYCLE_COUNT;
        XCVR_TSM->FAST_CTRL1    = new_tsm_timings->FAST_CTRL1;
        XCVR_TSM->FAST_CTRL2    = new_tsm_timings->FAST_CTRL2;
        XCVR_TSM->FAST_CTRL3    = new_tsm_timings->FAST_CTRL3;
        /* TIMING00 register value is not modified for RSM operation. */
        /* TIMING01 register value is not modified for RSM operation. */
        /* TIMING02 register value is not modified for RSM operation. */
        /* TIMING03 register value is not modified for RSM operation. */
        /* TIMING04 register value is not modified for RSM operation. */
        /* TIMING05 register value is not modified for RSM operation. */
        /* TIMING06 register value is not modified for RSM operation. */
        /* TIMING07 register value is not modified for RSM operation. */
        /* TIMING08 register value is not modified for RSM operation. */
        XCVR_TSM->TIMING09 = new_tsm_timings->TIMING09;
        XCVR_TSM->TIMING10 = new_tsm_timings->TIMING10;
        XCVR_TSM->TIMING11 = new_tsm_timings->TIMING11;
        XCVR_TSM->TIMING12 = new_tsm_timings->TIMING12;
        XCVR_TSM->TIMING13 = new_tsm_timings->TIMING13;
        XCVR_TSM->TIMING14 = new_tsm_timings->TIMING14;
        XCVR_TSM->TIMING15 = new_tsm_timings->TIMING15;
        XCVR_TSM->TIMING16 = new_tsm_timings->TIMING16;
        XCVR_TSM->TIMING17 = new_tsm_timings->TIMING17;
        XCVR_TSM->TIMING18 = new_tsm_timings->TIMING18;
        XCVR_TSM->TIMING19 = new_tsm_timings->TIMING19;
        XCVR_TSM->TIMING20 = new_tsm_timings->TIMING20;
        XCVR_TSM->TIMING21 = new_tsm_timings->TIMING21;
        XCVR_TSM->TIMING22 = new_tsm_timings->TIMING22;
        XCVR_TSM->TIMING23 = new_tsm_timings->TIMING23;
        XCVR_TSM->TIMING24 = new_tsm_timings->TIMING24;
        XCVR_TSM->TIMING25 = new_tsm_timings->TIMING25;
        XCVR_TSM->TIMING26 = new_tsm_timings->TIMING26;
        XCVR_TSM->TIMING27 = new_tsm_timings->TIMING27;
        XCVR_TSM->TIMING28 = new_tsm_timings->TIMING28;
        XCVR_TSM->TIMING29 = new_tsm_timings->TIMING29;
        XCVR_TSM->TIMING30 = new_tsm_timings->TIMING30;
        XCVR_TSM->TIMING31 = new_tsm_timings->TIMING31;
        XCVR_TSM->TIMING32 = new_tsm_timings->TIMING32;
        XCVR_TSM->TIMING33 = new_tsm_timings->TIMING33;
        XCVR_TSM->TIMING34 = new_tsm_timings->TIMING34;
        XCVR_TSM->TIMING35 = new_tsm_timings->TIMING35;
        XCVR_TSM->TIMING36 = new_tsm_timings->TIMING36;
        XCVR_TSM->TIMING37 = new_tsm_timings->TIMING37;
        XCVR_TSM->TIMING38 = new_tsm_timings->TIMING38;
        XCVR_TSM->TIMING39 = new_tsm_timings->TIMING39;
        XCVR_TSM->TIMING40 = new_tsm_timings->TIMING40;
        XCVR_TSM->TIMING41 = new_tsm_timings->TIMING41;
        XCVR_TSM->TIMING42 = new_tsm_timings->TIMING42;
        XCVR_TSM->TIMING43 = new_tsm_timings->TIMING43;
        XCVR_TSM->TIMING44 = new_tsm_timings->TIMING44;
        XCVR_TSM->TIMING45 = new_tsm_timings->TIMING45;
        XCVR_TSM->TIMING46 = new_tsm_timings->TIMING46;
        XCVR_TSM->TIMING47 = new_tsm_timings->TIMING47;
        XCVR_TSM->TIMING48 = new_tsm_timings->TIMING48;
        XCVR_TSM->TIMING49 = new_tsm_timings->TIMING49;
        XCVR_TSM->TIMING50 = new_tsm_timings->TIMING50;
        XCVR_TSM->TIMING51 = new_tsm_timings->TIMING51;
        /* TIMING52 register value is not modified for RSM operation. */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
        XCVR_TSM->TIMING53 = new_tsm_timings->TIMING53;
        XCVR_TSM->TIMING54 = new_tsm_timings->TIMING54;
        XCVR_TSM->TIMING55 = new_tsm_timings->TIMING55;
        XCVR_TSM->TIMING56 = new_tsm_timings->TIMING56;
        XCVR_TSM->TIMING57 = new_tsm_timings->TIMING57;
        XCVR_TSM->TIMING58 = new_tsm_timings->TIMING58;
        XCVR_TSM->TIMING59 = new_tsm_timings->TIMING59;
        XCVR_TSM->TIMING60 = new_tsm_timings->TIMING60;
        XCVR_TSM->TIMING61 = new_tsm_timings->TIMING61;
#endif        
        /* OVRD0 register value is not modified for RSM operation. */
        /* OVRD1 register value is not modified for RSM operation. */
        /* OVRD2 register value is not modified for RSM operation. */
        /* OVRD3 register value is not modified for RSM operation. */

        /***********************************************/
        /************ END OF GENERATED CODE ************/
        /***********   xcvr_lcl_tsm_config_t   **********/
        /***********************************************/
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmRegBackup(rsm_reg_backup_t *reg_backup_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (reg_backup_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* XCVR_MISC */
#if (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))        
        reg_backup_ptr->XCVR_MISC_DMA_CTRL            = XCVR_MISC->DMA_CTRL;
        reg_backup_ptr->XCVR_MISC_LCL_CFG0            = XCVR_MISC->LCL_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_CFG1            = XCVR_MISC->LCL_CFG1;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG0         = XCVR_MISC->LCL_TX_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG1         = XCVR_MISC->LCL_TX_CFG1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_MISC_RSM_CTRL6           = XCVR_MISC->RSM_CTRL6;
#else
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG2         = XCVR_MISC->LCL_TX_CFG2;
#endif
        reg_backup_ptr->XCVR_MISC_LCL_RX_CFG0         = XCVR_MISC->LCL_RX_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_RX_CFG1         = XCVR_MISC->LCL_RX_CFG1;
        reg_backup_ptr->XCVR_MISC_LCL_RX_CFG2         = XCVR_MISC->LCL_RX_CFG2;
        reg_backup_ptr->XCVR_MISC_LCL_PM_MSB          = XCVR_MISC->LCL_PM_MSB;
        reg_backup_ptr->XCVR_MISC_LCL_PM_LSB          = XCVR_MISC->LCL_PM_LSB;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL0      = XCVR_MISC->LCL_GPIO_CTRL0;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL1      = XCVR_MISC->LCL_GPIO_CTRL1;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL2      = XCVR_MISC->LCL_GPIO_CTRL2;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL3      = XCVR_MISC->LCL_GPIO_CTRL3;
        reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL4      = XCVR_MISC->LCL_GPIO_CTRL4;
        reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_DELAY  = XCVR_MISC->LCL_DMA_MASK_DELAY;
        reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_PERIOD = XCVR_MISC->LCL_DMA_MASK_PERIOD;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL0           = XCVR_MISC->RSM_CTRL0;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL1           = XCVR_MISC->RSM_CTRL1;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL2           = XCVR_MISC->RSM_CTRL2;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL3           = XCVR_MISC->RSM_CTRL3;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL4           = XCVR_MISC->RSM_CTRL4;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_MISC_RSM_CTRL5           = XCVR_MISC->RSM_CTRL5;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL6           = XCVR_MISC->RSM_CTRL6;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL7           = XCVR_MISC->RSM_CTRL7;
        reg_backup_ptr->XCVR_MISC_RSM_INT_ENABLE   = XCVR_MISC->RSM_INT_ENABLE;
#endif
#endif /* BACKUP_LCL_REGS */
        /* XCVR_2P4GHZ_PHY */
        reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_CTRL = XCVR_2P4GHZ_PHY->RTT_CTRL;
        reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_REF  = XCVR_2P4GHZ_PHY->RTT_REF;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_2P4GHZ_PHY_DMD_CTRL1= XCVR_2P4GHZ_PHY->DMD_CTRL1;
        reg_backup_ptr->XCVR_2P4GHZ_PHY_DMD_CTRL2= XCVR_2P4GHZ_PHY->DMD_CTRL2;
#endif

        /* XCVR_TXDIG */
        reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL  = XCVR_TX_DIG->DATA_PADDING_CTRL;
        reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL1 = XCVR_TX_DIG->DATA_PADDING_CTRL_1;
        reg_backup_ptr->XCVR_TX_DIG_GFSK_CTRL = XCVR_TX_DIG->GFSK_CTRL;
        reg_backup_ptr->XCVR_TX_DIG_PA_CTRL            = XCVR_TX_DIG->PA_CTRL;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->PA_RAMP_TBL0            = XCVR_TX_DIG->PA_RAMP_TBL0;
        reg_backup_ptr->PA_RAMP_TBL1            = XCVR_TX_DIG->PA_RAMP_TBL1;
        reg_backup_ptr->PA_RAMP_TBL2            = XCVR_TX_DIG->PA_RAMP_TBL2;
        reg_backup_ptr->PA_RAMP_TBL3            = XCVR_TX_DIG->PA_RAMP_TBL3;
#endif
        /* XCVR_PLL */
        status = XCVR_LCL_RsmPLLBackup(reg_backup_ptr); /* NULLPTR check in this routine should never fail since checked above */
        /* XCVR_RXDIG */
        reg_backup_ptr->XCVR_RX_DIG_RCCAL_CTRL1 = XCVR_RX_DIG->RCCAL_CTRL1;
        reg_backup_ptr->XCVR_RX_DIG_DFT_CTRL    = XCVR_RX_DIG->DFT_CTRL;
        reg_backup_ptr->XCVR_RX_DIG_CTRL1       = XCVR_RX_DIG->CTRL1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        reg_backup_ptr->XCVR_RX_DIG_AGC_CTRL = XCVR_RX_DIG->AGC_CTRL;
#endif
        /* RADIO_CTRL */
        reg_backup_ptr->RADIO_CTRL_RF_CTRL = RADIO_CTRL->RF_CTRL;
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmRegRestore(const rsm_reg_backup_t *reg_backup_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (reg_backup_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* XCVR_MISC */
#if (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))        
        XCVR_MISC->DMA_CTRL            = reg_backup_ptr->XCVR_MISC_DMA_CTRL;
        XCVR_MISC->LCL_CFG0            = reg_backup_ptr->XCVR_MISC_LCL_CFG0;
        XCVR_MISC->LCL_CFG1            = reg_backup_ptr->XCVR_MISC_LCL_CFG1;
        XCVR_MISC->LCL_TX_CFG0         = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG0;
        XCVR_MISC->LCL_TX_CFG1         = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_MISC->RSM_CTRL6 = reg_backup_ptr->XCVR_MISC_RSM_CTRL6;
#else
        XCVR_MISC->LCL_TX_CFG2         = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG2;
#endif
        XCVR_MISC->LCL_RX_CFG0         = reg_backup_ptr->XCVR_MISC_LCL_RX_CFG0;
        XCVR_MISC->LCL_RX_CFG1         = reg_backup_ptr->XCVR_MISC_LCL_RX_CFG1;
        XCVR_MISC->LCL_RX_CFG2         = reg_backup_ptr->XCVR_MISC_LCL_RX_CFG2;
        XCVR_MISC->LCL_PM_MSB          = reg_backup_ptr->XCVR_MISC_LCL_PM_MSB;
        XCVR_MISC->LCL_PM_LSB          = reg_backup_ptr->XCVR_MISC_LCL_PM_LSB;
        XCVR_MISC->LCL_GPIO_CTRL0      = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL0;
        XCVR_MISC->LCL_GPIO_CTRL1      = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL1;
        XCVR_MISC->LCL_GPIO_CTRL2      = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL2;
        XCVR_MISC->LCL_GPIO_CTRL3      = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL3;
        XCVR_MISC->LCL_GPIO_CTRL4      = reg_backup_ptr->XCVR_MISC_LCL_GPIO_CTRL4;
        XCVR_MISC->LCL_DMA_MASK_DELAY  = reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_DELAY;
        XCVR_MISC->LCL_DMA_MASK_PERIOD = reg_backup_ptr->XCVR_MISC_LCL_DMA_MASK_PERIOD;
        XCVR_MISC->RSM_CTRL0           = reg_backup_ptr->XCVR_MISC_RSM_CTRL0;
        XCVR_MISC->RSM_CTRL1           = reg_backup_ptr->XCVR_MISC_RSM_CTRL1;
        XCVR_MISC->RSM_CTRL2           = reg_backup_ptr->XCVR_MISC_RSM_CTRL2;
        XCVR_MISC->RSM_CTRL3           = reg_backup_ptr->XCVR_MISC_RSM_CTRL3;
        XCVR_MISC->RSM_CTRL4           = reg_backup_ptr->XCVR_MISC_RSM_CTRL4;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_MISC->RSM_CTRL5 = reg_backup_ptr->XCVR_MISC_RSM_CTRL5;
        XCVR_MISC->RSM_CTRL6 = reg_backup_ptr->XCVR_MISC_RSM_CTRL6;
        XCVR_MISC->RSM_CTRL7 = reg_backup_ptr->XCVR_MISC_RSM_CTRL7;
        XCVR_MISC->RSM_INT_ENABLE = reg_backup_ptr->XCVR_MISC_RSM_INT_ENABLE;
#endif
#endif /* (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1)) */   

        /* XCVR_2P4GHZ_PHY */
        XCVR_2P4GHZ_PHY->RTT_CTRL = reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_CTRL;
        XCVR_2P4GHZ_PHY->RTT_REF  = reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_REF;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_2P4GHZ_PHY->DMD_CTRL1 = reg_backup_ptr->XCVR_2P4GHZ_PHY_DMD_CTRL1;
        XCVR_2P4GHZ_PHY->DMD_CTRL2 = reg_backup_ptr->XCVR_2P4GHZ_PHY_DMD_CTRL2;
#endif
        /* XCVR_TXDIG */
        XCVR_TX_DIG->DATA_PADDING_CTRL   = reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL;
        XCVR_TX_DIG->DATA_PADDING_CTRL_1 = reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL1;
        XCVR_TX_DIG->GFSK_CTRL = reg_backup_ptr->XCVR_TX_DIG_GFSK_CTRL;
        XCVR_TX_DIG->PA_CTRL             = reg_backup_ptr->XCVR_TX_DIG_PA_CTRL;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        XCVR_TX_DIG->PA_RAMP_TBL0 = reg_backup_ptr->PA_RAMP_TBL0;
        XCVR_TX_DIG->PA_RAMP_TBL1 = reg_backup_ptr->PA_RAMP_TBL1;
        XCVR_TX_DIG->PA_RAMP_TBL2 = reg_backup_ptr->PA_RAMP_TBL2;
        XCVR_TX_DIG->PA_RAMP_TBL3 = reg_backup_ptr->PA_RAMP_TBL3;
#endif
        /* XCVR_PLL */
        status = XCVR_LCL_RsmPLLRestore(reg_backup_ptr); /* NULLPTR check in this routine should never fail since checked above */
        /* XCVR_RXDIG */
        XCVR_RX_DIG->RCCAL_CTRL1 = reg_backup_ptr->XCVR_RX_DIG_RCCAL_CTRL1;
        XCVR_RX_DIG->DFT_CTRL    = reg_backup_ptr->XCVR_RX_DIG_DFT_CTRL;
        XCVR_RX_DIG->CTRL1       = reg_backup_ptr->XCVR_RX_DIG_CTRL1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
        XCVR_RX_DIG->AGC_CTRL = reg_backup_ptr->XCVR_RX_DIG_AGC_CTRL;
#endif
        /* RADIO_CTRL */
        RADIO_CTRL->RF_CTRL = reg_backup_ptr->RADIO_CTRL_RF_CTRL;
    }

    return status;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
xcvrLclStatus_t XCVR_LCL_SetFstepRam(const xcvr_lcl_fstep_t *fstep_settings, uint16_t num_steps)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if (fstep_settings == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status =
            XCVR_LCL_RsmCheckSeqLen(num_steps, RSM_FSTEP_RAM_COUNT); /* check that sequence length is not exceeded */
    }

    /* Copy all of the settings entries into packet RAM location */
    if (status == gXcvrLclStatusSuccess)
    {
        uint8_t pn_step_count = XCVR_LCL_CountPnRttSteps(fstep_settings, num_steps); /* calculate  the PN step count */
        if (pn_step_count > XCVR_RSM_FCS_PKPK_MAX_STEP_COUNT)
        {
            status = gXcvrLclStatusInvalidLength; /* too many PN steps */
        }
        else
        {
            const void *ptr = memcpy((uint32_t *)RSM_FSTEP_RAM, (const void *)fstep_settings,
                                     ((uint32_t)num_steps * RSM_FSTEP_ENTRY_SZ));
            /* memcpy returns the first argument which cannot be NULLPTR since it is tested above in this module  */
            (void)ptr; /* touch the result to "use" it */
        }
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_SetPnRamShort(const xcvr_lcl_pn32_config_t *pn_values, uint16_t num_steps)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if (pn_values == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status =
            XCVR_LCL_RsmCheckSeqLen(num_steps, RSM_PN_RAM_32_COUNT); /* check that sequence length is not exceeded */
    }

    /* Copy settings into packet RAM location */
    if (status == gXcvrLclStatusSuccess)
    {
        const void *ptr =
            memcpy((uint32_t *)RSM_PN_RAM, (const void *)pn_values, ((uint32_t)num_steps * RSM_PN_RAM_32_ENTRY_SZ));
        /* memcpy returns the first argument which cannot be NULLPTR since it is tested above in this module  */
        (void)ptr; /* touch the result to "use" it */
    }

    return status;
}

#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
xcvrLclStatus_t XCVR_LCL_SetPnRamLong(const xcvr_lcl_pn64_config_t *pn_values, uint16_t num_steps)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if (pn_values == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status =
            XCVR_LCL_RsmCheckSeqLen(num_steps, RSM_PN_RAM_64_COUNT); /* check that sequence length is not exceeded */
    }

    /* Copy settings into packet RAM location */
    if (status == gXcvrLclStatusSuccess)
    {
        const void *ptr =
            memcpy((uint32_t *)RSM_PN_RAM, (const void *)pn_values, ((uint32_t)num_steps * RSM_PN_RAM_64_ENTRY_SZ));
        /* memcpy returns the first argument which cannot be NULLPTR since it is tested above in this module  */
        (void)ptr; /* touch the result to "use" it */
    }

    return status;
}
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */

xcvrLclStatus_t XCVR_LCL_GetCtuneResults(uint8_t *ctune_results, uint16_t num_steps)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if (ctune_results == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status =
            XCVR_LCL_RsmCheckSeqLen(num_steps, RSM_PCBD_RAM_COUNT); /* check that sequence length is not exceeded */
    }

    /* Copy settings out of packet RAM location */
    if (status == gXcvrLclStatusSuccess)
    {
        const void *ptr =
            memcpy((void *)ctune_results, (const uint32_t *)RSM_PCBD_RAM, ((uint32_t)num_steps * RSM_PCBD_ENTRY_SZ));
        /* memcpy returns the first argument which cannot be NULLPTR since it is tested above in this module  */
        (void)ptr; /* touch the result to "use" it */
    }

    return status;
}
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) */

xcvrLclStatus_t XCVR_LCL_GetRttResults(xcvr_lcl_rtt_data_raw_t *rtt_results, uint16_t num_steps)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if (rtt_results == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status = XCVR_LCL_RsmCheckSeqLen(num_steps, RSM_RTT_RAM_COUNT); /* check that sequence length is not exceeded */
    }

    /* Copy settings out of packet RAM location */
    if (status == gXcvrLclStatusSuccess)
    {
        const void *ptr =
            memcpy((void *)rtt_results, (const uint32_t *)RSM_RTT_RAM, ((uint32_t)num_steps * RSM_RTT_RAM_ENTRY_SZ));
        /* memcpy returns the first argument which cannot be NULLPTR since it is tested above in this module  */
        (void)ptr; /* touch the result to "use" it */
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_UnpackRttResult(const xcvr_lcl_rtt_data_raw_t *rtt_results,
                                         xcvr_lcl_rtt_data_t *rtt_unpacked,
                                         XCVR_RSM_SQTE_RATE_T rate)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if ((rtt_results == NULLPTR) || (rtt_unpacked == NULLPTR))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Unpack the RTT raw results into separate entries */
        uint32_t packed_data;
        uint32_t temp;
        int32_t temp_cfo;
        int8_t cfo_div = 4; /* used in integer math to create 15.25 multiplier for raw CFO conversion at 1Mbps */
        if (rate == XCVR_RSM_RATE_2MBPS)
        {
            cfo_div = 2; /* used in integer math to create 30.5 multiplier for raw CFO conversion at 2Mbps */
        }
        packed_data = B3(rtt_results->rtt_data_b3) | B2(rtt_results->rtt_data_b2) | B1(rtt_results->rtt_data_b1) |
                      B0(rtt_results->rtt_data_b0_success);
        rtt_unpacked->rtt_vld   = ((packed_data & XCVR_RSM_RTT_VALID_MASK) != 0U); /* boolean result */
        rtt_unpacked->rtt_found = ((packed_data & XCVR_RSM_RTT_FOUND_MASK) != 0U); /* boolean result */
        temp                    = (packed_data & XCVR_RSM_RTT_CFO_MASK) >> XCVR_RSM_RTT_CFO_SHIFT;
        temp_cfo = (int16_t)temp; /* packed data is signed 16 bits, then work in signed 32 bits for conversion to Hz */
        temp_cfo = (temp_cfo * 61) / cfo_div; /* 15.25Hz (or 30.5Hz) per LSB, implement in integer math as divide by 4
                                                 (or 2) and multiply by 61 */
        rtt_unpacked->cfo = temp_cfo;
        temp              = (packed_data & XCVR_RSM_RTT_INT_ADJ_MASK) >>
               XCVR_RSM_RTT_INT_ADJ_SHIFT; /* work in 32 bits before casting smaller */
        rtt_unpacked->int_adj = (uint8_t)temp;
        temp                  = (packed_data & XCVR_RSM_RTT_HAM_DIST_SAT_MASK) >>
               XCVR_RSM_RTT_HAM_DIST_SAT_SHIFT; /* work in 32 bits before casting smaller */
        rtt_unpacked->ham_dist_sat = (uint8_t)temp;
        temp                       = (packed_data & XCVR_RSM_RTT_P_DELTA_MASK) >>
               XCVR_RSM_RTT_P_DELTA_SHIFT; /* work in 32 bits before casting smaller */
        rtt_unpacked->p_delta = (uint16_t)temp;
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_MakeFstep(xcvr_lcl_fstep_t *fstep_entry,
                                   uint16_t channel_num,
                                   uint8_t ctune,
                                   uint16_t hpm_cal,
                                   XCVR_RSM_FSTEP_TYPE_T step_format,
                                   XCVR_RSM_T_PM_FM_SEL_T t_pm_sel)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if (fstep_entry == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* check for errors in the input data */
        /* consider making the error checking process optional for speed */
#if defined(CHECK_FSTEP_ERRORS)
        if (channel_num > 127)
        {
            status = gXcvrLclStatusInvalidArgs;
        }
        if (hpm_cal & 0xF000U != 0U) /* unexpected bits are set in the HPM CAL value */
        {
            status = gXcvrLclStatusInvalidArgs;
        }
#endif /* defined(CHECK_FSTEP_ERRORS) */
    }

    if (status == gXcvrLclStatusSuccess)
    {
        /* Create one Frequency Step structure at the pointed location */
        fstep_entry->ext_channel_num_ovrd_lsb = (uint8_t)(channel_num & 0xFFU);
        fstep_entry->ext_channel_num_ovrd_msb = (uint8_t)((channel_num & 0xFF00U) >> 8U);
        fstep_entry->ext_ctune                = ctune;
        fstep_entry->ext_hmp_cal_factor_lsb   = (uint8_t)((hpm_cal & 0x1FEU) >> 1U); /* Bits [8:1] */
        static uint8_t temp1, temp2, temp3;
        temp1 = XCVR_RSM_HPM_CAL_MSB((uint8_t)((hpm_cal & 0x01E00U) >> 9U));
        temp2 = XCVR_RSM_STEP_FORMAT((uint8_t)step_format);
        temp3 = XCVR_RSM_T_PM_FM_SEL((uint8_t)t_pm_sel);
        (void)temp1;
        (void)temp2;
        (void)temp3;
        fstep_entry->tpm_step_format_hmp_cal_factor_msb =
            ((XCVR_RSM_HPM_CAL_MSB((uint8_t)((hpm_cal & 0x01E00U) >> 9U))) | /* Bits [12:9] */
             (XCVR_RSM_STEP_FORMAT((uint8_t)step_format)) | (XCVR_RSM_T_PM_FM_SEL((uint8_t)t_pm_sel)));
    }

    return status;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
xcvrLclStatus_t XCVR_LCL_ValidateLclSettings(xcvr_lcl_rsm_config_t * rsm_settings_ptr, XCVR_RSM_T_CAPTURE_SEL_T t_capture)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (rsm_settings_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Track args check by using all statements that evaluate to false (0 value) if param is ok */
        /* Sum up every status check and compare to zero at the end. If not == zero then return invalid args result */
        uint32_t tmp_logic_sum = 0U; 

        /* Check for valid t_capture values */
        /* Only support SQTE mode of RSM */
        tmp_logic_sum += (uint32_t)((t_capture != XCVR_RSM_T_CAPTURE_20_SEL) && 
#if defined(LCL_SUPPORT_10USEC_T_CAPTURE) && (LCL_SUPPORT_10USEC_T_CAPTURE == 1)
                                  (t_capture != XCVR_RSM_T_CAPTURE_10_SEL) &&
#endif  /* defined(LCL_SUPPORT_10USEC_T_CAPTURE) && (LCL_SUPPORT_10USEC_T_CAPTURE == 1) */
                                  (t_capture != XCVR_RSM_T_CAPTURE_40_SEL)  &&
                                 (t_capture !=  XCVR_RSM_T_CAPTURE_652_SEL));

        if (rsm_settings_ptr->num_ant_path > 1U)
        {
            /* Check for valid combination of T_SW and PA_ramping time */
            /* First make sure PA ramp is a valid value */
            tmp_logic_sum += (uint32_t)(rsm_settings_ptr->pa_ramp_time >=XCVR_RSM_PA_RAMP_ERROR); /* Evaluate to false when params are ok */

            /* T_SW==4 and PA_RAMP==1 */
            /* T_SW==10 and (any valid PA ramp time) */
            bool valid_combo = (rsm_settings_ptr->t_sw == lclTSw_4usec) && ((rsm_settings_ptr->pa_ramp_time == XCVR_RSM_PA_RAMP_1_USEC) || (rsm_settings_ptr->pa_ramp_time == XCVR_RSM_PA_RAMP_0_USEC));
            valid_combo |= (rsm_settings_ptr->t_sw == lclTSw_10usec); /* 10usec T_SW works with any valid PA ramp value (checked above) */
            tmp_logic_sum += (uint32_t)(!valid_combo); /* Evaluate to false when params are ok */
        }
    
        if (tmp_logic_sum != 0U)  /* If any of the above logic statements is true then the argument is invalid */
        {
          status = gXcvrLclStatusInvalidArgs;
        }
    }
    
    
    return status;
}

xcvrLclStatus_t XCVR_LCL_ConfigLclBlock(xcvr_lcl_rsm_config_t * rsm_settings_ptr, XCVR_RSM_T_CAPTURE_SEL_T t_capture, uint8_t *toneAntennaIDs_p, bool ena_antsw_pa_ramping)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
#if defined(XCVR_SKIP_RSM_SETTINGS_CHECK) && (XCVR_SKIP_RSM_SETTINGS_CHECK == 0)
    status = XCVR_LCL_ValidateLclSettings(rsm_settings_ptr, t_capture);
#endif /* defined(XCVR_SKIP_RSM_SETTINGS_CHECK) && (XCVR_SKIP_RSM_SETTINGS_CHECK == 0) */
    if (status == gXcvrLclStatusSuccess)
    {
        /* Setup LCL block for normal operations with 1 antenna */
        uint8_t t_pm = rsm_settings_ptr->t_pm0;
        uint8_t t_sw = rsm_settings_ptr->t_sw;
        uint8_t rttPhy = rsm_settings_ptr->rate;
        uint32_t spint_us, temp, offset, sampling_rate_factor;
        uint8_t temp_tx_sw_ena;
        xcvr_lcl_tqi_setting_tbl_t * tqi_tbl_ptr;
        /* Configure 2Mbps oversampling if needed */
        if (rttPhy == XCVR_RSM_RATE_1MBPS)
        {
            sampling_rate_factor = 1U;
            tqi_tbl_ptr = &tqi_1mbps_settings;
        }
        else
        {
            sampling_rate_factor = SAMPLING_RATE_FACTOR_2MBPS;
            tqi_tbl_ptr = &tqi_2mbps_settings;
        }

        /* Configure SPINT to 1 interval per usec */
        spint_us = 1;

        /* LCL basic config - enable PA ramping if requested and num antenna path > 1 */
        if ((ena_antsw_pa_ramping) && (rsm_settings_ptr->num_ant_path > 1U))
        {
            XCVR_MISC->LCL_CFG1 |= (XCVR_MISC_LCL_CFG1_ANT_SW_RF_MASK | XCVR_MISC_LCL_CFG1_ANT_SW_MODE3_MASK);
            temp_tx_sw_ena = 1U;
        }
        else
        {
            XCVR_MISC->LCL_CFG1 &= ~(XCVR_MISC_LCL_CFG1_ANT_SW_RF_MASK | XCVR_MISC_LCL_CFG1_ANT_SW_MODE3_MASK);
            temp_tx_sw_ena = 0U;
        }

        /* PA ramp down/up feature calculations */
        uint8_t t_ramp_usec = rsm_settings_ptr->pa_ramp_time;  /* Actual PA ramping time in usec; Valid values are 1, 2, 3, 4 (note: 3 requires reprogramming PA ramping) */
        offset = (TX_DATA_FLUSH_DLY_1MBPS+t_ramp_usec)*8U;
        uint8_t add_up;
        if ((2*t_ramp_usec) > t_sw)
        {
            add_up = 0U; 
        }
        else
        {
            add_up = t_sw-(2*t_ramp_usec);
            add_up = (add_up > 7U ? 7U : add_up); /* saturate to 7 maximum */
            add_up = (add_up >2 ? (add_up-2U) : 0U); /* saturate to 0 minimum */
        }
        
        /* ensure divisions do not have remainder */
        if ((rsm_settings_ptr->op_mode != XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE) &&
            (((t_capture+t_sw)*(rsm_settings_ptr->num_ant_path+1U)) != t_pm))
        {
            status = gXcvrLclStatusInvalidArgs;
        }
        else
        {
            /* RX/TX config */
            XCVR_MISC->LCL_TX_CFG0 = XCVR_MISC_LCL_TX_CFG0_TX_DELAY(0U) |
                                                        XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF(offset) |
                                                        XCVR_MISC_LCL_TX_CFG0_TX_SW_FRAC_OFFSET_DN(0U) |
                                                        XCVR_MISC_LCL_TX_CFG0_TX_SW_FRAC_OFFSET_UP(0U);
            
            XCVR_MISC->LCL_RX_CFG0 = XCVR_MISC_LCL_RX_CFG0_RX_DELAY(0U) |   /* Receive ant switch should align with on-the-air time */
                                                        XCVR_MISC_LCL_RX_CFG0_RX_DELAY_OFF(0U);

            
            uint16_t temp_t_capture;
            if (rsm_settings_ptr->op_mode == XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE)
            {
                /* Force manual SPINT and HI/LO period settings for 654usec long (2usec extra) T_PM period */
                /* Required due to the size of LCL HI_PER and LO_PER registers */
                spint_us = 9U; /* 9usec per interval */
                temp_t_capture = 109U; /* 109 intervals * 9usec per interval */
                t_sw =  1U; /* 1 interval * 9usec per interval */
            }
            else
            {
              temp_t_capture = t_capture;
            }
            temp = XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL(6)    /* RSM trigger */ |
                XCVR_MISC_LCL_RX_CFG1_RX_HI_PER(temp_t_capture) |       /* HI_PER in usec for KW47 */
                XCVR_MISC_LCL_RX_CFG1_RX_LO_PER(t_sw);                 /* LO_PER in usec for KW47 */

            /* TX and RX configs have same register mapping for HI/LOW period. SPINT is different due to sampling rate */
            XCVR_MISC->LCL_RX_CFG1 = temp | XCVR_MISC_LCL_RX_CFG1_RX_SPINT(RX_SAMPLING_RATE*sampling_rate_factor*spint_us-1U);
            XCVR_MISC->LCL_TX_CFG1 = temp | XCVR_MISC_LCL_TX_CFG1_TX_SPINT(TX_SAMPLING_RATE*sampling_rate_factor*spint_us-1U) |
                                                        XCVR_MISC_LCL_TX_CFG1_TX_SW_ACTIVE(temp_tx_sw_ena) | /* Enable power ramping in T_SW */
                                                        XCVR_MISC_LCL_TX_CFG1_TX_SW_OFFSET(0x0U) |
                                                        XCVR_MISC_LCL_TX_CFG1_TX_SW_ADD_OFFSET_UP(add_up);

            /* Configure TX+RX mode, and duration. All other fields are reset value (0) */
            XCVR_MISC->LCL_CFG0 = XCVR_MISC_LCL_CFG0_CTE_DUR(rsm_settings_ptr->num_ant_path) | 
                            XCVR_MISC_LCL_CFG0_LCL_ADJ_ENDMASK_MASK |
                            XCVR_MISC_LCL_CFG0_LCL_ANT_PERMUT_EN_MASK |
                            XCVR_MISC_LCL_CFG0_DELAY_SIGNED_MASK |
                            XCVR_MISC_LCL_CFG0_ANT_SYNC(rsm_settings_ptr->num_ant_path - 1U) |
                            XCVR_MISC_LCL_CFG0_AP_MAX(rsm_settings_ptr->num_ant_path - 1U)  |  /* AP_MAX is a zero based register representing values 1 to 4 */
                            XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | 
                            XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK |
                            XCVR_MISC_LCL_CFG0_LCL_EN_MASK; 

            XCVR_MISC->LCL_GPIO_CTRL4 = XCVR_MISC_LCL_GPIO_CTRL4_LUT_WRAP_PTR(rsm_settings_ptr->num_ant_path);
            temp = toneAntennaIDs_p[0] | (toneAntennaIDs_p[1] << 4U) | (toneAntennaIDs_p[2] << 8U) | (toneAntennaIDs_p[3] << 12U);
            XCVR_MISC->LCL_GPIO_CTRL0 = temp;
            
            temp = XCVR_RX_DIG->AGC_CTRL_STAT;
            temp &= ~(XCVR_RX_DIG_AGC_CTRL_STAT_AGC_MAX_IDX_MASK);
            temp |= XCVR_RX_DIG_AGC_CTRL_STAT_AGC_MAX_IDX(1U);
            XCVR_RX_DIG->AGC_CTRL_STAT = temp;
            
            XCVR_MISC->LCL_DMA_MASK_PERIOD = XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER(0U); /* Don't' want a reference period, only the regular centered windows */
            if (rttPhy == XCVR_RSM_RATE_1MBPS)
            {
                offset = 3U;
            }
            else
            {
                offset = 5U;
            }

            XCVR_MISC->LCL_DMA_MASK_DELAY = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(offset) | XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(5U);

            /* DMA mask centering & TQI setup */
            xcvr_lcl_tqi_entry_t * tqi_settings_ptr = NULLPTR;;
            temp = XCVR_MISC->DMA_MASK_CTRL;
            temp &= ~(XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER_MASK);
            switch (t_capture)
            {
                case XCVR_RSM_T_CAPTURE_652_SEL:
                case XCVR_RSM_T_CAPTURE_40_SEL:
                    temp |= XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER(lclDmaMask32usecCenter);
                    tqi_settings_ptr = &(tqi_tbl_ptr->t_slot_40usec_tqi);
                    break;
                case XCVR_RSM_T_CAPTURE_20_SEL:
                    temp |= XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER(lclDmaMask16usecCenter);
                    tqi_settings_ptr = &(tqi_tbl_ptr->t_slot_20usec_tqi);
                    break;
#if defined(LCL_SUPPORT_10USEC_T_CAPTURE) && (LCL_SUPPORT_10USEC_T_CAPTURE == 1)
                case XCVR_RSM_T_CAPTURE_10_SEL:
                    temp |= XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER(lclDmaMask8usecCenter);
                    tqi_settings_ptr = &(tqi_tbl_ptr->t_slot_10usec_tqi);
                    break;
#endif  /* defined(LCL_SUPPORT_10USEC_T_CAPTURE) && (LCL_SUPPORT_10USEC_T_CAPTURE == 1) */
                default:
                    temp |= XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER(lclDmaMaskNoCenter);
                    status = gXcvrLclStatusInvalidArgs;
            }
            XCVR_MISC->DMA_MASK_CTRL = temp;
            if (tqi_settings_ptr != NULLPTR)
            {
                /* Apply the TQI settings and enable TQI */
                XCVR_RX_DIG->TQI_CTRL = (XCVR_RX_DIG_TQI_CTRL_TQI_EN(1U) |
                                                            XCVR_RX_DIG_TQI_CTRL_IQ_AVG_DPTH(tqi_settings_ptr->iq_depth)  |
                                                            XCVR_RX_DIG_TQI_CTRL_MAG_AVG_DPTH(tqi_settings_ptr->mag_depth) );
                XCVR_RX_DIG->TQI_THR = (XCVR_RX_DIG_TQI_THR_INLINE_THR_TQI(2U) |
                                                            XCVR_RX_DIG_TQI_THR_T1((tqi_settings_ptr->t1)) |
                                                            XCVR_RX_DIG_TQI_THR_T2((tqi_settings_ptr->t2)));
            }
        }
    }
     
    return status;
}

#else  /* KW45 version */

#define T_CAPTURE_DELAY (17U) /* accounts for RX warmup delay + data path latency. Measured. */
#define T_TX_ANT_SW_DELAY (2U) /* to align lant_sw signals on RX and TX side. RX TX_PM state starts 2us earlier (TX_DATA_FLUSH_DLY). */
xcvrLclStatus_t XCVR_LCL_ConfigLclBlock(xcvr_lcl_rsm_config_t * rsm_settings_ptr,  XCVR_RSM_T_CAPTURE_SEL_T  ant_slot_time)
{ 
    uint32_t spint_us, temp, offset, sampling_rate_factor;
    uint32_t t_capture = rsm_settings_ptr->rsm_dma_dur_pm; /* Re-use of RSM setting for LCL block */
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    
    /* Configure 2Mbps oversampling if needed */
    sampling_rate_factor = (rsm_settings_ptr->rate == XCVR_RSM_RATE_1MBPS) ? 1U : SAMPLING_RATE_FACTOR_2MBPS;
    
    /* Configure SPINT according to T_PM duration in order not to overflow SPINT and HI/LO_PER 5bits values */
    if ((ant_slot_time == XCVR_RSM_T_CAPTURE_20_SEL) || (ant_slot_time == XCVR_RSM_T_CAPTURE_40_SEL))
        spint_us = 2;
    else
        spint_us = 1;

    /* RX/TX config */
    offset = (T_TX_ANT_SW_DELAY / spint_us);
    temp = XCVR_MISC_LCL_TX_CFG0_TX_DELAY(offset);
    offset = (T_TX_ANT_SW_DELAY % spint_us);
    temp |= XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF(offset*TX_SAMPLING_RATE*sampling_rate_factor);
    
    XCVR_MISC->LCL_RX_CFG0 = 0; /* trigger delay = 0 */
    XCVR_MISC->LCL_TX_CFG0 = temp; /* trigger delay = T_TX_ANT_SW_DELAY */
    
/* ensure divisions do not have remainder */
    if (((t_capture / spint_us) + ((ant_slot_time + (uint32_t)rsm_settings_ptr->t_sw - t_capture) / spint_us)) * spint_us != ((uint32_t)rsm_settings_ptr->t_sw + ant_slot_time))
    {
        status = gXcvrLclStatusInvalidDuration;
    }

    temp = XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL(6) /* RSM trigger */ |
        XCVR_MISC_LCL_RX_CFG1_RX_HI_PER(t_capture / spint_us /* HI_PER: dma mask enable duration */) | 
        XCVR_MISC_LCL_RX_CFG1_RX_LO_PER((ant_slot_time + (uint32_t)rsm_settings_ptr->t_sw - t_capture) / spint_us); /* LO_PER */

    /* TX and RX configs have same register mapping. SPINT is different due to sampling rate */
    XCVR_MISC->LCL_RX_CFG1 = temp | XCVR_MISC_LCL_RX_CFG1_RX_SPINT(RX_SAMPLING_RATE*sampling_rate_factor*spint_us-1U);
    XCVR_MISC->LCL_TX_CFG1 = temp | XCVR_MISC_LCL_TX_CFG1_TX_SPINT(TX_SAMPLING_RATE*sampling_rate_factor*spint_us-1U);

    /* Configure TX+RX mode, and duration. All other fields are reset value (0) */
    XCVR_MISC->LCL_CFG0 = XCVR_MISC_LCL_CFG0_CTE_DUR(rsm_settings_ptr->num_ant_path) | XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK; /* account for T_PM ext */
    
    /* workaround for KFOURWONE-702 */
    offset = (T_CAPTURE_DELAY + ant_slot_time + (uint32_t)rsm_settings_ptr->t_sw - t_capture ) / (2U * spint_us);
    temp = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(offset); /* LO_PER/2 to get dma_mask centered inside T_PM */
    offset = (T_CAPTURE_DELAY + ant_slot_time + rsm_settings_ptr->t_sw - t_capture) % (2U * spint_us);
    temp |= XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(offset*RX_SAMPLING_RATE*sampling_rate_factor);
    XCVR_MISC->LCL_DMA_MASK_DELAY = temp;
    XCVR_MISC->LCL_DMA_MASK_PERIOD = XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER(t_capture / spint_us); /* == HI_PER */

    return status;
}

#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */    




#define HPM_CAL_IN_RX 0
xcvrLclStatus_t XCVR_LCL_CalibratePll(const channel_num_t *hadm_chan_idx_list,
                                      xcvr_lcl_pll_cal_data_t *cal_results,
                                      uint16_t num_freqs,
                                      bool update_curve_fit,
                                      XCVR_RSM_SQTE_RATE_T rate)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if ((hadm_chan_idx_list == NULLPTR) || (cal_results == NULLPTR)  || (num_freqs == 0U))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        uint8_t hop_tbl_cfg_restore;
        hop_tbl_cfg_restore = (uint8_t)(((XCVR_PLL_DIG->CHAN_MAP & XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_MASK) >>
                                         XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_SHIFT)); /* used for later restore  */
        XCVR_LCL_RsmPLLInit(rate); /* Apply RSM PLL customizations */
        if (rate == XCVR_RSM_RATE_2MBPS)
        {
            /* Force alternate data rate selection */
            RADIO_CTRL->RF_CTRL |=
                RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_EN_MASK | RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_MASK;
        }
        /* Loop through the list of frequencies and force warmup, grab the calibration data, and warmdown */
        uint16_t i;
        const channel_num_t *list_ptr        = hadm_chan_idx_list;
        xcvr_lcl_pll_cal_data_t *results_ptr = cal_results;
        uint32_t temp_hpm_ctrl               = XCVR_PLL_DIG->HPM_CTRL;
        uint32_t temp_hpmcal_ctrl            = XCVR_PLL_DIG->HPMCAL_CTRL;
        uint32_t temp_count;
        uint32_t hpm_cal_time = 250U; /* init to an error value */
#if (HPM_CAL_IN_RX)
        uint32_t tsm_ovrd3_restore    = XCVR_TSM->OVRD3;
        uint32_t tsm_timing29_restore = XCVR_TSM->TIMING29;
        uint32_t tsm_timing39_restore = XCVR_TSM->TIMING39;
        uint32_t tsm_timing40_restore = XCVR_TSM->TIMING40;
        uint32_t tsm_timing12_restore = XCVR_TSM->TIMING12;
        uint32_t tsm_timing33_restore = XCVR_TSM->TIMING33;
        uint32_t tsm_timing41_restore = XCVR_TSM->TIMING41;
        uint32_t tsm_timing47_restore = XCVR_TSM->TIMING47;
        uint32_t tsm_timing48_restore = XCVR_TSM->TIMING48;
        uint32_t tsm_timing49_restore = XCVR_TSM->TIMING49;
        uint32_t tsm_timing50_restore = XCVR_TSM->TIMING50;
        uint32_t tsm_timing32_restore = XCVR_TSM->TIMING32;
        uint32_t tsm_timing51_restore = XCVR_TSM->TIMING51;
        uint32_t tsm_timing21_restore = XCVR_TSM->TIMING21;
#else
        uint32_t tsm_timing12_restore = XCVR_TSM->TIMING12;
#endif /* HPM_CAL_IN_RX */
        channel_num_t chan_2442_num;
        if (update_curve_fit)
        {
            status = XCVR_LCL_MakeChanNumFromHadmIndex(
                40U, &chan_2442_num); /* Convert to the frequency list format for comparison later */
            /* Setup for calculation of effective frequency of calibration */
            i = (uint8_t)((XCVR_PLL_DIG->HPM_CTRL & XCVR_PLL_DIG_HPM_CTRL_HPM_CAL_TIME_MASK) >>
                          XCVR_PLL_DIG_HPM_CTRL_HPM_CAL_TIME_SHIFT);
            switch (i)
            {
                case 0:
                    hpm_cal_time = 25U;
                    break;
                case 1:
                    hpm_cal_time = 50U;
                    break;
                case 2:
                    hpm_cal_time = 100U;
                    break;
                default:
                    hpm_cal_time = 250U; /* Error value */
                    break;
            }
        }
        /* Set PLL to use HOP_TBL_CFG_OVRD for manual frequency control; OVRD_EN is asserted during manual calibration,
         * RSM asserts this signal automatically during normal RSM usage */
        XCVR_PLL_DIG->CHAN_MAP &= ~(XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_MASK);
        XCVR_PLL_DIG->CHAN_MAP |=
            XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_EN_MASK | XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD(2U);
        /* During calibration, HPM_CAL must always happen. CTUNE should be enabled by default so is not handled here */
        XCVR_PLL_DIG->HPM_CTRL &=
            ~(XCVR_PLL_DIG_HPM_CTRL_RX_HPM_CAL_EN_MASK 
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* Only applies to KW47 */
              | XCVR_PLL_DIG_HPM_CTRL_HPM_DYNAMIC_SEL_MASK
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */
                ); /* HPM cal is performed during warmup (not driven from RSM) */
        XCVR_PLL_DIG->HPM_CTRL |=
            XCVR_PLL_DIG_HPM_CTRL_RX_HPM_CAL_EN_MASK; /* HPM cal perfomed during RX (instead of just during TX) */
#if (HPM_CAL_IN_RX)
        /* Delay PLL loop closure until HPM CAL has time to complete */
        XCVR_TSM->TIMING29 &=
            ~(XCVR_TSM_TIMING29_SEQ_BG_PUP_IBG_TX_RX_HI_MASK | XCVR_TSM_TIMING29_SEQ_BG_PUP_IBG_TX_RX_LO_MASK);
        XCVR_TSM->TIMING39 &= ~(XCVR_TSM_TIMING39_SEQ_XO_DIST_EN_CLK_ADCDAC_RX_HI_MASK);
        XCVR_TSM->TIMING40 &= ~(XCVR_TSM_TIMING40_SEQ_DAC_PUP_RX_HI_MASK | XCVR_TSM_TIMING40_SEQ_DAC_PUP_RX_LO_MASK);
        XCVR_TSM->TIMING12 &= ~(XCVR_TSM_TIMING12_SIGMA_DELTA_EN_RX_HI_MASK);
        XCVR_TSM->TIMING33 &=
            ~(XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_RX_HI_MASK | XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_RX_LO_MASK);
        XCVR_TSM->TIMING41 &= ~(XCVR_TSM_TIMING41_SEQ_VCO_EN_HPM_RX_HI_MASK);
        XCVR_TSM->TIMING47 &= ~(XCVR_TSM_TIMING47_SEQ_DIVN_PUP_RX_HI_MASK);
        XCVR_TSM->TIMING48 &= ~(XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_MASK);
        XCVR_TSM->TIMING49 &= ~(XCVR_TSM_TIMING49_SEQ_PD_EN_PD_DRV_RX_HI_MASK);
        XCVR_TSM->TIMING29 |=
            XCVR_TSM_TIMING29_SEQ_BG_PUP_IBG_TX_RX_HI(0U) | XCVR_TSM_TIMING29_SEQ_BG_PUP_IBG_TX_RX_LO(89U);
        XCVR_TSM->TIMING39 |= XCVR_TSM_TIMING39_SEQ_XO_DIST_EN_CLK_ADCDAC_RX_HI(17U);
        XCVR_TSM->TIMING40 |= XCVR_TSM_TIMING40_SEQ_DAC_PUP_RX_HI(0U) | XCVR_TSM_TIMING40_SEQ_DAC_PUP_RX_LO(89U);
        XCVR_TSM->TIMING12 |= XCVR_TSM_TIMING12_SIGMA_DELTA_EN_RX_HI(87U);
        XCVR_TSM->TIMING33 |=
            XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_RX_HI(0x11U) | XCVR_TSM_TIMING33_SEQ_PD_EN_FCAL_BIAS_RX_LO(89U);
        XCVR_TSM->TIMING41 |= XCVR_TSM_TIMING41_SEQ_VCO_EN_HPM_RX_HI(0x11U);
        XCVR_TSM->TIMING47 |= XCVR_TSM_TIMING47_SEQ_DIVN_PUP_RX_HI(87U);
        XCVR_TSM->TIMING48 |= XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI(89U);
        XCVR_TSM->TIMING49 |= XCVR_TSM_TIMING49_SEQ_PD_EN_PD_DRV_RX_HI(89U);
        /* New overrides to resolve RX to TX offset */
        XCVR_TSM->TIMING50 &= ~(XCVR_TSM_TIMING50_SEQ_CBPF_EN_DCOC_RX_HI_MASK);
        XCVR_TSM->TIMING50 |= XCVR_TSM_TIMING50_SEQ_CBPF_EN_DCOC_RX_HI(17U);
        XCVR_TSM->TIMING32 &=
            ~(XCVR_TSM_TIMING32_SEQ_RCCAL_PUP_RX_HI_MASK | XCVR_TSM_TIMING32_SEQ_RCCAL_PUP_RX_LO_MASK);
        XCVR_TSM->TIMING32 |=
            XCVR_TSM_TIMING32_SEQ_RCCAL_PUP_RX_HI(0xFFU) | XCVR_TSM_TIMING32_SEQ_RCCAL_PUP_RX_LO(0xFFU);
        XCVR_TSM->TIMING51 &= ~(XCVR_TSM_TIMING51_SEQ_RX_GANG_PUP_RX_HI_MASK);
        XCVR_TSM->TIMING51 |= XCVR_TSM_TIMING51_SEQ_RX_GANG_PUP_RX_HI(17U);
        XCVR_TSM->TIMING21 &= ~(XCVR_TSM_TIMING21_SEQ_LDO_CAL_PUP_RX_LO_MASK);
        XCVR_TSM->TIMING21 |= XCVR_TSM_TIMING21_SEQ_LDO_CAL_PUP_RX_LO(17U);

#else
        XCVR_TSM->TIMING12 &= ~(XCVR_TSM_TIMING12_SIGMA_DELTA_EN_TX_HI_MASK);
        XCVR_TSM->TIMING12 |= XCVR_TSM_TIMING12_SIGMA_DELTA_EN_TX_HI(87U);
        XCVR_TSM->OVRD0 |= XCVR_TSM_OVRD0_TX_DIG_EN_OVRD_EN_MASK;
        XCVR_TSM->OVRD0 &=
            ~(XCVR_TSM_OVRD0_TX_DIG_EN_OVRD_MASK); /* Forces TX_DIG_EN to zero  so nothing will be transmitted */
#endif /* HPM_CAL_IN_RX */
        for (i = 0; i < num_freqs; i++)
        {
            /* Setup the desired frequency */
            XCVR_PLL_DIG->CHAN_MAP &= ~(XCVR_PLL_DIG_CHAN_MAP_CHANNEL_NUM_OVRD_MASK);
            XCVR_PLL_DIG->CHAN_MAP |= XCVR_PLL_DIG_CHAN_MAP_CHANNEL_NUM_OVRD(*list_ptr);
            /* Warmup RX & wait for completion - takes ~100usec */
#if (HPM_CAL_IN_RX)
            XCVR_ForceRxWu();
            XCVR_WaitRxWu();
#if (1)
            temp_count = 300U;
#else
            temp_count = 10000U;
#endif
            while (temp_count > 0U)
            {
                temp_count--;
            }
#else
            XCVR_ForceTxWu();
            XCVR_WaitTxWu();
#endif /* HPM_CAL_IN_RX */
            /* Capture calibrated values & warmdown */
            results_ptr->hpm_cal_val =
                (uint16_t)((XCVR_PLL_DIG->HPMCAL_CTRL & XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_FACTOR_MASK) >>
                           XCVR_PLL_DIG_HPMCAL_CTRL_HPM_CAL_FACTOR_SHIFT);
#if (defined(CTUNE_MANUAL_CAL) && (CTUNE_MANUAL_CAL == 1))
            results_ptr->ctune_cal_val =
                (uint8_t)((XCVR_PLL_DIG->CTUNE_RES & XCVR_PLL_DIG_CTUNE_RES_CTUNE_SELECTED_MASK) >>
                          XCVR_PLL_DIG_CTUNE_RES_CTUNE_SELECTED_SHIFT);
#endif /* (defined(CTUNE_MANUAL_CAL) && (CTUNE_MANUAL_CAL == 1)) */
            if (update_curve_fit &&
                (*list_ptr ==
                 chan_2442_num)) /* Only perform the curve fit update when needed and when the frequency is 2442MHz. */
            {
                /* calculation of effective frequency (in MHz) */
                temp_count = ((XCVR_PLL_DIG->HPM_CAL1 & XCVR_PLL_DIG_HPM_CAL1_HPM_COUNT_1_MASK) >>
                              XCVR_PLL_DIG_HPM_CAL1_HPM_COUNT_1_SHIFT);
                temp_count += ((XCVR_PLL_DIG->HPM_CAL2 & XCVR_PLL_DIG_HPM_CAL2_HPM_COUNT_2_MASK) >>
                               XCVR_PLL_DIG_HPM_CAL2_HPM_COUNT_2_SHIFT);
                hpm_cal_2442_data.eff_cal_freq =
                    (uint16_t)(temp_count / (4U * hpm_cal_time)); /* Frequency of the calibration in MHz*/
                hpm_cal_2442_data.hpm_cal_factor_2442 = results_ptr->hpm_cal_val;
            }
#if (HPM_CAL_IN_RX)
            XCVR_ForceRxWd();
#else
            XCVR_ForceTxWd();
#endif      /* HPM_CAL_IN_RX */
            /* Wait for completion of warmdown (until TSM goes to idle) */
#if (1)
            XCVR_WaitRxTxWd();
#else
            temp_count = 10000U;
            while (temp_count > 0U)
            {
                temp_count--;
            }
#endif
            list_ptr++;
            results_ptr++;
            while (((XCVR_MISC->XCVR_STATUS & XCVR_MISC_XCVR_STATUS_TSM_COUNT_MASK) >>
                    XCVR_MISC_XCVR_STATUS_TSM_COUNT_SHIFT) != 0U)
            {
            }
        }
        XCVR_PLL_DIG->CHAN_MAP &=
            ~(XCVR_PLL_DIG_CHAN_MAP_CHANNEL_NUM_OVRD_MASK | /* Clear the channel number override, it appears to cause
                                                               problems later... */
              XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_EN_MASK |
              XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_MASK); /* Release the override for HOP_TBL_CFG */
        XCVR_PLL_DIG->CHAN_MAP |= XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD(hop_tbl_cfg_restore);

        /* Restore TSM timing or override values supporting HPM_CAL in RX operation */
#if (HPM_CAL_IN_RX)
        XCVR_TSM->OVRD3    = tsm_ovrd3_restore;
        XCVR_TSM->TIMING29 = tsm_timing29_restore;
        XCVR_TSM->TIMING39 = tsm_timing39_restore;
        XCVR_TSM->TIMING40 = tsm_timing40_restore;
        XCVR_TSM->TIMING12 = tsm_timing12_restore;
        XCVR_TSM->TIMING33 = tsm_timing33_restore;
        XCVR_TSM->TIMING41 = tsm_timing41_restore;
        XCVR_TSM->TIMING47 = tsm_timing47_restore;
        XCVR_TSM->TIMING48 = tsm_timing48_restore;
        XCVR_TSM->TIMING49 = tsm_timing49_restore;
        XCVR_TSM->TIMING50 = tsm_timing50_restore;
        XCVR_TSM->TIMING32 = tsm_timing32_restore;
        XCVR_TSM->TIMING51 = tsm_timing51_restore;
        XCVR_TSM->TIMING21 = tsm_timing21_restore;
#else
        XCVR_TSM->OVRD0 &= ~(XCVR_TSM_OVRD0_TX_DIG_EN_OVRD_EN_MASK);
        XCVR_TSM->TIMING12 = tsm_timing12_restore;
#endif /* HPM_CAL_IN_RX */
        if (rate == XCVR_RSM_RATE_2MBPS)
        {
            /* Remove alternate data rate selection */
            RADIO_CTRL->RF_CTRL &=
                ~(RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_EN_MASK | RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_MASK);
        }

        /* Restore the setting for whether HPM_CAL happens in PLL or not */
        XCVR_PLL_DIG->HPM_CTRL    = temp_hpm_ctrl;
        XCVR_PLL_DIG->HPMCAL_CTRL = temp_hpmcal_ctrl;
    }

    return status;
}

#if (0) /* Not needed due to switching to pre-calculated Fref^3 curve */
xcvrLclStatus_t XCVR_LCL_HpmCalCurveFit(const xcvr_lcl_pll_cal_data_t *cal_results,
                                        xcvr_lcl_hpm_cal_interp_t *hpm_cal_interp)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if ((cal_results == NULLPTR) || (hpm_cal_interp == NULLPTR))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        uint64_t temp_cube                  = (uint64_t)(cal_results->eff_cal_freq);
        hpm_cal_interp->hpm_cal_factor_2442 = cal_results->hpm_cal_val;
        hpm_cal_interp->eff_cal_freq        = cal_results->eff_cal_freq;
        hpm_cal_interp->Kcal_2442           = temp_cube * temp_cube * temp_cube; /* effective cal freq^3 */
        hpm_cal_interp->Kcal_2442 =
            hpm_cal_interp->Kcal_2442 *
            (uint64_t)(
                cal_results->hpm_cal_val); /* Kcal calculation is incoming HPM_CAL_FACTOR * effective cal freq^3 */
    }

    return status;
}
#endif

xcvrLclStatus_t XCVR_LCL_InterpolatePllCal(const uint16_t *hadm_chan_idx_list,
                                           xcvr_lcl_pll_cal_data_t *cal_results,
                                           const uint16_t num_freqs,
                                           const xcvr_lcl_hpm_cal_interp_t *hpm_cal_interp)
{
    /* This array contains pre-calculated Fref^3 / freq^3 for all HADM channels 0..78, stored in fixed-point Q7 format
     */
    /* with Fref = 2442 MHz (channel 40) */
    static const uint8_t hpm_interp_fact[RSM_HADM_MAX_CHAN_INDEX + 1] = {
        135U, 134U, 134U, 134U, 134U, 134U, 133U, 133U, 133U, 133U, 133U, 133U, 133U, 132U, 132U, 132U,
        132U, 132U, 132U, 131U, 131U, 131U, 131U, 131U, 131U, 130U, 130U, 130U, 130U, 130U, 130U, 129U,
        129U, 129U, 129U, 129U, 129U, 128U, 128U, 128U, 128U, 128U, 128U, 128U, 127U, 127U, 127U, 127U,
        127U, 127U, 126U, 126U, 126U, 126U, 126U, 126U, 126U, 125U, 125U, 125U, 125U, 125U, 125U, 124U,
        124U, 124U, 124U, 124U, 124U, 124U, 123U, 123U, 123U, 123U, 123U, 123U, 123U, 122U, 122U};

    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if ((hadm_chan_idx_list == NULLPTR) || (cal_results == NULLPTR) || (hpm_cal_interp == NULLPTR) || (num_freqs == 0U))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Loop through the list of frequencies and perform interpolations for each frequency */
        uint16_t hpm_cal_2442 = hpm_cal_interp->hpm_cal_factor_2442;
        uint16_t i;
        const channel_num_t *list_ptr        = hadm_chan_idx_list;
        xcvr_lcl_pll_cal_data_t *results_ptr = cal_results;
        for (i = 0; i < num_freqs; i++)
        {
            /* Perform HPM CAL interpolation */
            assert((*list_ptr < (RSM_HADM_MAX_CHAN_INDEX + 1U))); /* Ensure array bounds are respected */
            uint32_t temp_calc       = (uint32_t)(hpm_interp_fact[*list_ptr]);
            temp_calc                = (temp_calc * hpm_cal_2442) >> 7U; /* compensate for Q7 format */
            results_ptr->hpm_cal_val = (uint16_t)temp_calc;
            list_ptr++;
            results_ptr++;
        }
    }

    return status;
}

/* Storage for TSM and RF_CTRL registers to support splitting DCOC manual calibrate into two parts (optimizing CPU instead of polling) */
static uint32_t timing09_backup;
static uint32_t timing13_backup; 
static uint32_t fast_ctrl2_backup;    
static uint32_t rf_ctrl_backup;

void XCVR_LCL_CalibrateDcocStart(XCVR_RSM_SQTE_RATE_T rate)
{

    /* RX WU */
    rf_ctrl_backup = RADIO_CTRL->RF_CTRL;
    uint32_t temp_rf_ctrl = rf_ctrl_backup;
    if (rate == XCVR_RSM_RATE_2MBPS)
    {
        temp_rf_ctrl |=
            RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_MASK | RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_EN_MASK;
    }
    else
    {
        temp_rf_ctrl &=
            ~(RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_MASK | RADIO_CTRL_RF_CTRL_RIF_SEL_2MBPS_OVRD_EN_MASK);
    }
    RADIO_CTRL->RF_CTRL = temp_rf_ctrl;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
    /* Set XCVR to an out-of-band frequency to avoid possible intereference to the DCOC DAC trim process */
    xcvrStatus_t status = gXcvrSuccess_c;
    status = XCVR_OverrideRxFrequency(2385000000UL, -1000000UL); /* 1MHz IF for BLE */
    (void)status;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

    /*** Configure TSM Timings to enable DCOC ***/
    /* Needed Signals to run DCOC:
     * - DCOC_GAIN_CFG_EN (09)
     * - DCOC_CAL_EN (13)
     * - SEQ_LDO_GANG_PUP (24)
     * - SEQ_BG_PUP_IBG_RX (30)
     * - SEQ_CBPF_EN_DCOC (50)
     * - SEQ_RX_GANG_PUP (51)
     * BLE is already implementing DCOC during RX warmup. no Change to apply.
     * This is supporting 32MHz and 26MHz XTAL.
     * RSM is not using DCOC, Needed TSM timings are temporary changed (DCOC_GAIN_CFG_EN and DCOC_CAL_EN)
     * Other signals are already set during a RX WU after FAST_RX2TX_START point (PLL Lock).
     * DCOC is programmed to start after PLL LOCK time at count 55 for 42us
     */ 
     timing09_backup = XCVR_TSM->TIMING09; // DCOC_GAIN_CFG_EN 
     timing13_backup = XCVR_TSM->TIMING13; // DCOC_CAL_EN 
     uint32_t timing48 = XCVR_TSM->TIMING48; // SEQ_DIVN_CLOSEDLOOP (not changed, only reading value) 
     fast_ctrl2_backup = XCVR_TSM->FAST_CTRL2;    
     /* if DCOC is disabled */ 
     if( ((timing13_backup & XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_HI_MASK) >> XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_HI_SHIFT) == 0xFFU ) 
     {
        XCVR_TSM->TIMING09 = timing48; // DCOC_GAIN_CFG_EN = SEQ_DIVN_CLOSEDLOOP 
        uint32_t dcoc_cal_en_rx_hi = (((timing48 & XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_MASK) >> XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_SHIFT) + 12U);
        XCVR_TSM->TIMING13 = XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_HI( dcoc_cal_en_rx_hi ) | XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_LO( dcoc_cal_en_rx_hi + 42U);
        /* if FAST RX WU is enabled, update FAST_START to be after the DCOC */
        if( 0U != (XCVR_TSM->FAST_CTRL1 & XCVR_TSM_FAST_CTRL1_FAST_RX_WU_EN_MASK) )
        { 
            XCVR_TSM->FAST_CTRL2 &= ~XCVR_TSM_FAST_CTRL2_FAST_START_RX_MASK;
            XCVR_TSM->FAST_CTRL2 |= XCVR_TSM_FAST_CTRL2_FAST_START_RX(dcoc_cal_en_rx_hi + 42U);
        }
     }
     /* WU */
     XCVR_ForceRxWu();
}

xcvrLclStatus_t XCVR_LCL_CalibrateDcocComplete(void)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    /* Wait for completion of RX WU that was started in XCVR_LCL_CalibrateDcocStart() */
    XCVR_WaitRxWu();
    /* RX WD */
    XCVR_ForceRxWd();
    XCVR_WaitRxTxWd();

    /* Restore TSM state */
    XCVR_TSM->TIMING09 = timing09_backup;
    XCVR_TSM->TIMING13 = timing13_backup;
    XCVR_TSM->FAST_CTRL2 = fast_ctrl2_backup;

    /* Restore RF_CTRL state */
    RADIO_CTRL->RF_CTRL = rf_ctrl_backup;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
    /* Release channel over-rides and return PLL to Link Layer Control */
    /* Remove any PLL settings that caused out-of-band receive operations (for safety) */
    XCVR_OverrideRxFrequency(2402000000UL, -1000000UL);
    XCVR_ReleasePLLOverride();
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

    if ((XCVR_RX_DIG->DCOC_STAT == 0x00002020U) && (XCVR_RX_DIG->DCOC_DIG_CORR_RESULT == 0U))
    {
        status = gXcvrLclStatusFail;
    }
    return status;
}

void XCVR_LCL_ContPhaseOvrd(void)
{
    /* Override TSM signals to keep state of TX and RX dividers consistent (phase) */
    XCVR_TSM->OVRD1 |= XCVR_TSM_OVRD1_SEQ_LDO_VCO_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD1_SEQ_LDO_VCO_PUP_OVRD_MASK |
                       XCVR_TSM_OVRD1_SEQ_LDO_LV_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD1_SEQ_LDO_LV_PUP_OVRD_MASK |
                       XCVR_TSM_OVRD1_SEQ_BG_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD1_SEQ_BG_PUP_OVRD_MASK;

    XCVR_TSM->OVRD2 |= XCVR_TSM_OVRD2_SEQ_VCO_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD2_SEQ_VCO_PUP_OVRD_MASK |
                       XCVR_TSM_OVRD2_SEQ_LO_PUP_VLO_RXDRV_OVRD_EN_MASK | XCVR_TSM_OVRD2_SEQ_LO_PUP_VLO_RXDRV_OVRD_MASK;

    XCVR_TSM->OVRD3 |= XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TX_OVRD_EN_MASK | XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TX_OVRD_MASK;
}

void XCVR_LCL_AllPhaseRelease(void)
{
    /* Release all previous overrides, both continuous phase and measurement only */
    XCVR_TSM->OVRD1 &= ~(XCVR_TSM_OVRD1_SEQ_LDO_VCO_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD1_SEQ_LDO_VCO_PUP_OVRD_MASK |
                         XCVR_TSM_OVRD1_SEQ_LDO_LV_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD1_SEQ_LDO_LV_PUP_OVRD_MASK |
                         XCVR_TSM_OVRD1_SEQ_BG_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD1_SEQ_BG_PUP_OVRD_MASK);

    XCVR_TSM->OVRD2 &=
        ~(XCVR_TSM_OVRD2_SEQ_VCO_PUP_OVRD_EN_MASK | XCVR_TSM_OVRD2_SEQ_VCO_PUP_OVRD_MASK |
          XCVR_TSM_OVRD2_SEQ_LO_PUP_VLO_RXDRV_OVRD_EN_MASK | XCVR_TSM_OVRD2_SEQ_LO_PUP_VLO_RXDRV_OVRD_MASK);

    XCVR_TSM->OVRD3 &=
        ~(XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TXDRV_OVRD_EN_MASK | XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TXDRV_OVRD_MASK |
          XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TX_OVRD_EN_MASK | XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TX_OVRD_MASK);
}

#if !defined(GCOV_DO_COVERAGE) /* local except when testing code coverage */
static uint8_t XCVR_LCL_CalcAdcOffset(uint8_t adc_offset_s7, uint8_t dig_corr_s8)
#else
uint8_t XCVR_LCL_CalcAdcOffset(uint8_t adc_offset_s7, uint8_t dig_corr_s8)
#endif /*  !defined(GCOV_DO_COVERAGE) */
{
/* Common ADC offset calculation for use by XCVR_LCL_SetupManualDcoc() */
    uint8_t temp_adc_offset;
    int16_t temp_total_offset;
    /* Input data is 7 bit 2's complement signed value stored in uint8_t */                                                        
    /* Sign extend from 7 to 8 bits, still in uint8_t */
    temp_adc_offset = adc_offset_s7;
    temp_adc_offset |= (((temp_adc_offset & 0x40U) == 0x40U) ? 0x80U : 0x0U); 
    /* Cast the signed data (stored in unsigned 8 bits) to int8_t for combination with another int8_t (in an int16_t
     * variable) */
    temp_total_offset = (int8_t)temp_adc_offset;
    /* add 8 bit signed data */
    /* Input data for digital correction is 2's complement 8bit stored in uint8_t */
    temp_total_offset += (int8_t)dig_corr_s8;
    /* saturate to ensure the total (stored in 16 bits) fits within the allocated 7 bits (signed) */
    if (temp_total_offset > 63)
    {
        temp_total_offset = 63;
    }
    else
    {
        if (temp_total_offset < -64)
        {
            temp_total_offset = -64;
        }
    }
    /* cast the signed 16 bits back to unsigned 8 bits */
    return  (uint8_t)(temp_total_offset); 
}

uint32_t XCVR_LCL_SetupManualDcoc(void)
{
    /* Override DCOC ADC and DAC to produce higher amplitude in measurements */
    uint32_t temp_dcoc_stat = XCVR_RX_DIG->DCOC_STAT;
    uint32_t temp_resid     = XCVR_RX_DIG->DCOC_DIG_CORR_RESULT;
    uint8_t adc_offset_i, adc_offset_q;

    uint32_t temp_return_val;
    /* Capture the DCOC DAC with the value from the DCOC_STAT register (from last DCOC calibration) */
    temp_return_val =  temp_dcoc_stat & (XCVR_RX_DIG_DCOC_CTRL2_DCOC_DAC_OVRD_I_MASK | 
                                                            XCVR_RX_DIG_DCOC_CTRL2_DCOC_DAC_OVRD_Q_MASK);
    adc_offset_i = XCVR_LCL_CalcAdcOffset((uint8_t)((temp_dcoc_stat & XCVR_RX_DIG_DCOC_STAT_DCOC_ADC_OFFSET_I_MASK) >>
                                                                        XCVR_RX_DIG_DCOC_STAT_DCOC_ADC_OFFSET_I_SHIFT),
                                                               (uint8_t)((temp_resid & XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_I_MASK) >>
                                                                    XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_I_SHIFT));
    adc_offset_q = XCVR_LCL_CalcAdcOffset((uint8_t)((temp_dcoc_stat & XCVR_RX_DIG_DCOC_STAT_DCOC_ADC_OFFSET_Q_MASK) >>
                                                                        XCVR_RX_DIG_DCOC_STAT_DCOC_ADC_OFFSET_Q_SHIFT),
                                                               (uint8_t)((temp_resid & XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_Q_MASK) >>
                                                                    XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_Q_SHIFT));
    temp_return_val |= (XCVR_RX_DIG_DCOC_CTRL2_DCOC_ADC_OFFSET_OVRD_I(adc_offset_i) |
                                XCVR_RX_DIG_DCOC_CTRL2_DCOC_ADC_OFFSET_OVRD_Q(adc_offset_q));

    return temp_return_val;
}

void XCVR_LCL_OverrideDcoc(uint32_t dcoc_ctrl2_value, bool override)
{
 uint32_t temp = XCVR_RX_DIG->DCOC_CTRL0;
    if (override)
    {
        /* Set manual value for DAC and ADC for DCOC and force those to be active */
        XCVR_RX_DIG->DCOC_CTRL2 = dcoc_ctrl2_value;
        temp |=  (XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_OVRD_EN_MASK | 
                                                            XCVR_RX_DIG_DCOC_CTRL0_DCOC_ADC_OFFSET_OVRD_EN_MASK);
        temp &= ~XCVR_RX_DIG_DCOC_CTRL0_DCOC_DIG_CORR_EN_MASK;   /* Turn off digital correct since it is included in the override values already. */
    }
    else
    {
        /* Release the override */
        temp &=  ~(XCVR_RX_DIG_DCOC_CTRL0_DCOC_DAC_OVRD_EN_MASK | 
                                                            XCVR_RX_DIG_DCOC_CTRL0_DCOC_ADC_OFFSET_OVRD_EN_MASK);
        temp |= XCVR_RX_DIG_DCOC_CTRL0_DCOC_DIG_CORR_EN_MASK; /* Turn  digital correct back on since override is released. */
    }
    XCVR_RX_DIG->DCOC_CTRL0 = temp;
        
}

void XCVR_LCL_EnablePhaseMeasure(void)
{
    /* Program VLO_TXDRV_OVRD & _EN */
    XCVR_TSM->OVRD3 |= XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TXDRV_OVRD_EN_MASK |
                       XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TXDRV_OVRD_MASK | XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TX_OVRD_EN_MASK |
                       XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TX_OVRD_MASK;
     XCVR_RX_DIG->DCOC_CTRL0 |= XCVR_RX_DIG_DCOC_CTRL0_DCOC_DIG_CORR_EN_MASK;
    /* if another gain must be used for a better input signal measurement, it must be configured here ( DCOC_GAIN_CFG_EN to be forced LOW by override + AGC gain config) */
}

xcvrLclStatus_t XCVR_LCL_ProcessPhaseMeasure(int8_t *i_resid, int8_t *q_resid, uint32_t dc_resid_val)
{
    xcvrLclStatus_t status = gXcvrLclStatusInvalidArgs;
    if ((i_resid != NULLPTR) && (q_resid != NULLPTR))
    {
        /* Read DC residual for I and Q */
        uint8_t resid_i_u = (uint8_t)((dc_resid_val & XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_I_MASK) >>
                                      XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_I_SHIFT);
        uint8_t resid_q_u = (uint8_t)((dc_resid_val & XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_Q_MASK) >>
                                      XCVR_RX_DIG_DCOC_DIG_CORR_RESULT_DCOC_DIG_CORR_Q_SHIFT);
        /* cast to int8_t and store to pointed locations */
        *i_resid = (int8_t)resid_i_u;
        *q_resid = (int8_t)resid_q_u;

        /* Release VLO_TXDRV_OVRD & _EN */
        XCVR_TSM->OVRD3 &=
            ~(XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TXDRV_OVRD_EN_MASK | XCVR_TSM_OVRD3_SEQ_LO_PUP_VLO_TXDRV_OVRD_MASK);
        XCVR_RX_DIG->DCOC_CTRL0 &= ~XCVR_RX_DIG_DCOC_CTRL0_DCOC_DIG_CORR_EN_MASK;
        
        status = gXcvrLclStatusSuccess;
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmCompCfo(int32_t cfo_in_hz)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    if ((cfo_in_hz > 500000) || (cfo_in_hz < -500000)) /* Test CFO limitation to keep to ~20bits */
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Each bit in the PLL_OFFSET_CTRL register is about 0.95Hz of frequency change so convert by multiply by 67 and
         * dividing by 64 */
        int32_t itemp                 = -cfo_in_hz; /* Compensation is in the opposite direction of the CFO value */
        itemp                         = itemp * 67;
        itemp                         = itemp / 64;
        XCVR_PLL_DIG->PLL_OFFSET_CTRL = XCVR_PLL_DIG_PLL_OFFSET_CTRL_PLL_NUMERATOR_OFFSET(itemp);
    }

    return status;
}

int32_t XCVR_LCL_RsmReadCfoComp(void)
{
    uint32_t utemp = XCVR_PLL_DIG->PLL_OFFSET_CTRL;
    int32_t itemp;
    if ((utemp & (1UL << 27)) != 0U)
    {
        utemp |= ~XCVR_PLL_DIG_PLL_OFFSET_CTRL_PLL_NUMERATOR_OFFSET_MASK; /* sign extend in the uint type */
    }
    itemp = (int32_t)utemp;
    itemp = itemp * 64;
    itemp = itemp / 67;
    return -itemp; /* CFO value is the negative of the frequency offset */
}

xcvrLclStatus_t XCVR_LCL_MakeChanNumFromHadmIndex(uint8_t hadm_chan_index, uint16_t *fstep_chan_num)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    if (hadm_chan_index > RSM_HADM_MAX_CHAN_INDEX) /* Test so we don't access out of array bounds */
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
#if (0)
        uint16_t mapped_chan_num =
            (uint16_t)(hadm_chan_index) >>
            1U; /* divide by 2 to get the normal BLE channel index for format #2 HOP_TBL_CFG_OVRD */
        if ((hadm_chan_index & 0x1U) == 0x1U) /* original HADM channel was an odd number */
        {
            mapped_chan_num++; /* go to next channel up (2MHz higher) to allow -1MHz to hit the target channel */
            mapped_chan_num |= OFFSET_NEG_1MHZ << 7U; /* Apply -1MHz */
        }
        *fstep_chan_num = mapped_chan_num;
#else
        MAKE_MAPPED_CHAN_OVRD2(hadm_chan_index, *fstep_chan_num);
#endif
    }

    return status;
}

uint8_t XCVR_LCL_CountPnRttSteps(const xcvr_lcl_fstep_t *fstep_settings, uint16_t num_steps)
{
    uint8_t count = 0xFFU; /* Error indication */

    if ((NULLPTR != fstep_settings) && (0U < num_steps) && (129U > num_steps))
    {
        uint16_t i;
        const xcvr_lcl_fstep_t *step_ptr = fstep_settings; /* point to the array of Fstep structures */
        count                            = 0;
        for (i = 0U; i < num_steps; i++)
        {
            uint8_t temp = (step_ptr->tpm_step_format_hmp_cal_factor_msb & XCVR_RSM_STEP_FORMAT_MASK) >>
                           XCVR_RSM_STEP_FORMAT_SHIFT;
            if (temp != (uint8_t)XCVR_RSM_STEP_TN_TN) /* Count all steps other than Tn-Tn */
            {
                count++;
            }
            step_ptr++;
        }
    }

    return count;
}

static xcvrLclStatus_t XCVR_LCL_CheckCaptureBufferParams(const xcvr_lcl_fstep_t *fstep_settings,
                                                 uint8_t num_steps,
                                                 uint16_t *dma_buffer_size,
                                                 uint16_t *dma_seq_length_us,
                                                 uint8_t ant_cnt)
{
    /* Helper function for XCVR_LCL_GetRSMCaptureBufferSize() parameter checking */
    xcvrLclStatus_t status            = gXcvrLclStatusSuccess;
    /* Check that none of the pointers are NULL */
    if ((fstep_settings == NULLPTR) || (dma_buffer_size == NULLPTR) || (dma_seq_length_us == NULLPTR))
    {
        status = gXcvrLclStatusInvalidArgs;
    }

    /* Check the sequence steps and antenna counts are within limits */
    if ((num_steps > XCVR_RSM_OVERALL_MAX_SEQ_LEN) || (ant_cnt > XCVR_RSM_MAX_NUM_ANT) || num_steps == 0U)
    {
        status = gXcvrLclStatusInvalidArgs;
    }

    return status;

}

xcvrLclStatus_t XCVR_LCL_GetRSMCaptureBufferSize(const xcvr_lcl_fstep_t *fstep_settings,
                                                 uint8_t num_steps,
                                                 XCVR_RSM_RXTX_MODE_T role,
                                                 uint16_t *dma_buffer_size,
                                                 uint16_t *dma_seq_length_us,
                                                 uint8_t ant_cnt)
{
#define TONE_EXT_COUNT (1U)  /* Tone extension always present in KW45 */
    xcvrLclStatus_t status            = gXcvrLclStatusSuccess;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    /* Cast fstep_settings pointer to a uint32_t pointer to emulate the PKT RAM array */
    /* keep track of offset into this array, starting at an assumed zero offset (from the base pointer which may not be the base of PKT RAM space) */
    uint32_t * cfg_ram_buffer = (uint32_t *) fstep_settings;
    uint16_t buffer_offset = 0U; /* Trackoffset into above buffer */
#else  /* KW45 version */
    const xcvr_lcl_fstep_t *fstep_ptr = fstep_settings;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

    /* Use parameter check helper function to reduce CCM complexity */
    status = XCVR_LCL_CheckCaptureBufferParams(fstep_settings,
                                                 num_steps,
                                                 dma_buffer_size,
                                                 dma_seq_length_us,
                                                 ant_cnt);
                                                 
    if (status == gXcvrLclStatusSuccess)
    {
        /* Read configuration from registers and calculate times in usec where needed. */
        uint8_t rate =
            (uint8_t)((XCVR_MISC->RSM_CTRL0 & XCVR_MISC_RSM_CTRL0_RSM_RATE_MASK) >> XCVR_MISC_RSM_CTRL0_RSM_RATE_SHIFT);
        uint8_t rx_settling_latency = (uint8_t)((XCVR_TSM->WU_LATENCY & XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_MASK)>>XCVR_TSM_WU_LATENCY_RX_SETTLING_LATENCY_SHIFT);
        uint8_t rsm_rxlat_dig = 0;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN>=470)
        rsm_rxlat_dig = (uint8_t)((XCVR_MISC->RSM_CTRL6 & XCVR_MISC_RSM_CTRL6_RSM_RXLAT_DIG_MASK)>>XCVR_MISC_RSM_CTRL6_RSM_RXLAT_DIG_SHIFT);
#endif

        /* Select whether to use DMA duration from RSM registers or LCL antenna control */
        uint8_t dma_fm_dur  = (uint8_t)((XCVR_MISC->RSM_CTRL4 & XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0_MASK) >>
                                       XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0_SHIFT);
        uint32_t rx_dig_ctrl1 = XCVR_RX_DIG->CTRL1;
        uint16_t dma_iq_avg = (uint16_t)(((rx_dig_ctrl1 & XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_MASK) >>
                                          XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_SHIFT));
        uint8_t rx_dft_iq_out_averaged = 1; /* default dma get averaged samples with reduced sample rate ( KW45 ) */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN>=470)      
        rx_dft_iq_out_averaged = (rx_dig_ctrl1&XCVR_RX_DIG_CTRL1_RX_DFT_IQ_OUT_AVERAGED_MASK)>>XCVR_RX_DIG_CTRL1_RX_DFT_IQ_OUT_AVERAGED_SHIFT;
#endif
        uint32_t temp       = (XCVR_MISC->RSM_CTRL1);
        uint16_t t_fc_usec =
            (uint16_t)(T_FC_INCMT * ((temp & XCVR_MISC_RSM_CTRL1_RSM_T_FC_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_FC_SHIFT));
        uint16_t t_ip1_usec =
            (uint16_t)(T_IP_INCMT * ((temp & XCVR_MISC_RSM_CTRL1_RSM_T_IP1_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_IP1_SHIFT));
        uint16_t t_ip2_usec =
            (uint16_t)(T_IP_INCMT * ((temp & XCVR_MISC_RSM_CTRL1_RSM_T_IP2_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_IP2_SHIFT));
        uint16_t t_s_usec =
            (uint16_t)(T_S_INCMT * ((temp & XCVR_MISC_RSM_CTRL1_RSM_T_S_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_S_SHIFT));
        uint16_t t_fm_usec[T_FM_FLD_COUNT];
        /* Calculate values in usec from the register contents */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
        t_fm_usec[0] = (uint16_t)(
            10U * (1U + ((temp & XCVR_MISC_RSM_CTRL1_RSM_T_FM0_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_FM0_SHIFT)));
        t_fm_usec[1] = (uint16_t)(
            10U * (1U + ((temp & XCVR_MISC_RSM_CTRL1_RSM_T_FM1_MASK) >> XCVR_MISC_RSM_CTRL1_RSM_T_FM1_SHIFT)));
        t_fm_usec[2]       = (uint16_t)(0U);
        t_fm_usec[3]       = (uint16_t)(0U);
#else
        /* Calculate values in usec from the register contents */
        t_fm_usec[0] = (uint16_t)(((XCVR_MISC->RSM_CTRL5 & XCVR_MISC_RSM_CTRL5_RSM_T_FM_MASK) >> XCVR_MISC_RSM_CTRL5_RSM_T_FM_SHIFT));
#endif
        /* T_DT */
        uint16_t t_dt_usec = 0U;
        uint16_t t_dt_mode0_usec = 0U;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
        const uint16_t t_dt_adder[7] = {0U, 32U, 96U, 32U, 64U, 96U, 128U};
        uint32_t rtt_type = (XCVR_MISC->RSM_CTRL2&XCVR_MISC_RSM_CTRL2_RSM_RTT_TYPE_MASK)>>XCVR_MISC_RSM_CTRL2_RSM_RTT_TYPE_SHIFT;
        t_dt_usec = (((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                 ((16U + 32U + t_dt_adder[rtt_type] + 4U) / 2U) :
                 8U + 32U + t_dt_adder[rtt_type] + 4U); /* 8bit preamble, 32 bit PN seq, 4 bit trailer (plus any RTT_TYPE adder)  */
        t_dt_mode0_usec = (((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                 ((16U + 32U + 4U) / 2U) :
                 8U + 32U + 4U);          /* 8bit preamble, 32 bit PN seq, 4 bit trailer (no RTT_TYPE adder)  */

#else
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
        t_dt_usec = (((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                         ((16U + 64U + 4U) / 2U) :
                         8U + 64U + 4U); /* 8bit preamble, 64 bit PN seq, 4 bit trailer */
#else
        t_dt_usec          = (((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                         ((16U + 32U + 4U) / 2U) :
                         8U + 32U + 4U); /* 8bit preamble, 32 bit PN seq, 4 bit trailer */
#endif
        t_dt_mode0_usec = t_dt_usec;  /* mode0 DT = main mode DT */
#endif  /*  defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */

        /* T_PM */
        uint16_t t_pm_usec[T_PM_FLD_COUNT];
        temp = XCVR_MISC->RSM_CTRL2;
        /* Calculate values in usec from the register contents */
        t_pm_usec[0] = (uint16_t)(
            T_PM_INCMT * (T_PM_REG_OFFSET + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM0_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM0_SHIFT)));
        t_pm_usec[1] = (uint16_t)(
            T_PM_INCMT * (T_PM_REG_OFFSET + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM1_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM1_SHIFT)));
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
        t_pm_usec[2] = (uint16_t)(
            T_PM_INCMT * (T_PM_REG_OFFSET + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM2_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM2_SHIFT)));
        t_pm_usec[3] = (uint16_t)(
            T_PM_INCMT * (T_PM_REG_OFFSET + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM3_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM3_SHIFT)));
#endif
        if ((dma_iq_avg == 0U) || (rx_dft_iq_out_averaged==0))
        {
            dma_iq_avg = 1U;
        }
        else
        {
            dma_iq_avg = ((uint16_t)2U << dma_iq_avg);
        }
        uint16_t sample_rate_per_usec = ((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                                            8U :
                                            4U; /* Raw (before decimation) sample rate for RX IQs */

        uint8_t dma_pm_dur;
        bool rsm_dma_mask_used;
        temp = XCVR_MISC->RSM_CTRL3;
        uint32_t dma_mask_ctrl = 0;
        uint8_t dma_mask_center = 0; /* disabled by default for KW45 compability */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN>=470)
        dma_mask_ctrl = XCVR_MISC->DMA_MASK_CTRL;
        dma_mask_center = (dma_mask_ctrl&XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER_MASK)>>XCVR_MISC_DMA_MASK_CTRL_DMA_MASK_CENTER_SHIFT;
#else
        (void)dma_mask_ctrl;  /* Compilation for other derivative can fail for unused variable */
#endif
        rsm_dma_mask_used = (temp & XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN_MASK) != 0U;
        if (rsm_dma_mask_used)
        {
            /* DMA_PM_DUR for RSM DMA mask is covering the entire T_PM which covers all antenna paths + TONE EXT */
            /* The RSM T_PM programming is the entire slot, no matter how many antenna or extension slots are present */
            dma_pm_dur  = (uint8_t)((temp & XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR_MASK) >>XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR_SHIFT);
        }
        else
        {
            /* DMA_PM_DUR for LCL DMA mask represents a single antenna path (compared to all antenna paths for RSM mask above) */
            uint32_t dma_intervals = ((XCVR_MISC->LCL_DMA_MASK_PERIOD & XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER_MASK) >>
                                       XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER_SHIFT);
            if (dma_intervals == 0U) /* When DMA_MASK_REF_PER is not in use it should be zero and RX_HI_PER should be used */
            {
                dma_intervals = ((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_MASK) >>
                                  XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_SHIFT);
            }
            
            uint32_t rx_spint = 1U + (( XCVR_MISC->LCL_RX_CFG1& XCVR_MISC_LCL_RX_CFG1_RX_SPINT_MASK)>>XCVR_MISC_LCL_RX_CFG1_RX_SPINT_SHIFT);   
            dma_pm_dur = (uint8_t)((dma_intervals * rx_spint) / sample_rate_per_usec);
            assert(((dma_intervals * rx_spint) % sample_rate_per_usec) == 0U); /* There shouldn't be any remainder... */
        
            if(dma_mask_center > 0) /* if dma_mask_center in use, the window duration is given by this field */
            {
                uint16_t center_window = (1<<(dma_mask_center-1)); /* us. dma_mask_center>0 only */
                if( center_window < dma_pm_dur ) /* pm duration used is the min between centering size and lcl_high_per */
                {
                    dma_pm_dur = center_window;
                }
            }
        }

        /* Read TSM WU WD configuration [CONNRF-1076] */
        uint32_t tsm_end_of_seq = XCVR_TSM->END_OF_SEQ;
        uint8_t end_of_rx_wd    = (uint8_t)((tsm_end_of_seq & XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_MASK) >>
                                         XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_SHIFT);
        uint8_t end_of_rx_wu    = (uint8_t)((tsm_end_of_seq & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >>
                                         XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT);
        uint8_t end_of_tx_wd    = (uint8_t)((tsm_end_of_seq & XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_MASK) >>
                                         XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_SHIFT);
        uint8_t end_of_tx_wu    = (uint8_t)((tsm_end_of_seq & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >>
                                         XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT);
        uint8_t ramp_up_dly =
            (uint8_t)((XCVR_TX_DIG->DATA_PADDING_CTRL_1 & XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY_MASK) >>
                      XCVR_TX_DIG_DATA_PADDING_CTRL_1_RAMP_UP_DLY_SHIFT);
        uint8_t tx_dig_en_tx_hi = (uint8_t)((XCVR_TSM->TIMING14 & XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI_MASK) >>
                                            XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI_SHIFT);
        uint8_t tx_data_flush_dly =
            (uint8_t)((XCVR_TX_DIG->DATA_PADDING_CTRL_1 & XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_MASK) >>
                      XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY_SHIFT);
        uint32_t fast_ctrl2 = XCVR_TSM->FAST_CTRL2;

        uint16_t warmup_us;
        uint16_t warmdown_us;
        if (role == XCVR_RSM_TX_MODE)
        {
            uint8_t fast_dest_tx  = (uint8_t)((fast_ctrl2 & XCVR_TSM_FAST_CTRL2_FAST_DEST_TX_MASK) >>
                                             XCVR_TSM_FAST_CTRL2_FAST_DEST_TX_SHIFT);
            uint8_t fast_start_tx = (uint8_t)((fast_ctrl2 & XCVR_TSM_FAST_CTRL2_FAST_START_TX_MASK) >>
                                              XCVR_TSM_FAST_CTRL2_FAST_START_TX_SHIFT);
            uint8_t fast_fc_tx_wu = (uint8_t)((XCVR_MISC->RSM_CTRL0 & XCVR_MISC_RSM_CTRL0_RSM_FAST_FC_TX_WU_MASK) >>
                                              XCVR_MISC_RSM_CTRL0_RSM_FAST_FC_TX_WU_SHIFT);
            uint8_t pa_ru         = ((ramp_up_dly + 2U) >= (end_of_tx_wu - tx_dig_en_tx_hi)) ?
                                (ramp_up_dly + 2U) - (end_of_tx_wu - tx_dig_en_tx_hi) :
                                0U;

            warmup_us   = (uint16_t)((uint16_t)end_of_tx_wu -
                                   ((uint16_t)fast_dest_tx - (uint16_t)fast_start_tx) * (uint16_t)fast_fc_tx_wu +
                                   (uint16_t)pa_ru + 3U);
            warmdown_us = (uint16_t)(((uint16_t)end_of_rx_wd - (uint16_t)end_of_rx_wu) + 2U);
        }
        else /* XCVR_RSM_RX_MODE */
        {
            uint8_t fast_dest_rx  = (uint8_t)((fast_ctrl2 & XCVR_TSM_FAST_CTRL2_FAST_DEST_RX_MASK) >>
                                             XCVR_TSM_FAST_CTRL2_FAST_DEST_RX_SHIFT);
            uint8_t fast_start_rx = (uint8_t)((fast_ctrl2 & XCVR_TSM_FAST_CTRL2_FAST_START_RX_MASK) >>
                                              XCVR_TSM_FAST_CTRL2_FAST_START_RX_SHIFT);
            uint8_t fast_fc_rx_wu = (uint8_t)((XCVR_MISC->RSM_CTRL0 & XCVR_MISC_RSM_CTRL0_RSM_FAST_FC_RX_WU_MASK) >>
                                              XCVR_MISC_RSM_CTRL0_RSM_FAST_FC_RX_WU_SHIFT);

            warmup_us = (uint16_t)((uint16_t)end_of_rx_wu -
                                   ((uint16_t)fast_dest_rx - (uint16_t)fast_start_rx) * (uint16_t)fast_fc_rx_wu + 1U);
            warmdown_us =
                (uint16_t)((uint16_t)tx_data_flush_dly + 2U + ((uint16_t)end_of_tx_wd - (uint16_t)end_of_tx_wu) + 1U);
        }

        /* Read DMA Mask configuration */
        uint8_t dma_signal_valid_mask_sel = RSM_DMA_SIGNAL_VALID_MASK_SEL_DMA_MASK; // enable DMA/LCL mask for KW45 and by default
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
        dma_signal_valid_mask_sel = (XCVR_MISC->DMA_MASK_CTRL & XCVR_MISC_DMA_MASK_CTRL_DMA_SIGNAL_VALID_MASK_SEL_MASK) >> XCVR_MISC_DMA_MASK_CTRL_DMA_SIGNAL_VALID_MASK_SEL_SHIFT;
#endif

        /* Read Step configuration from RAM */
        uint16_t dma_samples = 0U;
        uint16_t temp_samples;
        uint16_t sequence_length_us    = 0U;
        static uint16_t step_length_us = 0U;
        uint8_t config_size;
        for (uint8_t i = 0; i < num_steps; i++)
        {
            XCVR_RSM_T_PM_FM_SEL_T t_pm_sel;
            uint8_t step_format ;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
            uint8_t dummy;
            uint8_t tone_extension;
            config_size = XCVR_LCL_ReadStepCfg((COM_MODE_013_CFG_HDR_Type *)&cfg_ram_buffer[buffer_offset],
                                               (XCVR_RSM_FSTEP_TYPE_T *)&step_format,
                                               (XCVR_RSM_T_PM_FM_SEL_T *)&t_pm_sel,
                                               &dummy, /* Throwaway this result */
                                               &tone_extension,
                                               &dummy, /* Throwaway this result */
                                               &dummy); /* Throwaway this result */
#else  /* KW45 version */
            temp                = fstep_ptr->tpm_step_format_hmp_cal_factor_msb;
            step_format = (uint8_t)((temp & XCVR_RSM_STEP_FORMAT_MASK) >> XCVR_RSM_STEP_FORMAT_SHIFT);
            t_pm_sel    = (XCVR_RSM_T_PM_FM_SEL_T)((temp & XCVR_RSM_T_PM_FM_SEL_MASK) >> XCVR_RSM_T_PM_FM_SEL_SHIFT);
            (void)config_size; /* Touch unused variable */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */
            step_length_us      = 0U;
            switch ((XCVR_RSM_FSTEP_TYPE_T)step_format)
            {
                case XCVR_RSM_STEP_FCS:
                    if( dma_signal_valid_mask_sel & RSM_DMA_SIGNAL_VALID_MASK_SEL_DT_RX) { /* dt_rx state mask capture */
                        dma_samples += ((t_dt_mode0_usec+rx_settling_latency+rsm_rxlat_dig+1) * sample_rate_per_usec ); /* no averaging */
                    }
                    if (role == XCVR_RSM_TX_MODE)
                    {   /* Only Initiator captures DMA samples for the frequency compensation */
                        if(dma_signal_valid_mask_sel & (RSM_DMA_SIGNAL_VALID_MASK_SEL_DMA_MASK|RSM_DMA_SIGNAL_VALID_MASK_SEL_FM_RX))
                        {
                            dma_samples += (dma_fm_dur * sample_rate_per_usec / dma_iq_avg);
                            if(dma_signal_valid_mask_sel & RSM_DMA_SIGNAL_VALID_MASK_SEL_FM_RX) { /* fm_rx state mask capture */
                              /* add additionnal samples with no averaging */
                              dma_samples += ((t_fm_usec[t_pm_sel] - dma_fm_dur + rx_settling_latency) * sample_rate_per_usec);
                            }  
                        }
                    }
                    /* Seq Len = T_FC+2*T_DT+T_IP1+T_S+T_FM */
                    step_length_us =
                        t_fc_usec + (2U * t_dt_mode0_usec) + t_ip1_usec + t_s_usec + t_fm_usec[t_pm_sel];
                    break;
                case XCVR_RSM_STEP_PK_PK:
                    if( dma_signal_valid_mask_sel & RSM_DMA_SIGNAL_VALID_MASK_SEL_DT_RX) { /* dt_rx state mask capture. no averaging */
                        dma_samples += ((t_dt_usec+rx_settling_latency+rsm_rxlat_dig+1) * sample_rate_per_usec);
                    }
                    /* Seq Len = T_FC+2*T_DT+T_IP1 */
                    step_length_us = t_fc_usec + (2U * t_dt_usec) + t_ip1_usec;
                    break;
                case XCVR_RSM_STEP_TN_TN:
                    if(!rsm_dma_mask_used)
                    {
                        /* RSM DMA mask not used == LCL block used; Must consider multi-ant */
                        temp_samples = dma_pm_dur * ((uint16_t)ant_cnt+(uint16_t)TONE_EXT_COUNT); /* DMA capture is repeated for each antenna */
                    }
                    else { /* Use RSM DMA Mask */
                        temp_samples = dma_pm_dur;
                    }
                    if(dma_signal_valid_mask_sel & (RSM_DMA_SIGNAL_VALID_MASK_SEL_DMA_MASK|RSM_DMA_SIGNAL_VALID_MASK_SEL_PM_RX))
                    {
                        dma_samples += (temp_samples * sample_rate_per_usec) / dma_iq_avg;
                        if( dma_signal_valid_mask_sel & RSM_DMA_SIGNAL_VALID_MASK_SEL_PM_RX)
                        {
                            /* pm_rx state mask capture : add additionnal samples with no averaging */
                            dma_samples += ((t_pm_usec[t_pm_sel] - temp_samples +rx_settling_latency) * sample_rate_per_usec);
                            dma_samples += (rx_dft_iq_out_averaged)? -1 : 1; /* compensate for additialnal sample */
                        }
                    }
                    /* Seq Len = T_FC+2*T_PM*NUM_ANT+T_IP2 */
                    step_length_us = t_fc_usec + (2U * t_pm_usec[t_pm_sel]) + t_ip2_usec;
                    break;
                case XCVR_RSM_STEP_PK_TN_TN_PK:
                    if( dma_signal_valid_mask_sel & RSM_DMA_SIGNAL_VALID_MASK_SEL_DT_RX) { /* dt_rx state mask capture . no averaging */
                        dma_samples += ((t_dt_usec+rx_settling_latency+rsm_rxlat_dig+1) * sample_rate_per_usec);
                    }
                    
                    if(!rsm_dma_mask_used)
                    {
                        /* RSM DMA mask not used == LCL block used; Must consider multi-ant */
                        temp_samples = dma_pm_dur * ((uint16_t)ant_cnt+(uint16_t)TONE_EXT_COUNT); /* DMA capture is repeated for each antenna */
                    }
                    else { /* Use RSM DMA Mask */
                        temp_samples = dma_pm_dur;
                    }
                    if(dma_signal_valid_mask_sel & (RSM_DMA_SIGNAL_VALID_MASK_SEL_DMA_MASK|RSM_DMA_SIGNAL_VALID_MASK_SEL_PM_RX))
                    {
                        dma_samples += (temp_samples * sample_rate_per_usec) / dma_iq_avg;
                        
                        if( dma_signal_valid_mask_sel & RSM_DMA_SIGNAL_VALID_MASK_SEL_PM_RX)
                        {
                            /* pm_rx state mask capture : add additionnal samples with no averaging */
                            dma_samples += ((t_pm_usec[t_pm_sel] - temp_samples +rx_settling_latency ) * sample_rate_per_usec);
                            dma_samples += (3*sample_rate_per_usec); /* compensate for additialnal sample */
                        }
                    } 
                    /* Seq Len = T_FC+2*T_DT+2*T_S+2*T_PM*NUM_ANT+T_IP2 */
                    step_length_us = t_fc_usec + (2U * t_dt_usec) + (2U * t_pm_usec[t_pm_sel] ) +
                                     (2U * t_s_usec) +  t_ip2_usec;
                    break;
                default:
                    /* Error case */
                    step_length_us = 0U;
                    break;
            }
            sequence_length_us += step_length_us;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
            buffer_offset += config_size;
#else  /* KW45 version */
            fstep_ptr++;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */
        }

        /* replace first T_FC by WU duration */
        sequence_length_us += warmup_us - t_fc_usec;
        /* add warmdown duration */
        sequence_length_us += warmdown_us;

        *dma_buffer_size   = dma_samples;
        *dma_seq_length_us = sequence_length_us;
    }

    return status;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
// TODO: change mode param from uint8_t to enum type 
xcvrLclStatus_t XCVR_LCL_EnaLpmClkSwitch(uint8_t mode)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    uint32_t temp_lpm_ctrl;
    uint32_t temp_lpm_sdm_ctrl1;
    temp_lpm_ctrl = XCVR_PLL_DIG->LPM_CTRL;
    temp_lpm_sdm_ctrl1 = XCVR_PLL_DIG->LPM_SDM_CTRL1;
    switch (mode)
    {
        case 0U:
            temp_lpm_ctrl &= ~(XCVR_PLL_DIG_LPM_CTRL_LPM_FAST_SW_MASK | XCVR_PLL_DIG_LPM_CTRL_LPM_CODES_ADJ_MASK);
            break;
        case 1U:
            temp_lpm_ctrl &= ~( XCVR_PLL_DIG_LPM_CTRL_LPM_CODES_ADJ_MASK);
            temp_lpm_ctrl |= (XCVR_PLL_DIG_LPM_CTRL_LPM_FAST_SW_MASK);
            temp_lpm_sdm_ctrl1 &= ~(XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_CNT_MASK | XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_VAL_MASK);
            temp_lpm_sdm_ctrl1 |= (XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_CNT(7U) | XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_VAL(2U));
            break;
        case 2U:
            temp_lpm_ctrl &= ~(XCVR_PLL_DIG_LPM_CTRL_LPM_FAST_SW_MASK);
            temp_lpm_ctrl |= (XCVR_PLL_DIG_LPM_CTRL_LPM_CODES_ADJ_MASK);
            temp_lpm_sdm_ctrl1 &= ~(XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_CNT_MASK | XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_VAL_MASK);
            temp_lpm_sdm_ctrl1 |= (XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_CNT(6U) | XCVR_PLL_DIG_LPM_SDM_CTRL1_LPM_FCODES_VAL(1U));
            break;
        default:
            status = gXcvrLclStatusInvalidArgs;
            break;
    }

    if (status == gXcvrLclStatusSuccess)
    {
         XCVR_PLL_DIG->LPM_CTRL = temp_lpm_ctrl;
         XCVR_PLL_DIG->LPM_SDM_CTRL1 = temp_lpm_sdm_ctrl1;
    }

    return status;
}

#define TSM_DISABLED (0xFFFFFFFFU)  /* value for a TSM timing register that is disabled */
xcvrLclStatus_t XCVR_LCL_EnaPic(XCVR_RSM_PIC_MODE_TYPE_T mode, bool slow_bw_reduction)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    uint32_t temp_pic_core_en = TSM_DISABLED;
    uint32_t temp_pic_short_cint_short_en = TSM_DISABLED;
    uint32_t temp_pic_filter_low_bw_sm_en = TSM_DISABLED;
    uint32_t temp_pic_rfb_open_sm_en = TSM_DISABLED;
    uint32_t temp_pic_rint2_short_fm_en = TSM_DISABLED;
    switch (mode)
    {
        case XCVR_RSM_PIC_DISABLED: /* Disabled (legacy) doesn't require any change from DISABLED state */
            break;
        case XCVR_RSM_PIC_FAST_ONLY:
          {
            uint32_t temp_pd_en_fcal_bias;
            temp_pd_en_fcal_bias =xcvr_lcl_tsm_generic_config.TIMING33-(TSM_TX_HI(1U) | TSM_RX_HI(1U));/* Assertion is back 1 tick, deassertion is unchanged */
            temp_pd_en_fcal_bias |= (TSM_TX_LO(0xFFU) | TSM_RX_LO(0xFFU));/* Now make it never deassert, hold signals until warmdown */
            temp_pic_core_en = temp_pd_en_fcal_bias;
            temp_pic_rint2_short_fm_en = temp_pd_en_fcal_bias;
            temp_pic_rfb_open_sm_en = temp_pd_en_fcal_bias;
            uint32_t temp_analog_pll;
            temp_analog_pll = XCVR_ANALOG->PLL;
            temp_analog_pll &= ~(XCVR_ANALOG_PLL_PIC_VREF_CTRL_MASK |
                                            XCVR_ANALOG_PLL_PIC_SMOOTH_SWITCH_EN_MASK |
                                            XCVR_ANALOG_PLL_PLL_VCO_PIC_INPUT_EN_MASK |
                                            XCVR_ANALOG_PLL_PIC_RINT1_HVAL_SM_EN_MASK |
                                            XCVR_ANALOG_PLL_PIC_RINT2_VAL_SLOW_MODE_MASK |
                                            XCVR_ANALOG_PLL_PLL_PD_TRIM_FCAL_BIAS_MASK);
            temp_analog_pll |= (XCVR_ANALOG_PLL_PIC_VREF_CTRL(3U) |
                                            XCVR_ANALOG_PLL_PIC_RINT2_VAL_SLOW_MODE(2U) |
                                            XCVR_ANALOG_PLL_PIC_RINT1_HVAL_SM_EN(0U) |
                                            XCVR_ANALOG_PLL_PIC_SMOOTH_SWITCH_EN(0U) |
                                            XCVR_ANALOG_PLL_PLL_VCO_PIC_INPUT_EN_MASK |
                                            XCVR_ANALOG_PLL_PLL_PD_TRIM_FCAL_BIAS(2U));
            XCVR_ANALOG->PLL = temp_analog_pll;
            break;
          }
        case XCVR_RSM_PIC_FAST_SLOW:  /* Fast mode + Slow mode */
          {
            /* signals change first time 2 ticks before SEQ_DIVN_CLOSEDLOOP */
            uint32_t temp_seq_divn_closedloop;
            temp_seq_divn_closedloop = xcvr_lcl_tsm_generic_config.TIMING48;
            /* Go 2 ticks back for the start of the signals */
            uint8_t closedloop_tx_hi = ((temp_seq_divn_closedloop&XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_TX_HI_MASK)>>XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_TX_HI_SHIFT)-2U;
            uint8_t closedloop_rx_hi = ((temp_seq_divn_closedloop&XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_MASK)>>XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_SHIFT)-2U;
            /* signals change second time 1 tick after end of PA rampup */
            // TODO: properly calculate end of PA ramp from TX DIG settings or input parameters for TX_LO
            /* Use 1 tick before end of TX WU regardless of DATA PADDING and PA RAMP time. This may need to be modified for long padding or ramp times */
            uint8_t tx_end_tx_wu = ((xcvr_lcl_tsm_generic_config.END_OF_SEQ&XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK)>>XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT)-1U;
            uint8_t rx_dig_en = ((xcvr_lcl_tsm_generic_config.TIMING17&XCVR_TSM_TIMING17_RX_DIG_EN_RX_HI_MASK)>>XCVR_TSM_TIMING17_RX_DIG_EN_RX_HI_SHIFT)-1U; 
            temp_pic_rint2_short_fm_en = (XCVR_TSM_TIMING58_SEQ_PIC_RINT2_SHORT_FM_EN_TX_HI(closedloop_tx_hi) |
                                                        XCVR_TSM_TIMING58_SEQ_PIC_RINT2_SHORT_FM_EN_TX_LO(tx_end_tx_wu) |
                                                        XCVR_TSM_TIMING58_SEQ_PIC_RINT2_SHORT_FM_EN_RX_HI(closedloop_rx_hi) |
                                                        XCVR_TSM_TIMING58_SEQ_PIC_RINT2_SHORT_FM_EN_RX_LO(rx_dig_en));
            temp_pic_rfb_open_sm_en = (XCVR_TSM_TIMING57_SEQ_PIC_RFB_OPEN_SM_EN_TX_HI(closedloop_tx_hi) |
                                                        XCVR_TSM_TIMING57_SEQ_PIC_RFB_OPEN_SM_EN_TX_LO(tx_end_tx_wu) |
                                                        XCVR_TSM_TIMING57_SEQ_PIC_RFB_OPEN_SM_EN_RX_HI(closedloop_rx_hi) |
                                                        XCVR_TSM_TIMING57_SEQ_PIC_RFB_OPEN_SM_EN_RX_LO(rx_dig_en));
            temp_pic_core_en = xcvr_lcl_tsm_generic_config.TIMING33; /* assert the same time as PN_EN_FCAL_BIAS */
            if (slow_bw_reduction)
            {
                /* Setup temp_pic_filter_low_bw_sm_en */
                temp_pic_filter_low_bw_sm_en = (XCVR_TSM_TIMING56_SEQ_PIC_FILTER_LOW_BW_SM_EN_TX_HI(tx_end_tx_wu) |
                                                            XCVR_TSM_TIMING56_SEQ_PIC_FILTER_LOW_BW_SM_EN_TX_LO(0xFFU) |
                                                            XCVR_TSM_TIMING56_SEQ_PIC_FILTER_LOW_BW_SM_EN_RX_HI(rx_dig_en) |
                                                            XCVR_TSM_TIMING56_SEQ_PIC_FILTER_LOW_BW_SM_EN_RX_LO(0xFFU));
            }
          }
          break;
        default:
            status = gXcvrLclStatusInvalidArgs;
            break;
    }

    if (status == gXcvrLclStatusSuccess)
    {
        XCVR_TSM->TIMING54 = temp_pic_core_en;
        XCVR_TSM->TIMING55 = temp_pic_short_cint_short_en;
        XCVR_TSM->TIMING56 = temp_pic_filter_low_bw_sm_en;
        XCVR_TSM->TIMING57 = temp_pic_rfb_open_sm_en;
        XCVR_TSM->TIMING58 = temp_pic_rint2_short_fm_en;
    }
    
    return status;
}

#define DELAY_DIV_SYNC  (0U)  /* Allow shifting the location of the dividers sync TSM signals for optimization */
void XCVR_LCL_EnaDividerSync(bool enable)
{
    uint32_t temp_lodiv_sync_reset_en = TSM_DISABLED;
    uint32_t temp_lodiv_sync_en = TSM_DISABLED;
    
    if (enable)
    {
        uint8_t closedloop_tx_hi = (xcvr_lcl_tsm_generic_config.TIMING48&XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_TX_HI_MASK)>>XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_TX_HI_SHIFT;
        uint8_t closedloop_rx_hi = (xcvr_lcl_tsm_generic_config.TIMING48&XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_MASK)>>XCVR_TSM_TIMING48_SEQ_DIVN_CLOSEDLOOP_RX_HI_SHIFT;
        temp_lodiv_sync_en = ( XCVR_TSM_TIMING60_SEQ_LODIV_SYNC_EN_TX_HI(closedloop_tx_hi+2U) |
                                                     XCVR_TSM_TIMING60_SEQ_LODIV_SYNC_EN_TX_LO(closedloop_tx_hi+5U+DELAY_DIV_SYNC) |
                                                     XCVR_TSM_TIMING60_SEQ_LODIV_SYNC_EN_RX_HI(closedloop_rx_hi+2U+DELAY_DIV_SYNC) |
                                                     XCVR_TSM_TIMING60_SEQ_LODIV_SYNC_EN_RX_LO(closedloop_rx_hi+5U+DELAY_DIV_SYNC));
        temp_lodiv_sync_reset_en = (XCVR_TSM_TIMING59_SEQ_LODIV_SYNC_RESET_EN_TX_HI(closedloop_tx_hi+3U+DELAY_DIV_SYNC) |
                                                    XCVR_TSM_TIMING59_SEQ_LODIV_SYNC_RESET_EN_TX_LO(closedloop_tx_hi+4U+DELAY_DIV_SYNC) |
                                                    XCVR_TSM_TIMING59_SEQ_LODIV_SYNC_RESET_EN_RX_HI(closedloop_rx_hi+3U+DELAY_DIV_SYNC) |
                                                    XCVR_TSM_TIMING59_SEQ_LODIV_SYNC_RESET_EN_RX_LO(closedloop_rx_hi+4U+DELAY_DIV_SYNC));
    }
    XCVR_TSM->TIMING59 = temp_lodiv_sync_reset_en;
    XCVR_TSM->TIMING60 = temp_lodiv_sync_en;
    XCVR_TSM->TIMING61 = TSM_DISABLED;

}
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)   */

/* RSM interrupts are only supported on the NBU CPU */
#if defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)
static rsm_sw_handler_t rsm_handler = {NULLPTR, NULLPTR};

void XCVR_LCL_RsmRegisterCb(const rsm_sw_handler_t *user_rsm_handler)
{
    /* Update the local handler storage with the callback pointr and user data pointer */
    rsm_handler.userData      = user_rsm_handler->userData;
    rsm_handler.user_callback = user_rsm_handler->user_callback;
}

bool XCVR_LCL_RsmIrqEnDis(uint32_t mask, bool irq_enabled)
{
    bool retval = true;
    /* Determine the register address and mask for the bit to be set or cleared */
    if (0U != mask & ~XCVR_LCL_RSM_IRQ_EN_ALL_BITS) /* make sure mask doesn't set any other bits */
    {
        retval = false;
    }
    else /* no failure in the error check above */
    {
        if (irq_enabled)
        {
            XCVR_MISC->RSM_CSR = (XCVR_MISC->RSM_CSR & ~XCVR_LCL_RSM_IRQ_STAT_ALL_BITS) |
                                 mask; /* set the bits using the mask without clearing any W1C status bits */
        }
        else
        {
            XCVR_MISC->RSM_CSR = (XCVR_MISC->RSM_CSR & ~XCVR_LCL_RSM_IRQ_STAT_ALL_BITS) &
                                 ~(mask); /* clear the bits using the mask without clearing any W1C status bits */
        }
    }

    return retval;
}

void RSM_INT_IRQHandler(void)
{
    bool abort_flag = ((XCVR_MISC->RSM_CSR & XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_MASK) != 0U);

    /* Make sure the function pointer isn't NULLPTR before calling user callback */
    if (rsm_handler.user_callback != NULLPTR)
    {
        rsm_handler.user_callback(rsm_handler.userData, abort_flag, XCVR_MISC->RSM_CSR);
    }
    /* Clear all of the interrupt bits */
    XCVR_MISC->RSM_CSR |= XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_MASK |
                          XCVR_MISC_RSM_CSR_RSM_IRQ_FC_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_MASK |
                          XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_MASK;
}

#endif /* defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)) */

#endif /* #if defined(RADIO_IS_GEN_4P5) */
