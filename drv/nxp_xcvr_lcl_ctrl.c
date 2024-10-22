/*
 * Copyright 2020-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "nxp_xcvr_lcl_ctrl.h"

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

#define T_PM0_MAX (630U) /*!< Max value for T_PM0 for HADM usage.  */

#define IQ_SAMPLES_PER_USEC_1MBPS                                                                                     \
    4U /*!< Number of IQ samples per usec in 1Mbps data rate, used for checking averaging window for integer multiple \
          of samples length */
#define IQ_SAMPLES_PER_USEC_2MBPS                                                                                     \
    8U /*!< Number of IQ samples per usec in 2Mbps data rate, used for checking averaging window for integer multiple \
          of samples length */
#define F_2442_CUBED 14562534888L; /* 2442^3 Used in HPM_CAL interpolation curve */
#if defined(KW45_A0_SUPPORT) && (KW45_A0_SUPPORT > 0)
#define RSM_WA_KFOURWONE_1164 1U /* Enable workaround for RSM start failure. Fixed starting from A1 version */
#endif
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* HOP_TBL_CFG setting #3 allows most flexible frequency setting.     */
#define HOP_TBL_CFG_OVRD 2U
#if defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 3U)
const uint16_t channel_num_from_hadm_chan[RSM_HADM_MAX_CHAN_INDEX +
                                          1] = /* This table assumes that HOP_TBL_CFG setting #3 is used */
    {
        0x8800, /* 2402MHz */
        0x8C00, /* 2403MHz */
        0x9000, /* 2404MHz */
        0x9400, /* 2405MHz */
        0x9800, /* 2406MHz */
        0x9C00, /* 2407MHz */
        0xA000, /* 2408MHz */
        0xA400, /* 2409MHz */
        0xA800, /* 2410MHz */
        0xAC00, /* 2411MHz */
        0xB000, /* 2412MHz */
        0xB400, /* 2413MHz */
        0xB800, /* 2414MHz */
        0xBC00, /* 2415MHz */
        0xC000, /* 2416MHz */
        0xC400, /* 2417MHz */
        0xC800, /* 2418MHz */
        0xCC00, /* 2419MHz */
        0xD000, /* 2420MHz */
        0xD400, /* 2421MHz */
        0xD800, /* 2422MHz */
        0xDC00, /* 2423MHz */
        0xE000, /* 2424MHz */
        0xE400, /* 2425MHz */
        0xE800, /* 2426MHz */
        0xEC00, /* 2427MHz */
        0xF000, /* 2428MHz */
        0xF400, /* 2429MHz */
        0xF800, /* 2430MHz */
        0xFC00, /* 2431MHz */
        0x0,    /* 2432MHz */
        0x400,  /* 2433MHz */
        0x800,  /* 2434MHz */
        0xC00,  /* 2435MHz */
        0x1000, /* 2436MHz */
        0x1400, /* 2437MHz */
        0x1800, /* 2438MHz */
        0x1C00, /* 2439MHz */
        0x2000, /* 2440MHz */
        0x2400, /* 2441MHz */
        0x2800, /* 2442MHz */
        0x2C00, /* 2443MHz */
        0x3000, /* 2444MHz */
        0x3400, /* 2445MHz */
        0x3800, /* 2446MHz */
        0x3C00, /* 2447MHz */
        0x4000, /* 2448MHz */
        0x4400, /* 2449MHz */
        0x4800, /* 2450MHz */
        0x4C00, /* 2451MHz */
        0x5000, /* 2452MHz */
        0x5400, /* 2453MHz */
        0x5800, /* 2454MHz */
        0x5C00, /* 2455MHz */
        0x6000, /* 2456MHz */
        0x6400, /* 2457MHz */
        0x6800, /* 2458MHz */
        0x6C00, /* 2459MHz */
        0x7000, /* 2460MHz */
        0x7400, /* 2461MHz */
        0x7800, /* 2462MHz */
        0x7C00, /* 2463MHz */
        0x8001, /* 2464MHz */
        0x8401, /* 2465MHz */
        0x8801, /* 2466MHz */
        0x8C01, /* 2467MHz */
        0x9001, /* 2468MHz */
        0x9401, /* 2469MHz */
        0x9801, /* 2470MHz */
        0x9C01, /* 2471MHz */
        0xA001, /* 2472MHz */
        0xA401, /* 2473MHz */
        0xA801, /* 2474MHz */
        0xAC01, /* 2475MHz */
        0xB001, /* 2476MHz */
        0xB401, /* 2477MHz */
        0xB801, /* 2478MHz */
        0xBC01, /* 2479MHz */
        0xC001, /* 2480MHz */
};
#endif /* defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 3U) */
#if defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 2U)
#define OFFSET_NEG_1MHZ 0x100U /* HOP_TBL_CFG_OVRD format #2 numerator offset for -1MHz */
#endif                         /* defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 2U) */
#if defined(RSM_WA_KFOURWONE_1164)
static uint8_t store_rsm_trig_sel;
#endif /* defined (RSM_WA_KFOURWONE_1164) */

static struct
{
    uint32_t tsm_ovrd0;
    uint32_t tsm_ovrd1;
    uint32_t tsm_ovrd2;
    uint32_t tsm_ovrd3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)  
    uint32_t tsm_ovrd4;
#endif
    uint32_t tx_dig_data_padding_ctrl;
    uint32_t tx_dig_data_padding_ctrl_1;
    uint32_t tx_dig_pa_ctrl;
} xcvr_settings;

xcvr_lcl_hpm_cal_interp_t hpm_cal_2442_data;

// static bool cont_phase_ovrd_active = false;
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

    /* Independent of whether RSM dma mask is used, check validity of RSM_DMA_DUR0 (only used for both SQTE and PDE) */
    if (!XCVR_LCL_RsmCheckDmaDuration(rsm_settings_ptr->rsm_dma_dur_fm_ext, rsm_settings_ptr->rate,
                                      rsm_settings_ptr->averaging_win))
    {
        status = gXcvrLclStatusInvalidArgs;
    }

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
        tmp_logic_sum += (uint32_t)(rsm_settings_ptr->op_mode != XCVR_RSM_SQTE_MODE);

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
    temp |= XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD( HOP_TBL_CFG_OVRD); /* RSM asserts the OVRD_EN directly through hardware logic, SW need not set it */
    XCVR_PLL_DIG->CHAN_MAP = temp;
    
    /* Overrides to keep ADC on in both RX and TX to have consistent voltages on LDOs */
    XCVR_TSM->OVRD3 |= (XCVR_TSM_OVRD3_SEQ_ADC_PUP_OVRD_MASK | XCVR_TSM_OVRD3_SEQ_ADC_PUP_OVRD_EN_MASK);
    XCVR_TSM->OVRD2 |= (XCVR_TSM_OVRD2_SEQ_XO_DIST_EN_CLK_ADCDAC_OVRD_MASK |
                                        XCVR_TSM_OVRD2_SEQ_XO_DIST_EN_CLK_ADCDAC_OVRD_EN_MASK |
                                        XCVR_TSM_OVRD2_SEQ_BG_PUP_IBG_RX_OVRD_MASK | 
                                        XCVR_TSM_OVRD2_SEQ_BG_PUP_IBG_RX_OVRD_EN_MASK);

    /* Tuning cap overrides to prevent amplitude variations during HADM */
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
    if (rate == XCVR_RSM_RATE_1MBPS)
    {
        /* Settings for 1Mpbs operation */
        XCVR_PLL_DIG->HPM_CTRL |= XCVR_PLL_DIG_HPM_CTRL_RX_HPM_CAL_EN_MASK; /* Needed ? */

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
        
        temp = XCVR_PLL_DIG->HPM_SDM_RES;
        temp &= ~(XCVR_PLL_DIG_HPM_SDM_RES_HPM_COUNT_ADJUST_MASK);
        temp |= XCVR_PLL_DIG_HPM_SDM_RES_HPM_COUNT_ADJUST(0U);
        XCVR_PLL_DIG->HPM_SDM_RES = temp;
        
        /* IF compensation improvement */
        temp = XCVR_PLL_DIG->MOD_CTRL;
#if defined(CONNRF_1163_IF_COMP) && (CONNRF_1163_IF_COMP == 1)
        XCVR_PLL_DIG->HPM_BUMP = XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_TX(2U) | XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_CAL(2U) |
                                 XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_TX(2U) | XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_CAL(2U) |
                                 XCVR_PLL_DIG_HPM_BUMP_PLL_VCO_TRIM_KVM_TX(2U) |
                                 XCVR_PLL_DIG_HPM_BUMP_PLL_VCO_TRIM_KVM_CAL(2U);

        XCVR_PLL_DIG->LPM_SDM_CTRL1 |= XCVR_PLL_DIG_LPM_SDM_CTRL1_HPM_ARRAY_BIAS(63U);
        temp &= ~(XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL_MASK);
        temp |= XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(0x1950U);
#else
        XCVR_PLL_DIG->HPM_BUMP = XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_TX(2U) | XCVR_PLL_DIG_HPM_BUMP_HPM_VCM_CAL(2U) |
                                 XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_TX(0U) | XCVR_PLL_DIG_HPM_BUMP_HPM_FDB_RES_CAL(0U) |
                                 XCVR_PLL_DIG_HPM_BUMP_PLL_VCO_TRIM_KVM_TX(6U) |
                                 XCVR_PLL_DIG_HPM_BUMP_PLL_VCO_TRIM_KVM_CAL(6U);
        temp &= ~(XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL_MASK);
        temp |= XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(0x1001U);
#endif /* defined(CONNRF_1163_IF_COMP) && (CONNRF_1163_IF_COMP == 1) */
        XCVR_PLL_DIG->MOD_CTRL = temp;
    }
    else
    {
        /* Settings for 2 Mbps operation */
        temp = XCVR_TX_DIG->DATARATE_CONFIG_GFSK_CTRL;
        temp &=  ~(XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV_MASK);
        temp |=  XCVR_TX_DIG_DATARATE_CONFIG_GFSK_CTRL_DATARATE_CONFIG_GFSK_FDEV(0x400); /* Default: 0x400 */
        XCVR_TX_DIG->DATARATE_CONFIG_GFSK_CTRL = temp;    

        temp = XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL1;
        temp &= ~(XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_LPM_SCALE_CFG1_MASK  | XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_CAL_SCALE_CFG1_MASK);
        temp |= (XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_LPM_SCALE_CFG1(0x9) | /* Default: 0x8 */
                    XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_CAL_SCALE_CFG1(0xAU)); 
        XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL1 = temp;

        /* HPM BUMP for 2Mbps 662222*/
        // TODO: should there be a 2Mbps HPM BUMP
        
        temp = XCVR_PLL_DIG->PLL_DATARATE_CTRL;
        temp &= ~(XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_TX_DRS_MASK | 
                        XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_CAL_DRS_MASK);
        temp |= (XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_TX_DRS(6U) |
                    XCVR_PLL_DIG_PLL_DATARATE_CTRL_PLL_VCO_TRIM_KVM_CAL_DRS(6U));
        XCVR_PLL_DIG->PLL_DATARATE_CTRL = temp;

        temp =XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL1 ;
        temp &= ~(XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_TX_CFG1_MASK  | 
                    XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_CAL_CFG1_MASK);
        temp |= (XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_TX_CFG1(2U) |
                    XCVR_PLL_DIG_DATA_RATE_OVRD_CTRL1_HPM_FDB_RES_CAL_CFG1(2U));
        XCVR_PLL_DIG->DATA_RATE_OVRD_CTRL1 = temp;

        temp = XCVR_PLL_DIG->PLL_DATARATE_CTRL;
        temp &= ~(XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_TX_DRS_MASK |
                            XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_CAL_DRS_MASK);
        temp |= (XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_TX_DRS(2U) |
                    XCVR_PLL_DIG_PLL_DATARATE_CTRL_HPM_VCM_CAL_DRS(2U));
        XCVR_PLL_DIG->PLL_DATARATE_CTRL = temp;
        
        temp = XCVR_PLL_DIG->MOD_CTRL;
#if defined(CONNRF_1163_IF_COMP) && (CONNRF_1163_IF_COMP == 1)
        XCVR_PLL_DIG->LPM_SDM_CTRL1 |= XCVR_PLL_DIG_LPM_SDM_CTRL1_HPM_ARRAY_BIAS(63U);
        /* Define the frequency push on the VCO */
        temp &= ~(XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL_MASK);
        temp |= XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(
            0x15F8U); /* 0x15F8U is the best setting for -1.5 MHz@LO with HPM_CAL_SCALE_CFG1=0x9 */

#else
        /* Define the frequency push on the VCO */
        temp &= ~(XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL_MASK);
        temp |=  XCVR_PLL_DIG_MOD_CTRL_MODULATION_WORD_MANUAL(0x1000U); /* 0x1000U is -1 MHz@LO if HPM_CAL_SCALE_CFG1=0x9 */
#endif /* defined(CONNRF_1163_IF_COMP) && (CONNRF_1163_IF_COMP == 1) */
        XCVR_PLL_DIG->MOD_CTRL = temp;
    }
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
        /* backup all changed registers for later restore */
        reg_backup_ptr->XCVR_PLL_DIG_CHAN_MAP    = XCVR_PLL_DIG->CHAN_MAP;
        reg_backup_ptr->XCVR_PLL_DIG_HPM_CTRL    = XCVR_PLL_DIG->HPM_CTRL;
        reg_backup_ptr->XCVR_TX_DIG_GFSK_CTRL    = XCVR_TX_DIG->GFSK_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_LPM_CTRL    = XCVR_PLL_DIG->LPM_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_LPM_SDM_CTRL1 = XCVR_PLL_DIG->LPM_SDM_CTRL1;
        reg_backup_ptr->XCVR_PLL_DIG_DELAY_MATCH = XCVR_PLL_DIG->DELAY_MATCH;
        reg_backup_ptr->XCVR_PLL_DIG_HPM_SDM_RES = XCVR_PLL_DIG->HPM_SDM_RES;
        reg_backup_ptr->XCVR_PLL_DIG_HPM_BUMP    = XCVR_PLL_DIG->HPM_BUMP;
        reg_backup_ptr->XCVR_PLL_DIG_MOD_CTRL    = XCVR_PLL_DIG->MOD_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_PLL_NUM_OFFSET =
            XCVR_PLL_DIG->PLL_OFFSET_CTRL; /* this register is backed up because CFO compensation can corrupt it */
        reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_TX_CTRL = XCVR_PLL_DIG->TUNING_CAP_TX_CTRL;
        reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_RX_CTRL = XCVR_PLL_DIG->TUNING_CAP_RX_CTRL;
        reg_backup_ptr->XCVR_TSM_OVRD2 =
            XCVR_TSM->OVRD2; /* TSM Overrides controlling the PLL signals, so included in PLL backup/restore */
        reg_backup_ptr->XCVR_TSM_OVRD3 =
            XCVR_TSM->OVRD3; /* TSM Overrides controlling the PLL signals, so included in PLL backup/restore */
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
                ->XCVR_PLL_DIG_PLL_NUM_OFFSET; /* this register is restored because CFO compensation can corrupt it */
        XCVR_PLL_DIG->TUNING_CAP_TX_CTRL = reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_TX_CTRL;
        XCVR_PLL_DIG->TUNING_CAP_RX_CTRL = reg_backup_ptr->XCVR_PLL_DIG_TUNING_CAP_RX_CTRL;
        XCVR_TSM->OVRD2 =
            reg_backup_ptr
                ->XCVR_TSM_OVRD2; /* TSM Overrides controlling the PLL signals, so included in PLL backup/restore */
        XCVR_TSM->OVRD3 =
            reg_backup_ptr
                ->XCVR_TSM_OVRD3; /* TSM Overrides controlling the PLL signals, so included in PLL backup/restore  */
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
#ifndef XCVR_SKIP_RSM_SETTINGS_CHECK
    else
    {
        /* Verify the settings within the structure are valid */
        status = XCVR_LCL_ValidateRsmSettings(
            rsm_settings_ptr); /* Separate routine allows easier testing of this validation step */
    }
#endif
    if (status == gXcvrLclStatusSuccess)
    {
        bool rate_is_2mbps =
            (rsm_settings_ptr->rate == XCVR_RSM_RATE_2MBPS); /* True == 2Mbps rate; False == 1Mbps rate */
        bool is_sqte_mode =
            (rsm_settings_ptr->op_mode == XCVR_RSM_SQTE_MODE); /* Will need to use this test frequently */
        uint32_t temp;
        /* ************** */
        /* Setup LCL for ranging */
        /* ************** */
        /* Setup LCL_CTRL for antenna switching & DMA durations */
        /* Setup XCVR_DMA to capture phase and measurement samples  */
        /* Setup DSB to support the XCVR_DMA sample capture */
        // XCVR_MISC_DMA_CTRL_DMA_START_TRG set to RSM trigger.
        // XCVR_MISC_DMA_CTRL_DMA_SIGNAL_VALID_MASK_EN = 1

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

        /* ************** */
        /* Setup TXDIG for ranging */
        /* ************** */
        /* save for later restore */
        xcvr_settings.tx_dig_data_padding_ctrl   = XCVR_TX_DIG->DATA_PADDING_CTRL;
        xcvr_settings.tx_dig_data_padding_ctrl_1 = XCVR_TX_DIG->DATA_PADDING_CTRL_1;
        xcvr_settings.tx_dig_pa_ctrl             = XCVR_TX_DIG->PA_CTRL;
        
        temp = xcvr_settings.tx_dig_pa_ctrl;
        temp &= ~(XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL_MASK);
        temp |= XCVR_TX_DIG_PA_CTRL_PA_RAMP_SEL(1U);
        XCVR_TX_DIG->PA_CTRL = temp;

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
            temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(1U); /* Setup for 2Mbps rate */
        }
        else
        {
            temp |= XCVR_TX_DIG_DATA_PADDING_CTRL_1_TX_DATA_FLUSH_DLY(2U); /* Setup for 1Mbps rate */
        }
        XCVR_TX_DIG->DATA_PADDING_CTRL_1 = temp;

        /* ************** */
        /* Setup RXDIG for ranging */
        /* ************** */
        /* If RCCAL is not performed, setup the CBPF override value */
        temp = XCVR_RX_DIG->RCCAL_CTRL1;
        if (!xcvr_lcl_rsm_generic_config.do_rxdig_rccal)
        {
            temp |= XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD_EN_MASK; /* Enable override of CBPF value */
            temp &= ~(XCVR_RX_DIG_RCCAL_CTRL1_RCCAL_CODE_OVRD_MASK);
            if (rate_is_2mbps)
            {
                temp |= XCVR_RX_DIG_RCCAL_CTRL1_RCCAL_CODE_OVRD(0x2BU); /* Setup for 2Mbps rate */
            }
            else
            {
                temp |= XCVR_RX_DIG_RCCAL_CTRL1_RCCAL_CODE_OVRD(0x4DU); /* Setup for 1Mbps rate */
            }
            
        }
        else
        {
            temp &= ~(XCVR_RX_DIG_RCCAL_CTRL1_CBPF_CCODE_OVRD_EN_MASK); /* Disable override of CBPF value */
        }
        XCVR_RX_DIG->RCCAL_CTRL1 = temp;

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
        temp &= ~(XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_OUT_SEL_MASK | XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_MAG_OUT_SEL_MASK |
                  XCVR_RX_DIG_DFT_CTRL_DFT_RX_PH_OUT_SEL_MASK);
        temp |= XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_OUT_SEL(4U) | XCVR_RX_DIG_DFT_CTRL_DFT_RSSI_MAG_OUT_SEL(7U) |
                XCVR_RX_DIG_DFT_CTRL_DFT_RX_PH_OUT_SEL(2U);
        XCVR_RX_DIG->DFT_CTRL = temp;
        temp                  = XCVR_RX_DIG->CTRL1;
        temp |= XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK;
        temp &= ~(XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_MASK);
        /* Setup averager (in the same register) */
        temp |= XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN(
            rsm_settings_ptr->averaging_win); /* Setup according to input structure */
        XCVR_RX_DIG->CTRL1 = temp;
        /* Setup DMA start trigger using DMA configuration APIs */

        /* ************** */
        /* Setup TSM for ranging */
        /* ************** */
        status = XCVR_LCL_ReprogramTsmTimings(&xcvr_lcl_tsm_generic_config); /* TSM settings are stored in const config structures */

        temp = XCVR_RX_DIG->DFT_CTRL;
#if (1) /* TSM OVERRIDES for preserving PLL constant phase */
        xcvr_settings.tsm_ovrd0 = XCVR_TSM->OVRD0;
        xcvr_settings.tsm_ovrd1 = XCVR_TSM->OVRD1;
        xcvr_settings.tsm_ovrd2 = XCVR_TSM->OVRD2;
        xcvr_settings.tsm_ovrd3 = XCVR_TSM->OVRD3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)  
        xcvr_settings.tsm_ovrd4 = XCVR_TSM->OVRD4;
#endif

        temp |= XCVR_RX_DIG_DFT_CTRL_CGM_OVRD(4); // to maintain rx_dig_mixer_clk
#endif
        /* [CONNRF-1139] FIX Tx_gain not applied when TX WU runs DCOC */
        temp |= 1UL << (9U + XCVR_RX_DIG_DFT_CTRL_CGM_OVRD_SHIFT);
        XCVR_RX_DIG->DFT_CTRL = temp;

        /* ************** */
        /* Setup RSM for ranging */
        /* ************** */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL0; /* Structure based configuration for the RSM_CTRL0
                                                         register; No logic configurations to follow */
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
        /* Configure trigger select, trigger delay and number of steps */
        temp |= (XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL((uint32_t)rsm_settings_ptr->trig_sel) |
                 XCVR_MISC_RSM_CTRL0_RSM_TRIG_DLY((uint32_t)rsm_settings_ptr->trig_delay) |
                 XCVR_MISC_RSM_CTRL0_RSM_STEPS((uint32_t)rsm_settings_ptr->num_steps));
        XCVR_MISC->RSM_CTRL0 = temp;
#if defined(RSM_WA_KFOURWONE_1164)
        store_rsm_trig_sel = (uint8_t)(rsm_settings_ptr->trig_sel);
#endif /* defined (RSM_WA_KFOURWONE_1164) */

        /* Write T_FC/T_IP values */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL1; /* Structure based configuration for the RSM_CTRL1
                                                         register; No logic configurations to follow */
        temp &= ~(XCVR_MISC_RSM_CTRL1_RSM_T_FC_MASK | XCVR_MISC_RSM_CTRL1_RSM_T_IP1_MASK |
                                  XCVR_MISC_RSM_CTRL1_RSM_T_IP2_MASK);
        temp |= (XCVR_MISC_RSM_CTRL1_RSM_T_FC((uint8_t)((rsm_settings_ptr->t_fc + T_RD) / T_FC_INCMT)) |
                                 XCVR_MISC_RSM_CTRL1_RSM_T_IP1((uint8_t)((rsm_settings_ptr->t_ip1 + T_RD) / T_IP_INCMT)) |
                                 XCVR_MISC_RSM_CTRL1_RSM_T_IP2((uint8_t)((rsm_settings_ptr->t_ip2 + T_RD) / T_IP_INCMT)));
        XCVR_MISC->RSM_CTRL1 = temp;

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
                XCVR_MISC_RSM_CTRL2_RSM_T_PM1((uint32_t)(t_pm0_setting) + 10U);
#endif
        XCVR_MISC->RSM_CTRL2 = temp;

        /* Configure RSM_DT_RX_SYNC_DLY & RSM_DMA_RX_EN & RSM_DMA_DUR */
        temp = xcvr_lcl_rsm_generic_config.RSM_CTRL3; /* Structure based configuration for the RSM_CTRL3 register; Later
                                                      logic will configure additional fields */
        temp &= ~(XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN_MASK | XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY_MASK |
                  XCVR_MISC_RSM_CTRL3_RSM_DMA_DUR_MASK);    /* Zero out all fields being modified by logic */
        if (rate_is_2mbps)
        {
          temp |= XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY(7U); /* Original setting for both 1Mbps & 2Mbps rates (now testing only for 2Mbps) */
        }
        else
        {
          temp |= XCVR_MISC_RSM_CTRL3_RSM_DT_RX_SYNC_DLY(6U); /* Decreased by 1 for 1Mbps to test shifting sync point */
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
            temp &= ~(XCVR_MISC_RSM_CTRL3_RSM_DMA_RX_EN_MASK); /* Setup to use the LCL DMA mask control */
        }
        XCVR_MISC->RSM_CTRL3 = temp;

        /* Configure RSM_DMA_DUR0 & RSM_DMA_DLY0 independent of whether using dma mask (these are always used for FCS
         * sequence)*/
        temp = XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY0(rsm_settings_ptr->rsm_dma_dly_fm_ext) |
               XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0(rsm_settings_ptr->rsm_dma_dur_fm_ext);
        /* Configure RSM_DMA_DLY  if using dma mask */
        if (rsm_settings_ptr->use_rsm_dma_mask)
        {
            temp |= XCVR_MISC_RSM_CTRL4_RSM_DMA_DLY(rsm_settings_ptr->rsm_dma_dly_pm);
        }
        XCVR_MISC->RSM_CTRL4 = temp;

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

xcvrLclStatus_t XCVR_LCL_Set_TSM_FastStart(XCVR_RSM_RXTX_MODE_T role, uint32_t t_fc, uint32_t t_ip1, uint32_t t_ip2)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    uint32_t t_ip = (t_ip1 <= t_ip2) ? t_ip1 : t_ip2;

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

    fast_ctrl2 &= ~(XCVR_TSM_FAST_CTRL2_FAST_START_RX_MASK | XCVR_TSM_FAST_CTRL2_FAST_START_TX_MASK);
    uint32_t temp_fast_rx;
    uint32_t temp_fast_tx;
    switch (role)
    {
        /* implement CONNRF-1142 */
        case XCVR_RSM_RX_MODE:
            temp_fast_rx = t_fc + T_RD + fast_dest_rx + fast_tx2rx_start_fc - end_of_rx_wu - 4U;
            temp_fast_tx = t_ip + T_RD + fast_dest_tx + fast_rx2tx_start - end_of_tx_wu - 18U;
            break;
        case XCVR_RSM_TX_MODE:
            temp_fast_rx = t_ip + T_RD + fast_dest_rx + fast_tx2rx_start - end_of_rx_wu - 4U;
            temp_fast_tx = t_fc + T_RD + fast_dest_tx + fast_rx2tx_start_fc - end_of_tx_wu - 18U;
            break;
        default:
            status = gXcvrLclStatusFail; /* Error, default case should never be encountered */
            break;
    }
    if (status == gXcvrLclStatusSuccess)
    {
        fast_ctrl2 |= XCVR_TSM_FAST_CTRL2_FAST_START_RX(temp_fast_rx) | XCVR_TSM_FAST_CTRL2_FAST_START_TX(temp_fast_tx);
        XCVR_TSM->FAST_CTRL2 = fast_ctrl2;
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_RsmGo(XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t *rsm_settings_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    (void)rsm_settings_ptr; /* Retain the rsm_settings_ptr parameter for future possible use; (void) touches it for
                               unused parameter errors */
#if defined(RSM_WA_KFOURWONE_1164)
    XCVR_MISC->RSM_CTRL0 &= ~(XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL_MASK);
    XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL(store_rsm_trig_sel);
#endif /* defined (RSM_WA_KFOURWONE_1164) */

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
            XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_TX_EN_MASK; /* Enable TX */
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

#if defined(RSM_WA_KFOURWONE_1164)
    XCVR_MISC->RSM_CTRL0 &= ~(XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL_MASK);
    XCVR_MISC->RSM_CTRL0 |= XCVR_MISC_RSM_CTRL0_RSM_TRIG_SEL(XCVR_RSM_TRIG_CRC_VLD);
    uint8_t i = 64U; /* count for delay loop */
    while (i > 0U)
    {
        i--;
    }
#endif /* defined (RSM_WA_KFOURWONE_1164) */

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
#if BACKUP_LCL_REGS
        reg_backup_ptr->XCVR_MISC_DMA_CTRL            = XCVR_MISC->DMA_CTRL;
        reg_backup_ptr->XCVR_MISC_LCL_CFG0            = XCVR_MISC->LCL_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_CFG1            = XCVR_MISC->LCL_CFG1;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG0         = XCVR_MISC->LCL_TX_CFG0;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG1         = XCVR_MISC->LCL_TX_CFG1;
        reg_backup_ptr->XCVR_MISC_LCL_TX_CFG2         = XCVR_MISC->LCL_TX_CFG2;
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
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
        reg_backup_ptr->XCVR_MISC_RSM_CTRL5           = XCVR_MISC->RSM_CTRL5;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL6           = XCVR_MISC->RSM_CTRL6;
        reg_backup_ptr->XCVR_MISC_RSM_CTRL7           = XCVR_MISC->RSM_CTRL7;
        reg_backup_ptr->XCVR_MISC_RSM_INT_ENABLE   = XCVR_MISC->RSM_INT_ENABLE;
#endif
#endif /* BACKUP_LCL_REGS */
        /* XCVR_2P4GHZ_PHY */
        reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_CTRL = XCVR_2P4GHZ_PHY->RTT_CTRL;
        reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_REF  = XCVR_2P4GHZ_PHY->RTT_REF;


        /* XCVR_TXDIG */
        reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL  = XCVR_TX_DIG->DATA_PADDING_CTRL;
        reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL1 = XCVR_TX_DIG->DATA_PADDING_CTRL_1;
        reg_backup_ptr->XCVR_TX_DIG_PA_CTRL            = XCVR_TX_DIG->PA_CTRL;
        /* XCVR_PLL */
        status = XCVR_LCL_RsmPLLBackup(reg_backup_ptr); /* NULLPTR check in this routine should never fail since checked above */
        /* XCVR_RXDIG */
        reg_backup_ptr->XCVR_RX_DIG_RCCAL_CTRL1 = XCVR_RX_DIG->RCCAL_CTRL1;
        reg_backup_ptr->XCVR_RX_DIG_DFT_CTRL    = XCVR_RX_DIG->DFT_CTRL;
        reg_backup_ptr->XCVR_RX_DIG_CTRL1       = XCVR_RX_DIG->CTRL1;
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
#if BACKUP_LCL_REGS
        XCVR_MISC->DMA_CTRL            = reg_backup_ptr->XCVR_MISC_DMA_CTRL;
        XCVR_MISC->LCL_CFG0            = reg_backup_ptr->XCVR_MISC_LCL_CFG0;
        XCVR_MISC->LCL_CFG1            = reg_backup_ptr->XCVR_MISC_LCL_CFG1;
        XCVR_MISC->LCL_TX_CFG0         = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG0;
        XCVR_MISC->LCL_TX_CFG1         = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG1;
        XCVR_MISC->LCL_TX_CFG2         = reg_backup_ptr->XCVR_MISC_LCL_TX_CFG2;
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
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
        XCVR_MISC->RSM_CTRL5 = reg_backup_ptr->XCVR_MISC_RSM_CTRL5;
        XCVR_MISC->RSM_CTRL6 = reg_backup_ptr->XCVR_MISC_RSM_CTRL6;
        XCVR_MISC->RSM_CTRL7 = reg_backup_ptr->XCVR_MISC_RSM_CTRL7;
        XCVR_MISC->RSM_INT_ENABLE = reg_backup_ptr->XCVR_MISC_RSM_INT_ENABLE;
#endif
#endif /* BACKUP_LCL_REGS */
        /* XCVR_2P4GHZ_PHY */
        XCVR_2P4GHZ_PHY->RTT_CTRL = reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_CTRL;
        XCVR_2P4GHZ_PHY->RTT_REF  = reg_backup_ptr->XCVR_2P4GHZ_PHY_RTT_REF;
        /* XCVR_TXDIG */
        XCVR_TX_DIG->DATA_PADDING_CTRL   = reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL;
        XCVR_TX_DIG->DATA_PADDING_CTRL_1 = reg_backup_ptr->XCVR_TX_DIG_DATA_PADDING_CTRL1;
        XCVR_TX_DIG->PA_CTRL             = reg_backup_ptr->XCVR_TX_DIG_PA_CTRL;
        /* XCVR_PLL */
        status = XCVR_LCL_RsmPLLRestore(reg_backup_ptr); /* NULLPTR check in this routine should never fail since checked above */
        /* XCVR_RXDIG */
        XCVR_RX_DIG->RCCAL_CTRL1 = reg_backup_ptr->XCVR_RX_DIG_RCCAL_CTRL1;
        XCVR_RX_DIG->DFT_CTRL    = reg_backup_ptr->XCVR_RX_DIG_DFT_CTRL;
        XCVR_RX_DIG->CTRL1       = reg_backup_ptr->XCVR_RX_DIG_CTRL1;
        /* RADIO_CTRL */
        RADIO_CTRL->RF_CTRL = reg_backup_ptr->RADIO_CTRL_RF_CTRL;
    }

    return status;
}

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

#define HPM_CAL_IN_RX 0
xcvrLclStatus_t XCVR_LCL_CalibratePll(const channel_num_t *freq_list,
                                      xcvr_lcl_pll_cal_data_t *cal_results,
                                      uint16_t num_freqs,
                                      bool update_curve_fit,
                                      XCVR_RSM_SQTE_RATE_T rate)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer and invalid sequence lengths */
    if ((freq_list == NULLPTR) || (cal_results == NULLPTR))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status = XCVR_LCL_RsmCheckSeqLen(num_freqs,
                                         XCVR_RSM_OVERALL_MAX_SEQ_LEN); /* check that sequence length is not exceeded */
        ;
    }

    if (status == gXcvrLclStatusSuccess)
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
        const channel_num_t *list_ptr        = freq_list;
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
            XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD_EN_MASK | XCVR_PLL_DIG_CHAN_MAP_HOP_TBL_CFG_OVRD(HOP_TBL_CFG_OVRD);
        /* During calibration, HPM_CAL must always happen. CTUNE should be enabled by default so is not handled here */
        XCVR_PLL_DIG->HPM_CTRL &=
            ~(XCVR_PLL_DIG_HPM_CTRL_RX_HPM_CAL_EN_MASK); /* HPM cal is performed during warmup (not driven from RSM) */
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
    if ((hadm_chan_idx_list == NULLPTR) || (cal_results == NULLPTR) || (hpm_cal_interp == NULLPTR))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        status = XCVR_LCL_RsmCheckSeqLen(num_freqs,
                                         XCVR_RSM_OVERALL_MAX_SEQ_LEN); /* check that sequence length is not exceeded */
        ;
    }

    if (status == gXcvrLclStatusSuccess)
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
     if( (timing13_backup & XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_HI_MASK >> XCVR_TSM_TIMING13_DCOC_CAL_EN_RX_HI_SHIFT) == 0xFFU ) 
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
    // if another gain must be used for a better input signal measurement, it must be configured here ( DCOC_GAIN_CFG_EN to be forced LOW by override + AGC gain config) */
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
#if defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 3U)
        *fstep_chan_num = channel_num_from_hadm_chan[hadm_chan_index];
#endif /* defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 3U) */
#if defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 2U)
        uint16_t mapped_chan_num =
            (uint16_t)(hadm_chan_index) >>
            1U; /* divide by 2 to get the normal BLE channel index for format #2 HOP_TBL_CFG_OVRD */
        if ((hadm_chan_index & 0x1U) == 0x1U) /* original HADM channel was an odd number */
        {
            mapped_chan_num++; /* go to next channel up (2MHz higher) to allow -1MHz to hit the target channel */
            mapped_chan_num |= OFFSET_NEG_1MHZ << 7U; /* Apply -1MHz */
        }
        *fstep_chan_num = mapped_chan_num;
#endif /* defined(HOP_TBL_CFG_OVRD) && (HOP_TBL_CFG_OVRD == 2U) */
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
    const xcvr_lcl_fstep_t *fstep_ptr = fstep_settings;

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
        /* Select whether to use DMA duration from RSM registers or LCL antenna control */
        uint8_t dma_fm_dur  = (uint8_t)((XCVR_MISC->RSM_CTRL4 & XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0_MASK) >>
                                       XCVR_MISC_RSM_CTRL4_RSM_DMA_DUR0_SHIFT);
        uint16_t dma_iq_avg = (uint16_t)(((XCVR_RX_DIG->CTRL1 & XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_MASK) >>
                                          XCVR_RX_DIG_CTRL1_RX_IQ_PH_AVG_WIN_SHIFT));
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
        uint16_t t_dt_usec = 0U;
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
        t_dt_usec = (((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                         ((16U + 64U + 4U) / 2U) :
                         8U + 64U + 4U); /* 8bit preamble, 64 bit PN seq, 4 bit trailer */
#else
        t_dt_usec          = (((XCVR_RSM_SQTE_RATE_T)rate == XCVR_RSM_RATE_2MBPS) ?
                         ((16U + 32U + 4U) / 2U) :
                         8U + 32U + 4U); /* 8bit preamble, 32 bit PN seq, 4 bit trailer */
#endif
        uint16_t t_pm_usec[T_PM_FLD_COUNT];
        temp = XCVR_MISC->RSM_CTRL2;
        /* Calculate values in usec from the register contents */
        t_pm_usec[0] = (uint16_t)(
            T_PM_INCMT * (1U + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM0_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM0_SHIFT)));
        t_pm_usec[1] = (uint16_t)(
            T_PM_INCMT * (1U + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM1_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM1_SHIFT)));
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
        t_pm_usec[2] = (uint16_t)(
            T_PM_INCMT * (1U + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM2_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM2_SHIFT)));
        t_pm_usec[3] = (uint16_t)(
            T_PM_INCMT * (1U + ((temp & XCVR_MISC_RSM_CTRL2_RSM_T_PM3_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_T_PM3_SHIFT)));
#endif
        if (dma_iq_avg == 0U)
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

        /* Read Step configuration from RAM */
        uint16_t dma_samples = 0U;
        uint16_t temp_samples;
        uint16_t sequence_length_us    = 0U;
        static uint16_t step_length_us = 0U;
        for (uint8_t i = 0; i < num_steps; i++)
        {
            temp                = fstep_ptr->tpm_step_format_hmp_cal_factor_msb;
            uint8_t step_format = (uint8_t)((temp & XCVR_RSM_STEP_FORMAT_MASK) >> XCVR_RSM_STEP_FORMAT_SHIFT);
            uint8_t t_pm_sel    = (uint8_t)((temp & XCVR_RSM_T_PM_FM_SEL_MASK) >> XCVR_RSM_T_PM_FM_SEL_SHIFT);
            step_length_us      = 0U;
            switch ((XCVR_RSM_FSTEP_TYPE_T)step_format)
            {
                case XCVR_RSM_STEP_FCS:
                    if (role == XCVR_RSM_TX_MODE)
                    {
                        dma_samples +=
                            (dma_fm_dur * sample_rate_per_usec /
                             dma_iq_avg); /* Only Initiator captures DMA samples for the frequency compensation */
                    }
                    /* Seq Len = T_FC+2*T_DT+T_IP1+T_S+T_FM */
                    step_length_us =
                        t_fc_usec + (2U * t_dt_usec) + t_ip1_usec + t_s_usec + t_fm_usec[t_pm_sel];
                    break;
                case XCVR_RSM_STEP_PK_PK:
                    /* No DMA capture in this step */
                    /* Seq Len = T_FC+2*T_DT+T_IP1 */
                    step_length_us = t_fc_usec + (2U * t_dt_usec) + t_ip1_usec;
                    break;
                case XCVR_RSM_STEP_TN_TN:
                    temp_samples = dma_pm_dur;
                    if (!rsm_dma_mask_used)
                    {
                        /* RSM DMA mask not used == LCL block used; Must consider multi-ant */
                        temp_samples = temp_samples * ((uint16_t)ant_cnt+(uint16_t)TONE_EXT_COUNT); /* DMA capture is repeated for each antenna */
                    }
                    temp_samples = temp_samples * sample_rate_per_usec;
                    temp_samples = temp_samples / dma_iq_avg;
                    dma_samples += temp_samples; 
                    /* Seq Len = T_FC+2*T_PM*NUM_ANT+T_IP2 */
                    step_length_us = t_fc_usec + (2U * t_pm_usec[t_pm_sel]) + t_ip2_usec;
                    break;
                case XCVR_RSM_STEP_PK_TN_TN_PK:
                    temp_samples = dma_pm_dur;
                    if (!rsm_dma_mask_used)
                    {
                        /* RSM DMA mask not used == LCL block used; Must consider multi-ant */
                        temp_samples = temp_samples * ((uint16_t)ant_cnt+(uint16_t)TONE_EXT_COUNT); /* DMA capture is repeated for each antenna */

                    }
                    temp_samples = temp_samples * sample_rate_per_usec;
                    temp_samples = temp_samples / dma_iq_avg;
                    dma_samples += temp_samples; 
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
            fstep_ptr++;
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
