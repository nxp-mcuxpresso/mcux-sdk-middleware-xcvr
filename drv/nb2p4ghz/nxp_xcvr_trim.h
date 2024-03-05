/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2018-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef FSL_XCVR_TRIM_H
/* clang-format off */
#define FSL_XCVR_TRIM_H
/* clang-format on */

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "nxp2p4_xcvr.h"

/*!
 * @addtogroup xcvr_trim Radio Trim and Test Routines
 * @{
 */

/************************************************************************************
*************************************************************************************
* Public constant definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/
/* \brief  The enumerations used to define the I & Q channel selections. */
typedef enum
{
    I_CHANNEL    = 0U,
    Q_CHANNEL    = 1U,
    NUM_I_Q_CHAN = 2U
} IQ_t;

typedef enum
{
    gXcvrTrimSuccess_c        = 0U, /*!< Trim successful */
    gXcvrTrimInvalidMode_c    = 1U, /*!< The radio mode for IQMC calibration is invalid. */
    gXcvrTrimInvalidChannel_c = 2U, /*!< The radio channel for IQMC calibration is invalid. */
    gXcvrTrimIqmcCalOutOfRange_c =
        6U, /*!<  The IQMC_GAIN_ADJ or IQMC_PHASE_ADJ value(s) are out of the target range. */
    gXcvrTrimIqmcCalValFailRanges_c = 7U /*!<The validation of IQCM calibration failed. */
} xcvrTrimStatus_t;

#if defined(GCOV_DO_COVERAGE)      /* definition is local except when testing code coverage */
#define IQMCalibrationTrials (32U) /* Number of times the calibration is repeated & averaged */
/*! @brief IQMC calibration trials storage. */
typedef struct
{
    uint16_t IQMC_gain_cal_trials[IQMCalibrationTrials]; /*!< IQ Gain trials results */
    int16_t
        IQMC_phase_cal_trials[IQMCalibrationTrials]; /*!< IQ Phase trials results (must be tracked as signed value) */
    uint32_t IQMC_gain_adj_sum;                      /*!< IQ  Gain results running sum */
    int32_t IQMC_phase_adj_sum; /*!< IQ  Phase results running sum (must be tracked as signed value) */
} xcvr_iqmc_trials_t;
#endif /* !defined(GCOV_DO_COVERAGE) */

typedef uint8_t DAC_SWEEP_STEP2_t;
/* Enumeration of ADC_GAIN_CAL 2 */
#define NOMINAL2 (0U)
#define BBF_NEG (1U)
#define BBF_POS (2U)
#define TZA_STEP_N0 (3U)
#define TZA_STEP_N1 (4U)
#define TZA_STEP_N2 (5U)
#define TZA_STEP_N3 (6U)
#define TZA_STEP_N4 (7U)
#define TZA_STEP_N5 (8U)
#define TZA_STEP_N6 (9U)
#define TZA_STEP_N7 (10U)
#define TZA_STEP_N8 (11U)
#define TZA_STEP_N9 (12U)
#define TZA_STEP_N10 (13U)
#define TZA_STEP_P0 (14U)
#define TZA_STEP_P1 (15U)
#define TZA_STEP_P2 (16U)
#define TZA_STEP_P3 (17U)
#define TZA_STEP_P4 (18U)
#define TZA_STEP_P5 (19U)
#define TZA_STEP_P6 (20U)
#define TZA_STEP_P7 (21U)
#define TZA_STEP_P8 (22U)
#define TZA_STEP_P9 (23U)
#define TZA_STEP_P10 (24U)
#define NUM_SWEEP_STEP_ENTRIES2 (3U) /* Including the baseline entry #0. */

/* \brief  Defines an entry in an array of structs to describe TZA DCOC STEP and TZA_DCOC_STEP_RECIPROCAL. */
typedef struct
{
    uint16_t dcoc_step;
    uint16_t dcoc_step_rcp;
} TZAdcocstep_t;

typedef struct
{
    int8_t step_value;            /* The offset from nominal DAC value (see sweep_step_values[]) */
    int16_t internal_measurement; /* The value (average code) measured from DMA samples. */
} GAIN_CALC_TBL_ENTRY2_T;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*! *********************************************************************************
 * \brief  This function captures a set of digital samples of I and Q.
 *  An external buffer is required to store the samples captured into Packet RAM before processing.
 *
 * \param[in] buffer_ptr Pointer to the location for storing num_samples 32bit words for processing.
 * \param[in] sz_bytes Buffer size in bytes
 * \pre The calling routine must have triggered a RX warmup before calling this function.
 * \post This function enables AGC as a side effect.
 ***********************************************************************************/
void dbg_ram_dc_capture(int16_t *buffer_ptr, uint16_t sz_bytes);

/*! *********************************************************************************
 * \brief  This function calculates the average (DC value) based on a set of digital samples of I and Q.
 *  An external buffer is required to store the samples captured into Packet RAM before processing.
 *
 * \param[in] i_avg Pointer to the location for storing the calculated average for I channel samples.
 * \param[in] q_avg Pointer to the location for storing the calculated average for Q channel samples.
 * \param[in] buffer_ptr Pointer to the buffer containing the IQ samples. See dbg_ram_dc_capture function.
 * \param[in] num_samples The number of IQ pairs to capture for averaging
 * \return  0 if an error occurred, 1 if passed.
 ***********************************************************************************/
uint8_t rx_dc_sample_average(int16_t *i_avg, int16_t *q_avg, const int16_t *buffer_ptr, uint16_t num_samples);

#if defined(RADIO_IS_GEN_3P5)
/*!
 * @brief Function to sample the DC_EST register and return the average.
 *
 * This function samples the DC_EST register multiple times and performs an average to get an estimate
 * of the DC offset present in the signal.
 *
 * \param[out] i_avg Pointer to the location for storing the calculated average for I channel samples.
 * \param[out] q_avg Pointer to the location for storing the calculated average for Q channel samples.
 * \param[in] num_samples The number of samples to capture to calculate the average.
 * \return  0 if an error occurred, 1 if passed.
 */
uint8_t rx_dc_est_average(int16_t *i_avg, int16_t *q_avg, uint16_t num_samples);

#ifdef SUPPORT_IQ_DAC_TRIM
/*! *********************************************************************************
 * @brief  This function performs a trim of the BBA DCOC DAC on the DUT
 *
 * \return status - 1 if passed, 0 if failed.
 *
 * \details
 *   Requires the RX to be warmed up before this function is called.
 *
 ***********************************************************************************/
uint8_t rx_bba_dcoc_dac_trim_shortIQ(void);
#endif /* defined SUPPORT_IQ_DAC_TRIM */

/*! *********************************************************************************
 * @brief  This function performs a trim of the BBA DCOC DAC on the DUT
 *
 * \return status - 1 if passed, 0 if failed.
 *
 * \details
 *   Requires the RX to be warmed up before this function is called.
 *   Must be performed before performing the ::DCOC_DAC_INIT_Cal() trim.
 *
 ***********************************************************************************/
uint8_t rx_bba_dcoc_dac_trim_DCest(void);

/*! *********************************************************************************
 * @brief  This function performs a slope-sign seek trim on the DAC INIT
 *
 * \param[in] standalone_operation - boolean parameter indicating whether operation should be
 *  standalone or not. If standalone then RX warmup and warmdown are called internally.
 * \return status - 1 if passed, 0 if failed.
 *
 * \details
 *   Requires the RX to be warmed up before this function is called.
 *   Must be performed after the ::rx_bba_dcoc_dac_trim_DCest() trim.
 *
 ***********************************************************************************/
void DCOC_DAC_INIT_Cal(bool standalone_operation);

#endif /* !defined(RADIO_IS_GEN_3P5) */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN < 400)
#if defined(GCOV_DO_COVERAGE) /* routine is local except when testing code coverage */
/*! *********************************************************************************
 * @brief  This function calculates TZA steps and reciprocals for DCOC DAC trim
 *
 * \param[in] temp_step - The temporary step value to start the calculation.
 * \param[in] bbf_dcoc_step - The bbf_dcoc_step value to start the calculation.
 * \param[in] tza_dcoc_step - The array to be updated by the calculation (11 elements long).
 * \return status - 1 if passed, 0 if failed.
 *
 * \details
 *   Requires the RX to be warmed up before this function is called.
 *   Must be performed after the ::rx_bba_dcoc_dac_trim_DCest() trim.
 *
 ***********************************************************************************/
uint8_t calc_tza_step_recip(float32_t temp_step, uint32_t bbf_dcoc_step, TZAdcocstep_t tza_dcoc_step[]);

/*! *********************************************************************************
 * @brief  This function checks for DC on I and Q below the limits
 *
 * \param[in] i_dc - The measured DC on I channel.
 * \param[in] q_dc - The measured DC on I channel.
 * \return status - true if Dc on either channel is still too high, false if DC on both channels is low enough.
 *
 * \details
 *   Used by DCOC_DAC_INIT_Cal().
 *
 ***********************************************************************************/
bool dc_is_too_high(int16_t i_dc, int16_t q_dc);
#endif /* !defined(GCOV_DO_COVERAGE) */
#endif /* !defined(RADIO_IS_GEN_4P0) && !defined(RADIO_IS_GEN_4P5) */

/*! *********************************************************************************
 * @brief  This function performs a pulse on the RX init
 *
 * \details
 *   Used during some trim functions to support propagation of new settings.
 *
 ***********************************************************************************/
void force_rx_init_pulse(void);

#if defined(RADIO_IS_GEN_3P5)
/*! *********************************************************************************
 * \brief  This function performs the initialization for IQMC calibration process on the DUT
 *
 * \param [in] dcoc_ctrl_3_val - the setting to applied to DCOC_CTRL_3 register for manual DCOC correction
 *
 * \ingroup PublicAPIs
 *
 * \details
 *   Must be called prior to DCOCNull() routine.
 *
 ***********************************************************************************/
xcvrTrimStatus_t IQMCCalInit(uint32_t dcoc_ctrl_3_val, radio_mode_t radio_mode);
#else
/*! *********************************************************************************
 * \brief  This function performs the initialization for IQMC calibration process on the DUT
 *
 * \param [in] generic_channel_num - the channel number in the Generic FSK channel map to be used
 * for IQMC process.
 *
 * \ingroup PublicAPIs
 *
 * \details
 * AGC_OVRD register is corrupted by the IQMC process.
 *
 ***********************************************************************************/
xcvrTrimStatus_t IQMCCalInit(uint8_t generic_channel_num);
#endif /* defined(RADIO_IS_GEN_3P5) */

/*! *********************************************************************************
 * \brief  This function performs the IQMC calibration process on the DUT
 *
 * \param[out] iqmc_cal_ptr - pointer to the location where the calculated trim value
 *   should be stored..
 *
 * \return status of the operation, whether the calibration was successful or failed.
 *
 * \ingroup PublicAPIs
 *
 * \details
 *   Setup the RF signal generator to generate a -50dBm RF carrier with a
 *   300kHz offset to the channel frequency (2.44GHz channel).
 *
 ***********************************************************************************/
xcvrTrimStatus_t IQMCCal(uint32_t *iqmc_cal_ptr);

/*! *********************************************************************************
 * \brief  This function finalizes the IQMC calibration process on the DUT
 *
 * \param[in] iqmc_trim_reg_value - value to be written to the IQMC trim register.
 *
 * \ingroup PublicAPIs
 *
 * \details
 *   The IQMC calibration value is stored in the appropriate register to enable
 *   further calibration routines. Settings may be undone if they could impact other
 *   calibration routines.
 *
 ***********************************************************************************/
void IQMCFinalize(uint32_t iqmc_trim_reg_value);

#if defined(GCOV_DO_COVERAGE) /* routine is local except when testing code coverage */
/*! *********************************************************************************
 * \brief  This function runs a set of trials for the IQMC calibration process on the DUT
 *
 * \param[inout] trials_data - structure containing the measurements taken during the trials.
 * \param[in] num_trials - the number of trials to run. Must match the size of the arrays in the structure.
 *
 * \ingroup PublicAPIs
 *
 * \details
 *   The IQMC calibration trials store the gain and phase calibrations for each trial as well as a running
 *   sum of each of these values. These are later used for calculating the final trim (and for debug).
 *
 ***********************************************************************************/
void IQMCRunTrials(xcvr_iqmc_trials_t *trials_data, uint8_t num_trials);

/*! *********************************************************************************
 * \brief  This function calculates the results for a set of trials for the IQMC calibration process on the DUT
 *
 * \param[inout] trials_data - pointer to a structure containing the measurements taken during the trials.
 * \param[in] num_trials - the number of trials to run. Must match the size of the arrays in the structure.
 * \param[inout] iqmc_cal_ptr - pointer to a structure to contain the final trim results (for storage).
 *
 * \ingroup PublicAPIs
 *
 * \details
 *   The IQMC calibration trials store the gain and phase calibrations for each trial as well as a running
 *   sum of each of these values. These are later used for calculating the final trim (and for debug).
 *
 ***********************************************************************************/
xcvrTrimStatus_t IQMCCalcResult(const xcvr_iqmc_trials_t *trials_data, uint8_t num_trials, uint32_t *iqmc_cal_ptr);
#endif /* defined(GCOV_DO_COVERAGE) */

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* FSL_XCVR_TRIM_H */
