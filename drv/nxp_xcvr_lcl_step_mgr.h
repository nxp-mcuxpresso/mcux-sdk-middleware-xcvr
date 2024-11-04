/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef NXP_XCVR_LCL_STEP_MGR_H
/* clang-format off */
#define NXP_XCVR_LCL_STEP_MGR_H
/* clang-format on */

#include "fsl_common.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_lcl_step_structs.h"
#include "nxp_xcvr_lcl_ctrl.h"

/*!
 * @addtogroup xcvr_localization Localization Routines
 * @{
 */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) && (RF_OSC_26MHZ == 0) /* RSM step manager is new on Gen 4.7 radios & with 32MHZ RF_OSC */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

 /* ***** Structures to define step configuration types ***** */
/*!
 * @brief Mode 0, 1 & 3 Channel Sounding step configuration structure
 * This structure is used to store the configuration for a Mode 0, 1 or 3 step type . It has a variable length payload body portion that MUST be handled by firmware.
 */
typedef struct
{
    COM_MODE_CFG_HDR_Type header; /*!< The header describing a Mode 0/1/3 step. [Common header used for all step types]  */
    PAYLOAD_CFG_BODY_Type payload_body; /*!< The payload body for a Mode 0/1/3 step. [Common body used for Mode 0, 1, & 3 steps]  The size of this structure is the MAXIMUM size occupied by the payload body. Firmware MUST manage the actual 
                                                                            size of the payload body to allow subsequent steps to be located immediately after the used portion of the payload body structure. */
} xcvr_lcl_mode_0_1_3_cfg_t;

/*!
 * @brief Mode 2 Channel Sounding step configuration structure
 * This structure is used to store the configuration for a Mode 2 step type. It has a fixed length and consists of only the common step header structure.
 */
typedef struct
{
    COM_MODE_CFG_HDR_Type header; /*!< The header describing a Mode 2 step. */
} xcvr_lcl_mode_2_cfg_t;


 /* ***** Structures to define step response types ***** */
/*!
 * @brief Mode 0 & 1 Channel Sounding step result structure
 * This structure is used to store the results for Mode 0 & 1 step types. It has a fixed length and consists of a common result header plus an RTT result body.
 */
typedef struct
{
    COM_RES_HDR_Type header;    /*!< The result header for a Mode 0 & 1  step. [Common result header used for all steps] */
    COM_MODE_013_RES_BODY_Type rtt_result_body;  /*!< The RTT result body for a Mode 0, 1, & 3 step. [Common body used for Mode 0, 1, & 3 steps] */
} xcvr_lcl_mode_0_1_res_t;

/*!
 * @brief Mode 2 Channel Sounding step result structure
 * This structure is used to store the results for Mode 2 step type. It has a variable length (in the IQ portion) and that MUST be handled by firmware.
 */
typedef struct
{
    COM_RES_HDR_Type header;    /*!< The result header for a Mode2  step. [Common result header used for all steps] */
    IQ_RES_BODY_Type iq_result_body; /*!< The result body portion for IQ results (PCT) step. [Common body used for Mode 2 & 3 steps] */
} xcvr_lcl_mode_2_res_t;

/*!
 * @brief Mode 3 Channel Sounding step result structure
 * This structure is used to store the results for Mode 3 step type. It has a variable length (in the IQ portion) and that MUST be handled by firmware.
 */
typedef struct
{
    COM_RES_HDR_Type header;    /*!< The result header for a Mode 0 & 1  step. [Common result header used for all steps] */
    COM_MODE_013_RES_BODY_Type rtt_result_body;  /*!< The RTT result body for a Mode 0, 1, & 3 step. [Common body used for Mode 0, 1, & 3 steps] */
    IQ_RES_BODY_Type iq_result_body; /*!< The result body portion for IQ results (PCT) step. [Common body used for Mode 2 & 3 steps] */
} xcvr_lcl_mode_3_res_t;

/*Structure with information to configure steps for a CS event */
/*! @brief  CS subevent information storage. */
typedef struct 
{
    uint8_t num_steps;              /*!< Number of steps represented by this structure. */
    XCVR_RSM_FSTEP_TYPE_T * step_type; /*!< List of step types. */
    uint32_t * aa_list;                 /*!< List of access addresses for init and refl roles; Assumed to be interleaved with init first and refl second. */
    uint32_t * payload_list_init; /*!< List of payload words for initiator; Assumed the correct length and sufficient for all steps. */
    uint32_t * payload_list_refl;/*!< List of payload words for reflector; Assumed the correct length and sufficient for all steps. */
    uint16_t * channel_list;         /*!< List of channel index (0..79) for the steps. */
    uint16_t * cfo_list;                /*!< List of CFO values for the steps. */
    uint16_t * phase_add_list;         /*!< List of phase_adder values for the steps. */
    uint16_t * hpm_cal_list;         /*!< List of HPM CAL values for the steps. */
    uint16_t * step_cfg_list;        /*!< List of values for step configuration, as output by XCVR_LCL_MakeStepCfg(). Partially redundant with step_type above but keeping both for simplicity/speed. Must be 1 entry for every step. */
    XCVR_RSM_RTT_TYPE_T rtt_type;  /*!< RTT type for all Mode 0/1/3 steps in this entire sequence. */
    uint8_t num_ap;                         /*!< Number of antenna paths (not including tone extension period). */
    bool is_sniffer_mode;           /*!< Set to true if RSM is in sniffer mode (results are doubled) */
    bool phy_test_mode;             /*!< Forces AA and payload values to specific values for PHY test cases */
} cs_subevent_info_t;

/* Structure with information about the PKT RAM configuration and result storage */
/*! @brief  CS PKT RAM configuraiton and result storage info. */
typedef struct 
{
    PKT_RAM_BANK_SEL_T config_pkt_ram_bank; /*!< Which packet RAM bank stores configurations. */
    PKT_RAM_BANK_SEL_T result_pkt_ram_bank; /*!< Which packet RAM bank stores results. */
    uint16_t config_base_addr_word;         /*!< Word address (index) for the configuration buffer base. Indexes into the above selected packet RAM bank by words to */
    uint16_t result_base_addr_word;         /*!< Word address (index) for the result buffer base. */
    uint16_t config_depth_word;         /*!< Word depth (size) for the configuration buffer. */
    uint16_t result_depth_word;         /*!< Word depth (size) for the result buffer. */
    uint8_t interrupt_step_count;   /*!< Count of steps before interrupt should be produced. Repeats every count # of steps. */
    bool is_sniffer_mode;               /*!< True means configuration is for sniffer mode operation. Affects handling of results (2x the results are produced in sniffer mode) */
} cs_pkt_ram_config_info_t;


extern cs_subevent_info_t subevent_info;
extern const uint8_t rtt_payload_sizes[7];


#if defined(__cplusplus)
    extern "C" {
#endif



/*******************************************************************************
 * API
 ******************************************************************************/
 #define USE_LCL_STEP_MACROS (1)
 #if defined(USE_LCL_STEP_MACROS) && (USE_LCL_STEP_MACROS==1)
/*!
 * @brief Macro to configure (partially) settings for a Channel Sounding step.
 *
 * This macro configures a subset of the settings for a Channel Sounding step configuration. Every step type has an identical configuration of bitfields in the
 * second uint16_t portion of the step configuration so this routine sets up the common settings.
 *
 * @param[in] XCVR_RSM_FSTEP_TYPE_T step_format The step type for this step (Mode 0/1/2/3)..
 * @param[in] XCVR_RSM_T_PM_FM_SEL_T t_pm_sel The phase measurement time selection.
 * @param[in] uint8_t ant_permut The antenna permutation index.
 * @param[in] uint8_t t_pm_sel The Tone Extension setting.
 * @param[in] uint8_t oneway One Way ranging mode.
 * @param[in] uint8_t ant_cs_sync The antenna selection for PN transmission/reception.
 *
 * @return The 16bit value for use in the subsequent header creation calls.
 *
 */
#define  XCVR_LCL_MakeStepCfg(step_format, t_pm_sel, ant_permut, tone_ext, oneway, ant_cs_sync) \
            ((uint16_t)(COM_MODE_013_CFG_HDR_STEP_CFG_MODE(step_format) | \
             COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT(tone_ext) | \
             COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT(ant_permut) | \
             COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC(ant_cs_sync) | \
             COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY(oneway) | \
             COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL(t_pm_sel)))

/*!
 * @brief Macro to configure the common (all modes) step header structure.
 *
 * This macro configures the settings for a Channel Sounding step configuration header. Every step type has an identical configuration of bitfields in the
 * header of the step configuration so this routine sets up the common settings.
 *
 * @param[out] (xcvr_lcl_mode_0_1_3_cfg_t *)cfg_entry Pointer to the location to store the configuration structure (typically in PKT RAM).
 * @param[in] uint16_t channel_num the channel number to be applied. This gets converted to a HOP_TBL_CFG_OVRD format in the macro.
 * @param[in] uint16_t ctune the ctune value to be applied.
 * @param[in] uint16_t step_cfo The CFO adjustment to be applied..
 * @param[in] uint16_t hpm_cal the HPM CAL value to be applied.
 * @param[in] uint16_t phase_add The phase adder to include on this step.
 * @param[in] uint16_t step_cfg The step configuration value created by ::XCVR_LCL_MakeStepCfg().
 *
 * @note This macro can be used in two different ways. It can be used with cfg_entry pointer pointing to the location in PKT RAM where data 
 * needs to be written for use by RSM for CONFIGS. In this usage, it performs 6 writes (16 bit each) to the PKT RAM. The other use case is 
 * to call it with cfg_entry pointing to a static variable in system RAM. In this case, it writes 6x (16bit) to the system RAM and a later macro (XCVR_LCL_WriteCommonHeader)
 * needs to be used to perform 3 writes to PKT RAM (32 bit each). The system RAM variable should be of type COM_MODE_CFG_HDR_UNION_Type.
 */
#define XCVR_LCL_MakeCommonHeader(cfg_entry, channel_num, ctune, step_cfo, hpm_cal, phase_add, step_cfg) \
    do { \
         (cfg_entry)->header.STEP_CFG = (uint16_t)(step_cfg); /* This is the output of XCVR_LCL_MakeStepCfg() function */ \
         uint16_t mapped_channel; \
         MAKE_MAPPED_CHAN_OVRD2((channel_num), mapped_channel); /* Maps to proper format for RSM channels */ \
         (cfg_entry)->header.CHANNEL_NUM = mapped_channel; /* Entry is full 16 bits wide so no need to mask/shift */ \
         (cfg_entry)->header.STEP_CFO = (step_cfo); /* Entry is full 16 bits wide so no need to mask/shift */ \
         (cfg_entry)->header.HPM_CAL_FACTOR = COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR(hpm_cal>>1U); /*  Field stores the 12MSBs of HPM_CAL_FACTOR: HPM_CAL_FACTOR[12:1] */ \
         (cfg_entry)->header.CTUNE_MANUAL = COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL(ctune); \
         (cfg_entry)->header.PHASE_ADD = COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD(phase_add); } \
     while (0)

#if (1) /* use this option if the pointers passed into the macro are not incrementable */
#define XCVR_LCL_WriteCommonHeader(pkt_ram_p, sys_ram_p) \
    do { \
        (pkt_ram_p)->U32_data[0] = (sys_ram_p)->U32_data[0]; \
        (pkt_ram_p)->U32_data[1] = (sys_ram_p)->U32_data[1]; \
        (pkt_ram_p)->U32_data[2] = (sys_ram_p)->U32_data[2]; } \
     while (0)
#else /* use this option if the pointers passed in are incrementable */
#define XCVR_LCL_WriteCommonHeader(pkt_ram_p, sys_ram_p) \
    do { \
        *pkt_ram_p++ = *sys_ram_p++; \
        *pkt_ram_p++ = *sys_ram_p++; \
        *pkt_ram_p++ = *sys_ram_p++; } \
     while (0)
#endif

/*!
 * @brief Macro to configure only the CFO in common (all modes) step header structure.
 *
 * This macro configures the CFO setting for a Channel Sounding step configuration header. It is used to update previously programmed 
 * PKT RAM contents with a CFO value once it is calculated and available.
 *
 * @param[out] (xcvr_lcl_mode_0_1_3_cfg_t *)cfg_entry Pointer to the location to store the cCFO value (typically in PKT RAM).
 * @param[in] uint16_t step_cfo The CFO adjustment to be applied..
 *
 * @note This macro can be used in two different ways. It can be used with cfg_entry pointer pointing to the location in PKT RAM where data 
 * needs to be written for use by RSM for CONFIGS. In this usage, it performs 1 write (16 bit each) to the PKT RAM. The other use case is 
 * to call it with cfg_entry pointing to a static variable in system RAM. In this case, it writes 1x (16bit) to the system RAM and a later macro (XCVR_LCL_WriteCommonHeader)
 * needs to be used to perform 3 writes to PKT RAM (32 bit each). The system RAM variable should be of type COM_MODE_CFG_HDR_UNION_Type.
 */#define XCVR_LCL_UpdateHeaderCfo(cfg_entry, step_cfo) \
    do { \
         (cfg_entry)->header.STEP_CFO = (step_cfo);  } /* Entry is full 16 bits wide so no need to mask/shift */ \
     while (0)

/*!
 * @brief Macro to configure the Mode 0/1/3 step payload structure.
 *
 * This macro configures the payload contents (counting AA as part of payload for a Channel Sounding step configuration header. 
 *
 * @param[out] (xcvr_lcl_mode_0_1_3_cfg_t *)cfg_entry Pointer to the location to store the configuration structure (typically in PKT RAM).
 * @param[in] uint32_t AA_INIT The access addres to be used by the initiator in this step.
 * @param[in] uint32_t AA_REFL The access addres to be used by the reflector in this step.
 * @param[in] uint8_t num_payload_words Number of words in the payload, can range from 0 to 4. Exceeding this is not checked and can cause serious problems.
 * @param[in] uint32_t payload_init Pointer to an array of up to 4 words of payload to be used for the intiator payload.
 * @param[in] uint32_t payload_refl Pointer to an array of up to 4 words of payload to be used for the reflector payload.
 *
 * @note This macro should be used in with cfg_entry pointer pointing to the location in PKT RAM where the base of the configuration header is located. This is
 * the identical pointer that would be used in XCVR_LCL_MakeCommonHeader or XCVR_LCL_WriteCommonHeader macros. This macro always writes the 2 AAs 
 * then, if num_payload_words >0, writes some number of payload words as well. Payloads are interleaved on a word-by-word basis.
 */
#define XCVR_LCL_MakeMode013_Payload(cfg_entry, aa_init, aa_refl, num_payload_words, payload_init_ptr, payload_refl_ptr) \
    do { \
        (cfg_entry)->payload_body.AA_INIT = (aa_init); \
        (cfg_entry)->payload_body.AA_REFL = (aa_refl); \
        volatile uint32_t * payload_ptr = &((cfg_entry)->payload_body.PAYLOAD[0].INIT); \
        for (uint8_t words = 0U;words<(num_payload_words);words++) \
        { \
            *payload_ptr++ = *(payload_init_ptr)++; /* Increments the pointer that is passed in */ \
        } \
         for (uint8_t words2 = 0U;words2<(num_payload_words);words2++) \
        { \
            *payload_ptr++ = *(payload_refl_ptr)++;  /* Increments the pointer that is passed in */ \
        } \
    } \
     while (0)

/*!
 * @brief Macro to unpack the NADM portion of Mode 0/1/3 step result structure COM_MODE_013_RES_BODY.
 *
 * This macro unpacks the NADM result contents for a COM_MODE_013_RES_BODY structure NADM_ERROR_RSSI field. 
 *
 * @param[in] uint32_t nadm_result_packed The 32bit word from the NADM_ERROR_RSSI result field.
 * @param[out] uint8_t rssi_nb The narrowband RSSI value from this step.
 * @param[out] uint8_t nadm_fm_corr_value The FM correlation value from this step.
 * @param[out] uint8_t nadm_fm_corr_valid 1 indicates the FM correlation is valid; 0 indicates invalid.
 * @param[out] uint8_t nadm_fm_symb_err_value Count of symbol errors detected in the RTT payload.
 * @param[out] uint8_t nadm_fm_symb_err_valid 1 indicates the symbol error count is valid; 0 indicates invalid.
 * @param[out] uint16_t nadm_pdelta difference in correlations around the max correlator instance.
 *
 * @note This macro should be used with res_entry pointer pointing to the location in PKT RAM where the base of the result header is located. 
 */
#define XCVR_LCL_Unpack013_NADM_Result(nadm_result_packed, rssi_nb, nadm_fm_corr_value, nadm_fm_corr_valid, nadm_fm_symb_err_value, nadm_fm_symb_err_valid, nadm_pdelta) \
    do { \
        rssi_nb = ((nadm_result_packed&COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_SHIFT); \
        nadm_fm_corr_value = ((nadm_result_packed&COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_SHIFT); \
        nadm_fm_corr_valid = ((nadm_result_packed&COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID_SHIFT); \
        nadm_fm_symb_err_value = ((nadm_result_packed&COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE_SHIFT); \
        nadm_fm_symb_err_valid = ((nadm_result_packed&COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID_SHIFT); \
        nadm_pdelta = ((nadm_result_packed&COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE_MASK)>>COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE_SHIFT); \
    } \
     while (0)

/*!
 * @brief Macro to calculate the NADM metric from the FM_CORR value reported in COM_MODE_013_RES_BODY.
 *
 * This macro calculates the NADM metric for a COM_MODE_013_RES_BODY structure NADM_ERROR_RSSI field. 
 *
 * @param[in] uint8_t fm_corr_value The 8 bit FM_CORR value from the NADM_ERROR_RSSI result field.
 * @param[in] uint8_t datarate for the receive being processed.
 * @param[out] uint8_t nadm_metric The metric, ranging from 0 to 6 for valid inputs. 0xFF will be reported for zero FM_CORR input.
 *
 */
#define TGT_FM_CORR_1MBPS    (211)
#define NADM_METRIC_DIV_1MBPS (5)
#define TGT_FM_CORR_2MBPS    (255)
#define NADM_METRIC_DIV_2MBPS (8)
#define XCVR_LCL_CalcNadmMetric(fm_corr_value, datarate, nadm_metric) \
    do { \
        nadm_metric = 0xFFU; \
        if (fm_corr_value > 0U) \
        { \
            int16_t temp_calc = (int16_t)fm_corr_value; /* Working in signed values to handle subtract below zero. */ \
            if (datarate == XCVR_RSM_RATE_1MBPS) \
            { \
                temp_calc = (TGT_FM_CORR_1MBPS-temp_calc)/NADM_METRIC_DIV_1MBPS; /* Caculate metric result */ \
            } \
            else \
            { \
                temp_calc = (TGT_FM_CORR_2MBPS-temp_calc)/NADM_METRIC_DIV_2MBPS; /* Caculate metric result */ \
            } \
            if (temp_calc < 0) /* Saturate anything below zero to zero */ \
            { \
                nadm_metric = 0U; \
            } \
            else  \
            { \
                if (temp_calc > 6) /* Saturate anything above 6 to 6 */  \
                { \
                    nadm_metric = 6; \
                } \
                else \
                { \
                    nadm_metric = (uint8_t) temp_calc; \
                } \
            } \
        }\
    } \
     while (0)


/*!
 * @brief Macro to unpack the RTT portion of Mode 0/1/3 step result structure COM_MODE_013_RES_BODY.
 *
 * This macro unpacks the RTT result contents for a COM_MODE_013_RES_BODY structure NADM_ERROR_RSSI field. 
 *
 * @param[in] uint32_t rtt_result_packed The 32bit word from the RTT_RESULT result field.
 * @param[out] uint8_t rtt_valid 1 indicates the RTT is valid; 0 indicates invalid.
 * @param[out] uint8_t rtt_found 1 indicates the RTT is found; 0 indicates not found.
 * @param[out] uint16_t rtt_cfo The CFO estimate reported from the RTT step.
 * @param[out] uint8_t rtt_ham_dist_sat The computed Hamming distance, saturated to 2 bits.
 * @param[out] int8_t rtt_int_adj The integer adjustment of timing .
 * @param[out] uint16_t rtt_p_delta Difference between the squared correlation magnitude values from the RTT process.
 * @param[in]  XCVR_RSM_SQTE_RATE_T Datarate currently used.
 */
#define XCVR_LCL_Unpack013_RTT_Result(rtt_result_packed, rtt_valid, rtt_found, rtt_cfo, rtt_ham_dist_sat, rtt_int_adj, rtt_p_delta, datarate) \
    do { \
        rtt_valid = (uint8_t)((rtt_result_packed&COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_MASK)>>COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_SHIFT); \
        rtt_found = (uint8_t)((rtt_result_packed&COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_MASK)>>COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_SHIFT); \
        int8_t cfo_div = 4; /* used in integer math to create 15.25 multiplier for raw CFO conversion at 1Mbps */ \
        if (datarate == XCVR_RSM_RATE_2MBPS) \
        { \
            cfo_div = 2; /* used in integer math to create 30.5 multiplier for raw CFO conversion at 2Mbps */ \
        } \
        uint32_t temp; \
        int32_t temp_cfo; \
        temp = ((rtt_result_packed&COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_MASK)>>COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_SHIFT); \
        temp_cfo = (int16_t)temp; /* packed data is signed 16 bits, then work in signed 32 bits for conversion to Hz */ \
        temp_cfo = (temp_cfo * 61) / cfo_div; /* 15.25Hz (or 30.5Hz) per LSB, implement in integer math as divide by 4 (or 2) and multiply by 61 */ \
        rtt_cfo = (int16_t)(temp_cfo); \
        rtt_ham_dist_sat = (uint8_t)((rtt_result_packed&COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT_MASK)>>COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT_SHIFT); \
        uint8_t temp_adj; \
        temp_adj = (uint8_t)((rtt_result_packed&COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ_MASK)>>COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ_SHIFT); \
        rtt_int_adj = ((int8_t)(temp_adj<<6))>>6; /* extend sign */ \
        rtt_p_delta = (uint16_t)((rtt_result_packed&COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA_MASK)>>COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA_SHIFT); \
    } \
     while (0)

/*!
 * @brief Macro to unpack the CFO portion of Mode 0/1/3 step result structure COM_MODE_013_RES_BODY.
 *
 * This macro unpacks the CFO result contents for a COM_MODE_013_RES_BODY structure CFO_EST field. 
 *
 * @param[in] uint32_t cfo_est The 32bit word from the CFO_EST result field.
 * @param[out] int16_t cfo_signed Signed representation of CFO_EST.
 *
 */
#define XCVR_LCL_Unpack013_CFO_Result(cfo_est, cfo_signed) \
    do { \
        uint16_t temp_cfo = (uint16_t)( (cfo_est&COM_MODE_013_RES_BODY_CFO_EST_CFO_EST_MASK)>>COM_MODE_013_RES_BODY_CFO_EST_CFO_EST_SHIFT); \
        temp_cfo = (((temp_cfo&0x200U) == 0U )? temp_cfo : temp_cfo | 0xFC00); /* sign extend within the 16bits unsigned storage */ \
        cfo_signed = (int16_t)(temp_cfo); \
        } \
     while (0)

 #else
/*!
 * @brief Function to configure (partially) settings for a Channel Sounding step.
 *
 * This function configures a subset of the settings for a Channel Sounding step configuration. Every step type has an identical configuration of bitfields in the
 * second uint16_t portion of the step configuration so this routine sets up the common settings.
 *
 * @param[in] step_format The step type for this step (Mode 0/1/2/3)..
 * @param[in] t_pm_sel The phase measurement time selection.
 * @param[in] ant_permut The antenna permutation index.
 * @param[in] t_pm_sel The Tone Extension setting.
 * @param[in] oneway One Way ranging mode.
 * @param[in] ant_cs_sync The antenna selection for PN transmission/reception.
 *
 * @return The 16bit value for use in the subsequent header creation calls.
 *
 * @note Function is specified inline in the header to ensure it is inline always (for speed)
 */
uint16_t XCVR_LCL_MakeStepCfg(XCVR_RSM_FSTEP_TYPE_T step_format,
                                   XCVR_RSM_T_PM_FM_SEL_T t_pm_sel,
                                   uint8_t ant_permut,
                                   uint8_t tone_ext,
                                   uint8_t oneway,
                                   uint8_t ant_cs_sync);

/*!
 * @brief Function to configure header settings for a Mode 0/1/3 Channel Sounding step.
 *
 * This function configures header settings for a Mode 0/1/3 Channel Sounding step configuration. The ::XCVR_LCL_MakeStepCfg() routine is used to 
 * create the step_cfg input. Not all inputs are meaningful in certain RSM configurations.
 *
 * @param[out] cfg_entry Pointer to the location to store the configuration structure (typically in PKT RAM).
 * @param[in] channel_num the channel number to be applied.
 * @param[in] ctune the ctune value to be applied.
 * @param[in] step_cfo The CFO adjustment to be applied..
 * @param[in] hpm_cal the HPM CAL value to be applied.
 * @param[in] phase_add The phase adder to include on this step.
 * @param[in] step_cfg The step configuration value created by ::XCVR_LCL_MakeStepCfg().
 * @param[in] access_address Pointer to a list of 2 access addresses, first half AA and second half AA (in that order).
 *
 * @return The status of the header configuration process.
 *
 * @note Function is specified inline in the header to ensure it is inline always (for speed)
 */
xcvrLclStatus_t XCVR_LCL_MakeMode013CfgHeader(xcvr_lcl_mode_0_cfg_t *cfg_entry,
                                   uint16_t channel_num,
                                   uint16_t ctune,
                                   uint16_t step_cfo,
                                   uint16_t hpm_cal,
                                   uint16_t phase_add,
                                   uint16_t step_cfg,
                                   uint32_t * access_address);

/*!
 * @brief Function to configure payload contents for a Mode 1/3 Channel Sounding step.
 *
 * This function configures payload contents for a Mode 1/3 Channel Sounding step configuration. Only the used payload words are configured, unused words
 * are not written by this routine. 
 *
 * @param[out] cfg_entry Pointer to the location to store the configuration structure (typically in PKT RAM).
 * @param[in] rtt_type The RTT sequence type.
 * @param[in] random_payload1 Pointer to the random payload words. Not used when payload is channel sounding sequence.
 * @param[in] random_payload2 Pointer to the random payload words. Not used when payload is channel sounding sequence.
 *
 * @return The status of the payload configuration process.
 *
 * @note Function is specified inline in the header to ensure it is inline always (for speed)
 */
xcvrLclStatus_t XCVR_LCL_MakeMode13CfgPayload(xcvr_lcl_mode_1_3_cfg_t *cfg_entry,
                                   XCVR_RSM_RTT_TYPE_T rtt_type,
                                   uint32_t * random_payload1,
                                   uint32_t * random_payload2);


/*!
 * @brief Function to configure header settings for a Mode 2 Channel Sounding step.
 *
 * This function configures a header settings for a Mode 2 Channel Sounding step configuration. The ::XCVR_LCL_MakeStepCfg() routine is used to 
 * create the step_cfg input. Not all inputs are meaningful in certain RSM configurations.
 *
 * @param[out] cfg_entry Pointer to the location to store the configuration structure (typically in PKT RAM).
 * @param[in] channel_num the channel number to be applied.
 * @param[in] ctune the ctune value to be applied.
 * @param[in] step_cfo The CFO adjustment to be applied..
 * @param[in] hpm_cal the HPM CAL value to be applied.
 * @param[in] step_cfg The step configuration value created by ::XCVR_LCL_MakeStepCfg().
 *
 * @return The status of the header configuration process.
 *
 * @note Function is specified inline in the header to ensure it is inline always (for speed)
 */
xcvrLclStatus_t XCVR_LCL_MakeMode2CfgHeader(xcvr_lcl_mode_2_cfg_t *cfg_entry,
                                   uint16_t channel_num,
                                   uint16_t ctune,
                                   uint16_t step_cfo,
                                   uint16_t hpm_cal,
                                   uint32_t step_cfg);
#endif /* defined(USE_LCL_STEP_MACROS) && (USE_LCL_STEP_MACROS==1) */

/*!
 * @brief Function to calculate config and result sizes for a single Channel Sounding step.
 *
 * This function takes in information about a single Channel Sounding step and calculates the number of 32bit words in both the config and the result steps. This 
 * information is used to update the PKT RAM index and determine when double buffers roll over.
 *
 * @param[in] step_format The format of the step.
 * @param[in] rtt_type The RTT type for the step (must be constant throughout an entire sequence).
 * @param[in] num_ap The number of antenna paths in use.
 * @param[out] config_size_words The number of 32bit words in the config for the step.
 * @param[out] result_size_words The number of 32bit words in the result for the step.
 *
 * @return The status of the calculation (not the result but any error in input values).
 *
 * @note For a particular RTT Type and number of antenna paths combination, this routine can be called once for each step_format (0, 1, 2, 3) and the results stored in arrays locally at the caller. This can
 * optimize time when the same configuration is being used for a large number of steps.
 *
 */
xcvrLclStatus_t XCVR_LCL_CalcConfigResult_Size(XCVR_RSM_FSTEP_TYPE_T step_format, XCVR_RSM_RTT_TYPE_T rtt_type, uint8_t num_ap, uint8_t * config_size_words, uint8_t * result_size_words);


/*!
 * @brief Function to validate the information defined for a Channel Sounding subevent.
 *
 * This function takes in information about a complete Channel Sounding subevent and validates that various pointers are not NULL and the sequence will fit within the defined buffer size. It does not check this
 * buffer size can fit within a particular area of PKT RAM but just checks the sequence length (in words) is smaller than a defined size.
 *
 * @param[in] subevent_info_ptr Pointer to the information describing a subevent (all steps).
 * @param[in] pkt_ram_cfg_buffer_sz_words The number of words allocated to storage of configuration step data.
 * @param[in] pkt_ram_res_buffer_sz_words The number of words allocated to storage of result step data.
 *
 * @return The status of the validation check on the subevent info.
 *
 */
 xcvrLclStatus_t XCVR_LCL_ValidateSubeventInfo(cs_subevent_info_t * subevent_info_ptr, uint16_t pkt_ram_cfg_buffer_sz_words, uint16_t pkt_ram_res_buffer_sz_words);

/*!
 * @brief Function to program configuration steps into PKT RAM buffer for a complete subevent.
 *
 * This function takes in information about a complete Channel Sounding subevent and programs that information into PKT RAM at the pointed location. It assumes a signal buffering setup where the entire sequence 
 * is programmed to PKT RAM at the same time.
 *
 * @param[in] subevent_info_ptr Pointer to the information describing a subevent (all steps).
 * @param[out] pkt_ram_info_ptr pointer to the description data for the configuration and result buffers in PKT RAM. Only used if sys_ram_ptr is not NULL.
 * @param[out] sys_ram_ptr pointer to the location in system RAM for loading step configurations. If this is not NULL then the routine loads to system RAM.
 *
 * @return The status of the prorgamming process.
 *
 */
xcvrLclStatus_t XCVR_LCL_ProgramFstepRam(cs_subevent_info_t * subevent_info_ptr, cs_pkt_ram_config_info_t * pkt_ram_info_ptr, uint32_t * sys_ram_ptr);


/*!
 * @brief Function to initialize the configuration and result pointers for hardware use.
 *
 * This function takes in information about a the PKT RAM storage locations for a subevent and programs the hardware registers to be used 
 * for controlling the usage of PKT RAM by the RSM.
 *
 * @param[out] pkt_ram_info_ptr pointer to the description data for the configuration and result buffers in PKT RAM.
 *
 * @return The status of the prorgamming process.
 *
 */
 xcvrLclStatus_t XCVR_LCL_InitCfgResPointers(cs_pkt_ram_config_info_t * pkt_ram_info_ptr);

/*!
 * @brief Function to reset the configuration and result pointers for hardware use.
 *
 * This function resets some portions of the hardware registers to be used 
 * for controlling the usage of PKT RAM by the RSM. It is intended to be used when a sequence is programmed and has run once and must be run additional times.
 *
 * @return The status of the reset process.
 *
 */
xcvrLclStatus_t XCVR_LCL_ResetCfgResPointers(void);

/*!
 * @brief Function to read a step configuration from memory and calculate the number of words in the step.
 *
 * This function reads a step configuration from memory and calculates the length of the step (in words) and returns contents of the
 * step header
 *
 * @param[in] header_ptr Pointer to the location of the step configuration.
 * @param[out] step_format The step type for this step (Mode 0/1/2/3)..
 * @param[out] t_pm_sel The phase measurement time selection.
 * @param[out] ant_permut The antenna permutation index.
 * @param[out] tone_ext The Tone Extension setting.
 * @param[out] oneway One Way ranging mode.
 * @param[out] ant_cs_sync The antenna selection for PN transmission/reception.
 *
 * @return The number of words in the step.
 *
 */
uint8_t XCVR_LCL_ReadStepCfg(COM_MODE_013_CFG_HDR_Type * header_ptr,
                                    XCVR_RSM_FSTEP_TYPE_T * step_format,
                                   XCVR_RSM_T_PM_FM_SEL_T * t_pm_sel,
                                   uint8_t * ant_permut,
                                   uint8_t * tone_ext,
                                   uint8_t * oneway,
                                   uint8_t * ant_cs_sync);


/*!
 * @brief Function to load configuration steps to PKT RAM (from system RAM) in double buffering use cases.
 *
 * This function copies previously generated configuration steps from system RAM to PKT RAM. It copies up to <step count> number of steps
 * and handles when there are fewer steps left in the sequence.
 *
 * @return The status of the copy process.
 *
 * @pre The ::XCVR_LCL_InitCfgResPointers() moduole must have been called to initialize the pointers to PKT RAM and other subevent info.
 *
 */
xcvrLclStatus_t XCVR_LCL_LoadConfigSteps(uint32_t ** current_cfg_ptr);

/*!
 * @brief Function to read result steps from PKT RAM (to system RAM) in double buffering use cases.
 *
 * This function copies previously generated result steps from PKT RAM to system RAM. It copies up to <step count> number of results
 * and handles when there are fewer steps left in the sequence.
 *
 * @param[in] current_cfg_ptr Pointer to a pointer to the location of the step configuration.
 * @return The status of the copy process.
 *
 * @pre The ::XCVR_LCL_InitCfgResPointers() module must have been called to initialize the pointers to PKT RAM and other subevent info.
 *
 */
xcvrLclStatus_t XCVR_LCL_ReadResultSteps(uint32_t ** current_res_ptr);

/*!
 * @brief Function to perform checks on double buffering configuration info.
 *
 * This function performs checks on the information defining the PKT RAM buffers to ensure that they don't overlap, extend outside their PKT RAM
 * blocks, etc.
 *
 * @param[in] current_res_ptr Pointer to a pointer to the location of the step results.
 * @return The status of the verification process.
 *
 * @pre The ::XCVR_LCL_InitCfgResPointers() module must have been called to initialize the pointers to PKT RAM and other subevent info.
 *
 */
xcvrLclStatus_t XCVR_LCL_CheckPktRamCfg(cs_pkt_ram_config_info_t * pkt_ram_info_ptr);

/*!
 * @brief Function to handle double buffering processing and updates during either a STEP or EOS interrupt.
 *
 * This function handles the copy, pointer update, etc processing when double buffering is in use for the RSM config and results in PKT RAM. 
 * It writes/reads a number of configs and results equal to the step interrupt count and handles the end of sequence case where there may 
 * be fewer steps remaining (when an EOS interrupt is received rather than a step interrupt).
 *
 * @param[in] status_bits - the contents of XCVR_MISC->RSM_INT_STATUS indicating which interrupts are asserted.
 *
 * @return The status of the copy and update process.
 *
 * @pre The ::XCVR_LCL_InitCfgResPointers() module must have been called to initialize the pointers to PKT RAM and other subevent info.
 *
 * @note This routine is intended to be called from the RSM interrupt handler.
 *
 */
xcvrLclStatus_t XCVR_LCL_HandleIrqStepEos(int32_t status_bits);


xcvrLclStatus_t XCVR_LCL_SetupInitialConfigs(uint8_t total_num_steps, uint32_t * config_in_ptr, uint32_t * results_out_ptr, XCVR_RSM_RTT_TYPE_T rtt_type, uint8_t num_ap, bool sniffer_mode);

xcvrLclStatus_t XCVR_LCL_FinishFinalResults(void);








#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) && (RF_OSC_26MHZ == 0) */

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* NXP_XCVR_LCL_STEP_MGR_H */

