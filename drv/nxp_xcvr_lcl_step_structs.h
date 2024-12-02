/*
** ###################################################################
**     Processor:
**     Compilers:           Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Build:               b230824
**
**     Abstract:
**         CMSIS Peripheral Access Layer for nxp_xcvr_lcl_step_structs
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2024 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**
** ###################################################################
*/

/*!
 * @file nxp_xcvr_lcl_step_structs.h
 * @version 0.0
 * @date 0-00-00
 * @brief CMSIS Peripheral Access Layer for nxp_xcvr_lcl_step_structs
 *
 * CMSIS Peripheral Access Layer for nxp_xcvr_lcl_step_structs
 */

#ifndef NXP_XCVR_LCL_STEP_STRUCTS_H_
#define NXP_XCVR_LCL_STEP_STRUCTS_H_ /**< Symbol preventing repeated inclusion */

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */

/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
#if (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#else
#pragma push
#pragma anon_unions
#endif
#elif defined(__CWCC__)
#pragma push
#pragma cpp_extensions on
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma language = extended
#else
#error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- COM_MODE_013_CFG_HDR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COM_MODE_013_CFG_HDR_Peripheral_Access_Layer COM_MODE_013_CFG_HDR Peripheral Access Layer
 * @{
 */

// TODO: get new code generation from the CRR
/** COM_MODE_013_CFG_HDR - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CHANNEL_NUM;    /**< Channel number for the step., offset: 0x0 */
    __IO uint16_t STEP_CFG;       /**< Configuration for the step., offset: 0x2 */
    __IO uint16_t STEP_CFO;       /**< CFO setting for the step, offset: 0x4 */
    __IO uint16_t HPM_CAL_FACTOR; /**< HPM CAL factor to apply for the step., offset: 0x6 */
    __IO uint16_t CTUNE_MANUAL;   /**< CTUNE manual to inject for the step., offset: 0x8 */
    __IO uint16_t PHASE_ADD;      /**< Phase adder for the step., offset: 0xA */
    __IO uint32_t AA_INIT;        /**< Access address for the initiator (first half step)., offset: 0xC */
    __IO uint32_t AA_REFL;        /**< Access address for the reflector (second half step)., offset: 0x10 */
} COM_MODE_013_CFG_HDR_Type;

/* Manually created COMMON header type for all modes (to match updates in structure definitions */
typedef struct
{
    __IO uint16_t CHANNEL_NUM;    /**< Channel number for the step., offset: 0x0 */
    __IO uint16_t STEP_CFG;       /**< Configuration for the step., offset: 0x2 */
    __IO uint16_t STEP_CFO;       /**< CFO setting for the step, offset: 0x4 */
    __IO uint16_t HPM_CAL_FACTOR; /**< HPM CAL factor to apply for the step., offset: 0x6 */
    __IO uint16_t CTUNE_MANUAL;   /**< CTUNE manual to inject for the step., offset: 0x8 */
    __IO uint16_t PHASE_ADD;      /**< Phase adder for the step., offset: 0xA */
} COM_MODE_CFG_HDR_Type;

typedef union
{
    COM_MODE_CFG_HDR_Type header;
    uint32_t U32_data[3];
} COM_MODE_CFG_HDR_UNION_Type;

/* ----------------------------------------------------------------------------
   -- COM_MODE_013_CFG_HDR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COM_MODE_013_CFG_HDR_Register_Masks COM_MODE_013_CFG_HDR Register Masks
 * @{
 */

/*! @name CHANNEL_NUM - Channel number for the step. */
/*! @{ */

#define COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_MASK (0xFFFFU)
#define COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_SHIFT (0U)
/*! CHANNEL_NUM - Channel number for the step. */
#define COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_SHIFT)) & \
     COM_MODE_013_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_MASK)
/*! @} */

/*! @name STEP_CFG - Configuration for the step. */
/*! @{ */

#define COM_MODE_013_CFG_HDR_STEP_CFG_MODE_MASK (0x3U)
#define COM_MODE_013_CFG_HDR_STEP_CFG_MODE_SHIFT (0U)
/*! MODE - Mode for the step
 *  0b00..Frequency Compensation Step (Mode 0)
 *  0b01..Pk-Pk step (Mode 1)
 *  0b10..Tn-Tn step (Mode 2)
 *  0b11..Pk-Tn-Tn-Pk step (Mode 3)
 */
#define COM_MODE_013_CFG_HDR_STEP_CFG_MODE(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFG_MODE_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFG_MODE_MASK)

#define COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT_MASK (0xCU)
#define COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT_SHIFT (2U)
/*! TONE_EXT - Choice of tone extension for the initiator portion of the step.
 *  0b00..Tone extension is not present for either role TX.
 *  0b01..Tone extension is present for initiator TX only.
 *  0b10..Tone extension is present for reflector TX only.
 *  0b11..Tone extension is present for both roles TX.
 */
#define COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT_MASK)

#define COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_MASK (0x1F0U)
#define COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_SHIFT (4U)
/*! ANT_PERMUT - Antenna permutation pattern for the step. */
#define COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_MASK)

#define COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC_MASK (0x600U)
#define COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC_SHIFT (9U)
/*! ANT_CS_SYNC - Antenna cs Sync */
#define COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC_MASK)

#define COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY_MASK (0x800U)
#define COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY_SHIFT (11U)
/*! EP_ONE_WAY - One way ranging? */
#define COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY_MASK)

#define COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL_MASK (0x1000U)
#define COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL_SHIFT (12U)
/*! T_PM_SEL - Selection of T_PM choices. */
#define COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL_MASK)
/*! @} */

/*! @name STEP_CFO - CFO setting for the step */
/*! @{ */

#define COM_MODE_013_CFG_HDR_STEP_CFO_STEP_CFO_MASK (0xFFFFU)
#define COM_MODE_013_CFG_HDR_STEP_CFO_STEP_CFO_SHIFT (0U)
/*! STEP_CFO - CFO setting for the step. */
#define COM_MODE_013_CFG_HDR_STEP_CFO_STEP_CFO(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_STEP_CFO_STEP_CFO_SHIFT)) & \
     COM_MODE_013_CFG_HDR_STEP_CFO_STEP_CFO_MASK)
/*! @} */

/*! @name HPM_CAL_FACTOR - HPM CAL factor to apply for the step. */
/*! @{ */

#define COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_MASK (0xFFFU)
#define COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_SHIFT (0U)
/*! HPM_CAL_FACTOR - HPM CAL factor to apply for the step */
#define COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_SHIFT)) & \
     COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_MASK)
/*! @} */

/*! @name CTUNE_MANUAL - CTUNE manual to inject for the step. */
/*! @{ */

#define COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_MASK (0x1FFU)
#define COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_SHIFT (0U)
/*! CTUNE_MANUAL - CTUNE manual to inject for the step. */
#define COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_SHIFT)) & \
     COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_MASK)
/*! @} */

/*! @name PHASE_ADD - Phase adder for the step. */
/*! @{ */

#define COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD_MASK (0xFFU)
#define COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD_SHIFT (0U)
/*! PHASE_ADD - Phase adder for the step. */
#define COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD_SHIFT)) & \
     COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD_MASK)
/*! @} */

/*! @name AA_INIT - Access address for the initiator (first half step). */
/*! @{ */

#define COM_MODE_013_CFG_HDR_AA_INIT_AA_INIT_MASK (0xFFFFFFFFU)
#define COM_MODE_013_CFG_HDR_AA_INIT_AA_INIT_SHIFT (0U)
/*! AA_INIT - Access Address. */
#define COM_MODE_013_CFG_HDR_AA_INIT_AA_INIT(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_CFG_HDR_AA_INIT_AA_INIT_SHIFT)) & \
     COM_MODE_013_CFG_HDR_AA_INIT_AA_INIT_MASK)
/*! @} */

/*! @name AA_REFL - Access address for the reflector (second half step). */
/*! @{ */

#define COM_MODE_013_CFG_HDR_AA_REFL_AA_REFL_MASK (0xFFFFFFFFU)
#define COM_MODE_013_CFG_HDR_AA_REFL_AA_REFL_SHIFT (0U)
/*! AA_REFL - Access Address. */
#define COM_MODE_013_CFG_HDR_AA_REFL_AA_REFL(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_CFG_HDR_AA_REFL_AA_REFL_SHIFT)) & \
     COM_MODE_013_CFG_HDR_AA_REFL_AA_REFL_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group COM_MODE_013_CFG_HDR_Register_Masks */

/*!
 * @}
 */ /* end of group COM_MODE_013_CFG_HDR_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- COM_MODE_013_RES_BODY Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COM_MODE_013_RES_BODY_Peripheral_Access_Layer COM_MODE_013_RES_BODY Peripheral Access Layer
 * @{
 */

/** COM_MODE_013_RES_BODY - Register Layout Typedef */
typedef struct
{
    __IO uint32_t NADM_ERROR_RSSI; /**< Raw NADM or Payload Error and RSSI for the step., offset: 0x0 */
    __IO uint32_t RTT_RESULT;      /**< Results reported from the PHY RTT block for the step., offset: 0x4 */
    __IO uint32_t CFO_EST;         /**< CFO estimate for the step., offset: 0x8 */
    __IO uint32_t TIMESTAMP;       /**< TPM module timestamp for the step., offset: 0xC */
} COM_MODE_013_RES_BODY_Type;

/* ----------------------------------------------------------------------------
   -- COM_MODE_013_RES_BODY Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COM_MODE_013_RES_BODY_Register_Masks COM_MODE_013_RES_BODY Register Masks
 * @{
 */

/*! @name NADM_ERROR_RSSI - Raw NADM or Payload Error and RSSI for the step. */
/*! @{ */

#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_MASK (0xFFU)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_SHIFT (0U)
/*! RSSI_NB - Narrowband RSSI measured for the step. Signed?? format. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RSSI_NB_MASK)

#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PAY_ERR_MASK (0xFFFFFF00U)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PAY_ERR_SHIFT (8U)
/*! RAW_NADM_PAY_ERR - Depending upon the step type or configuration, this field contains a Raw NADM or Payload Error
 * result for the step. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PAY_ERR(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PAY_ERR_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PAY_ERR_MASK)

/* Hand edited */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_MASK (0x0000FF00U)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_SHIFT (8U)
/*! RAW_NADM_PAY_ERR - Depending upon the step type or configuration, this field contains a Raw NADM or Payload Error
 * result for the step. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALUE_MASK)

#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID_MASK (0x00010000U)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID_SHIFT (16U)
/*! RAW_NADM_PAY_ERR - Depending upon the step type or configuration, this field contains a Raw NADM or Payload Error
 * result for the step. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_CORR_VALID_MASK)

#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE_MASK (0x001E0000U)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE_SHIFT (17U)
/*! RAW_NADM_PAY_ERR - Depending upon the step type or configuration, this field contains a Raw NADM or Payload Error
 * result for the step. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALUE_MASK)

#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID_MASK (0x00200000U)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID_SHIFT (21U)
/*! RAW_NADM_PAY_ERR - Depending upon the step type or configuration, this field contains a Raw NADM or Payload Error
 * result for the step. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_FM_SYM_ERR_VALID_MASK)

#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE_MASK (0xFFC00000U)
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE_SHIFT (22U)
/*! RAW_NADM_PAY_ERR - Depending upon the step type or configuration, this field contains a Raw NADM or Payload Error
 * result for the step. */
#define COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE_SHIFT)) & \
     COM_MODE_013_RES_BODY_NADM_ERROR_RSSI_RAW_NADM_PDELTA_VALUE_MASK)
/* End Hand Edit */

/*! @} */

/*! @name RTT_RESULT - Results reported from the PHY RTT block for the step. */
/*! @{ */

#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_MASK (0x1U)
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_SHIFT (0U)
/*! RTT_VLD - RTT valid status. */
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_SHIFT)) & \
     COM_MODE_013_RES_BODY_RTT_RESULT_RTT_VLD_MASK)

#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_MASK (0x2U)
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_SHIFT (1U)
/*! RTT_FOUND - RTT found status. */
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_SHIFT)) & \
     COM_MODE_013_RES_BODY_RTT_RESULT_RTT_FOUND_MASK)

#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_MASK (0x3FFFCU)
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_SHIFT (2U)
/*! RTT_CFO - RTT Carrier Frequency Offset (CFO) result. */
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_SHIFT)) & \
     COM_MODE_013_RES_BODY_RTT_RESULT_RTT_CFO_MASK)

#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ_MASK (0xC0000U)
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ_SHIFT (18U)
/*! RTT_INT_ADJ - Integer adjustment reported from the RTT result. */
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ_SHIFT)) & \
     COM_MODE_013_RES_BODY_RTT_RESULT_RTT_INT_ADJ_MASK)

#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT_MASK (0x300000U)
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT_SHIFT (20U)
/*! RTT_HAM_DIST_SAT - Hamming Distance Saturation from the RTT result. */
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT_SHIFT)) & \
     COM_MODE_013_RES_BODY_RTT_RESULT_RTT_HAM_DIST_SAT_MASK)

#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA_MASK (0xFFC00000U)
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA_SHIFT (22U)
/*! RTT_P_DELTA - P-delta value from the RTT result. */
#define COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA_SHIFT)) & \
     COM_MODE_013_RES_BODY_RTT_RESULT_RTT_P_DELTA_MASK)
/*! @} */

/*! @name CFO_EST - CFO estimate for the step. */
/*! @{ */

#define COM_MODE_013_RES_BODY_CFO_EST_CFO_EST_MASK (0x3FFU)
#define COM_MODE_013_RES_BODY_CFO_EST_CFO_EST_SHIFT (0U)
/*! CFO_EST - Carrier Frequency Offset estimate for the step. Signed?? format. */
#define COM_MODE_013_RES_BODY_CFO_EST_CFO_EST(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_CFO_EST_CFO_EST_SHIFT)) & \
     COM_MODE_013_RES_BODY_CFO_EST_CFO_EST_MASK)
/*! @} */

/*! @name TIMESTAMP - TPM module timestamp for the step. */
/*! @{ */

#define COM_MODE_013_RES_BODY_TIMESTAMP_TPM_MASK (0xFFFFFFFFU)
#define COM_MODE_013_RES_BODY_TIMESTAMP_TPM_SHIFT (0U)
/*! TPM - Timestamp for the step. Resolution???. */
#define COM_MODE_013_RES_BODY_TIMESTAMP_TPM(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << COM_MODE_013_RES_BODY_TIMESTAMP_TPM_SHIFT)) & \
     COM_MODE_013_RES_BODY_TIMESTAMP_TPM_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group COM_MODE_013_RES_BODY_Register_Masks */

/*!
 * @}
 */ /* end of group COM_MODE_013_RES_BODY_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- COM_RES_HDR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COM_RES_HDR_Peripheral_Access_Layer COM_RES_HDR Peripheral Access Layer
 * @{
 */

/** COM_RES_HDR - Register Layout Typedef */
typedef struct
{
    __IO uint8_t STEP_ID;            /**< STEP_ID for the step., offset: 0x0 */
    __IO uint8_t SIZE_AGC_IDX;       /**< Result size and AGC index for the step., offset: 0x1 */
    __IO uint16_t PBCD_CTUNE_AA_DET; /**< PBCD or CTUNE and AA detection status for the step, offset: 0x2 */
} COM_RES_HDR_Type;

/* ----------------------------------------------------------------------------
   -- COM_RES_HDR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COM_RES_HDR_Register_Masks COM_RES_HDR Register Masks
 * @{
 */

/*! @name STEP_ID - STEP_ID for the step. */
/*! @{ */

#define COM_RES_HDR_STEP_ID_STEP_ID_MASK (0xFFU)
#define COM_RES_HDR_STEP_ID_STEP_ID_SHIFT (0U)
/*! STEP_ID - STEP_ID for the step. */
#define COM_RES_HDR_STEP_ID_STEP_ID(x) \
    (((uint8_t)(((uint8_t)(x)) << COM_RES_HDR_STEP_ID_STEP_ID_SHIFT)) & COM_RES_HDR_STEP_ID_STEP_ID_MASK)
/*! @} */

/*! @name SIZE_AGC_IDX - Result size and AGC index for the step. */
/*! @{ */

#define COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE_MASK (0xFU)
#define COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE_SHIFT (0U)
/*! RESULT_SIZE - Result size ??? for the step. */
#define COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE(x)                                  \
    (((uint8_t)(((uint8_t)(x)) << COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE_SHIFT)) & \
     COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE_MASK)

#define COM_RES_HDR_SIZE_AGC_IDX_AGC_IDX_MASK (0xF0U)
#define COM_RES_HDR_SIZE_AGC_IDX_AGC_IDX_SHIFT (4U)
/*! AGC_IDX - AGC Index reported for the step. */
#define COM_RES_HDR_SIZE_AGC_IDX_AGC_IDX(x) \
    (((uint8_t)(((uint8_t)(x)) << COM_RES_HDR_SIZE_AGC_IDX_AGC_IDX_SHIFT)) & COM_RES_HDR_SIZE_AGC_IDX_AGC_IDX_MASK)
/*! @} */

/*! @name PBCD_CTUNE_AA_DET - PBCD or CTUNE and AA detection status for the step */
/*! @{ */

#define COM_RES_HDR_PBCD_CTUNE_AA_DET_PBCD_CTUNE_MASK (0x1FFU)
#define COM_RES_HDR_PBCD_CTUNE_AA_DET_PBCD_CTUNE_SHIFT (0U)
/*! PBCD_CTUNE - PBCD or CTUNE for the step. */
#define COM_RES_HDR_PBCD_CTUNE_AA_DET_PBCD_CTUNE(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_RES_HDR_PBCD_CTUNE_AA_DET_PBCD_CTUNE_SHIFT)) & \
     COM_RES_HDR_PBCD_CTUNE_AA_DET_PBCD_CTUNE_MASK)

#define COM_RES_HDR_PBCD_CTUNE_AA_DET_TIME_DRIFT_MASK (0x3000U)
#define COM_RES_HDR_PBCD_CTUNE_AA_DET_TIME_DRIFT_SHIFT (12U)
/*! PBCD_CTUNE - PBCD or CTUNE for the step. */
#define COM_RES_HDR_PBCD_CTUNE_AA_DET_TIME_DRIFT(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_RES_HDR_PBCD_CTUNE_AA_DET_TIME_DRIFT_SHIFT)) & \
     COM_RES_HDR_PBCD_CTUNE_AA_DET_TIME_DRIFT_MASK)

#define COM_RES_HDR_PBCD_CTUNE_AA_DET_AA_DET_MASK (0x8000U)
#define COM_RES_HDR_PBCD_CTUNE_AA_DET_AA_DET_SHIFT (15U)
/*! AA_DET - AA detection status reported for the step. */
#define COM_RES_HDR_PBCD_CTUNE_AA_DET_AA_DET(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << COM_RES_HDR_PBCD_CTUNE_AA_DET_AA_DET_SHIFT)) & \
     COM_RES_HDR_PBCD_CTUNE_AA_DET_AA_DET_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group COM_RES_HDR_Register_Masks */

/*!
 * @}
 */ /* end of group COM_RES_HDR_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- IQ_RES_BODY Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IQ_RES_BODY_Peripheral_Access_Layer IQ_RES_BODY Peripheral Access Layer
 * @{
 */

/** IQ_RES_BODY - Register Layout Typedef */
typedef struct
{
    __IO uint32_t PCT_RESULT[5]; /**< IQ and TQI result for the step., array offset: 0x0, array step: 0x4 */
} IQ_RES_BODY_Type;

/* ----------------------------------------------------------------------------
   -- IQ_RES_BODY Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IQ_RES_BODY_Register_Masks IQ_RES_BODY Register Masks
 * @{
 */

/*! @name PCT_RESULT - IQ and TQI result for the step. */
/*! @{ */

#define IQ_RES_BODY_PCT_RESULT_TQI_MASK (0x3U)
#define IQ_RES_BODY_PCT_RESULT_TQI_SHIFT (0U)
/*! TQI - Tone Quality Indication (TQI) for the individual path. */
#define IQ_RES_BODY_PCT_RESULT_TQI(x) \
    (((uint32_t)(((uint32_t)(x)) << IQ_RES_BODY_PCT_RESULT_TQI_SHIFT)) & IQ_RES_BODY_PCT_RESULT_TQI_MASK)

#define IQ_RES_BODY_PCT_RESULT_PCT_I_MASK (0xFFF0U)
#define IQ_RES_BODY_PCT_RESULT_PCT_I_SHIFT (4U)
/*! PCT_I - In-phase component of the PCT for the individual path. Signed format ??? */
#define IQ_RES_BODY_PCT_RESULT_PCT_I(x) \
    (((uint32_t)(((uint32_t)(x)) << IQ_RES_BODY_PCT_RESULT_PCT_I_SHIFT)) & IQ_RES_BODY_PCT_RESULT_PCT_I_MASK)

#define IQ_RES_BODY_PCT_RESULT_PCT_Q_MASK (0xFFF00000U)
#define IQ_RES_BODY_PCT_RESULT_PCT_Q_SHIFT (20U)
/*! PCT_Q - Quadrature component of the PCT for the individual path. Signed format ??? */
#define IQ_RES_BODY_PCT_RESULT_PCT_Q(x) \
    (((uint32_t)(((uint32_t)(x)) << IQ_RES_BODY_PCT_RESULT_PCT_Q_SHIFT)) & IQ_RES_BODY_PCT_RESULT_PCT_Q_MASK)
/*! @} */

/* The count of IQ_RES_BODY_PCT_RESULT */
#define IQ_RES_BODY_PCT_RESULT_COUNT (5U)

/*!
 * @}
 */ /* end of group IQ_RES_BODY_Register_Masks */

/*!
 * @}
 */ /* end of group IQ_RES_BODY_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- MODE2_CFG_HDR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MODE2_CFG_HDR_Peripheral_Access_Layer MODE2_CFG_HDR Peripheral Access Layer
 * @{
 */

/** MODE2_CFG_HDR - Register Layout Typedef */
typedef struct
{
    __IO uint16_t CHANNEL_NUM;    /**< Channel number for the step., offset: 0x0 */
    __IO uint16_t STEP_CFG;       /**< Configuration for the step., offset: 0x2 */
    __IO uint16_t STEP_CFO;       /**< CFO setting for the step, offset: 0x4 */
    __IO uint16_t HPM_CAL_FACTOR; /**< HPM CAL factor to apply for the step., offset: 0x6 */
    __IO uint32_t CTUNE_MANUAL;   /**< CTUNE manual to inject for the step., offset: 0x8 */
} MODE2_CFG_HDR_Type;

/* ----------------------------------------------------------------------------
   -- MODE2_CFG_HDR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MODE2_CFG_HDR_Register_Masks MODE2_CFG_HDR Register Masks
 * @{
 */

/*! @name CHANNEL_NUM - Channel number for the step. */
/*! @{ */

#define MODE2_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_MASK (0xFFFFU)
#define MODE2_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_SHIFT (0U)
/*! CHANNEL_NUM - Channel number for the step. */
#define MODE2_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_SHIFT)) & \
     MODE2_CFG_HDR_CHANNEL_NUM_CHANNEL_NUM_MASK)
/*! @} */

/*! @name STEP_CFG - Configuration for the step. */
/*! @{ */

#define MODE2_CFG_HDR_STEP_CFG_MODE_MASK (0x3U)
#define MODE2_CFG_HDR_STEP_CFG_MODE_SHIFT (0U)
/*! MODE - Mode for the step
 *  0b00..Frequency Compensation Step (Mode 0)
 *  0b01..Pk-Pk step (Mode 1)
 *  0b10..Tn-Tn step (Mode 2)
 *  0b11..Pk-Tn-Tn-Pk step (Mode 3)
 */
#define MODE2_CFG_HDR_STEP_CFG_MODE(x) \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFG_MODE_SHIFT)) & MODE2_CFG_HDR_STEP_CFG_MODE_MASK)

#define MODE2_CFG_HDR_STEP_CFG_TONE_EXT_MASK (0xCU)
#define MODE2_CFG_HDR_STEP_CFG_TONE_EXT_SHIFT (2U)
/*! TONE_EXT - Choice of tone extension for the initiator portion of the step.
 *  0b00..Tone extension is not present for either role TX.
 *  0b01..Tone extension is present for initiator TX only.
 *  0b10..Tone extension is present for reflector TX only.
 *  0b11..Tone extension is present for both roles TX.
 */
#define MODE2_CFG_HDR_STEP_CFG_TONE_EXT(x) \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFG_TONE_EXT_SHIFT)) & MODE2_CFG_HDR_STEP_CFG_TONE_EXT_MASK)

#define MODE2_CFG_HDR_STEP_CFG_ANT_PERMUT_MASK (0x1F0U)
#define MODE2_CFG_HDR_STEP_CFG_ANT_PERMUT_SHIFT (4U)
/*! ANT_PERMUT - Antenna permutation pattern for the step. */
#define MODE2_CFG_HDR_STEP_CFG_ANT_PERMUT(x) \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFG_ANT_PERMUT_SHIFT)) & MODE2_CFG_HDR_STEP_CFG_ANT_PERMUT_MASK)

#define MODE2_CFG_HDR_STEP_CFG_ANT_CS_SYNC_MASK (0x600U)
#define MODE2_CFG_HDR_STEP_CFG_ANT_CS_SYNC_SHIFT (9U)
/*! ANT_CS_SYNC - Antenna cs Sync */
#define MODE2_CFG_HDR_STEP_CFG_ANT_CS_SYNC(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFG_ANT_CS_SYNC_SHIFT)) & \
     MODE2_CFG_HDR_STEP_CFG_ANT_CS_SYNC_MASK)

#define MODE2_CFG_HDR_STEP_CFG_EP_ONE_WAY_MASK (0x800U)
#define MODE2_CFG_HDR_STEP_CFG_EP_ONE_WAY_SHIFT (11U)
/*! EP_ONE_WAY - One way ranging? */
#define MODE2_CFG_HDR_STEP_CFG_EP_ONE_WAY(x) \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFG_EP_ONE_WAY_SHIFT)) & MODE2_CFG_HDR_STEP_CFG_EP_ONE_WAY_MASK)

#define MODE2_CFG_HDR_STEP_CFG_T_PM_SEL_MASK (0x1000U)
#define MODE2_CFG_HDR_STEP_CFG_T_PM_SEL_SHIFT (12U)
/*! T_PM_SEL - Selection of T_PM choices. */
#define MODE2_CFG_HDR_STEP_CFG_T_PM_SEL(x) \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFG_T_PM_SEL_SHIFT)) & MODE2_CFG_HDR_STEP_CFG_T_PM_SEL_MASK)
/*! @} */

/*! @name STEP_CFO - CFO setting for the step */
/*! @{ */

#define MODE2_CFG_HDR_STEP_CFO_STEP_CFO_MASK (0xFFFFU)
#define MODE2_CFG_HDR_STEP_CFO_STEP_CFO_SHIFT (0U)
/*! STEP_CFO - CFO setting for the step. */
#define MODE2_CFG_HDR_STEP_CFO_STEP_CFO(x) \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_STEP_CFO_STEP_CFO_SHIFT)) & MODE2_CFG_HDR_STEP_CFO_STEP_CFO_MASK)
/*! @} */

/*! @name HPM_CAL_FACTOR - HPM CAL factor to apply for the step. */
/*! @{ */

#define MODE2_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_MASK (0xFFFU)
#define MODE2_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_SHIFT (0U)
/*! HPM_CAL_FACTOR - HPM CAL factor to apply for the step */
#define MODE2_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR(x)                                    \
    (((uint16_t)(((uint16_t)(x)) << MODE2_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_SHIFT)) & \
     MODE2_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR_MASK)
/*! @} */

/*! @name CTUNE_MANUAL - CTUNE manual to inject for the step. */
/*! @{ */

#define MODE2_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_MASK (0x1FFU)
#define MODE2_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_SHIFT (0U)
/*! CTUNE_MANUAL - CTUNE manual to inject for the step. */
#define MODE2_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << MODE2_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_SHIFT)) & \
     MODE2_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group MODE2_CFG_HDR_Register_Masks */

/*!
 * @}
 */ /* end of group MODE2_CFG_HDR_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- PAYLOAD_CFG_BODY Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PAYLOAD_CFG_BODY_Peripheral_Access_Layer PAYLOAD_CFG_BODY Peripheral Access Layer
 * @{
 */

/** PAYLOAD_CFG_BODY - Register Layout Typedef */
typedef struct
{
    __IO uint32_t AA_INIT;
    __IO uint32_t AA_REFL;
    struct
    {
        __IO uint32_t INIT;
        __IO uint32_t REFL;
    } PAYLOAD[4];
} PAYLOAD_CFG_BODY_Type;

/* ----------------------------------------------------------------------------
   -- PAYLOAD_CFG_BODY Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PAYLOAD_CFG_BODY_Register_Masks PAYLOAD_CFG_BODY Register Masks
 * @{
 */

/*! @name PAYLOAD_INIT - 32bit word of payload for the initiator for the step. */
/*! @{ */

#define PAYLOAD_CFG_BODY_PAYLOAD_INIT_PLD_MASK (0xFFFFFFFFU)
#define PAYLOAD_CFG_BODY_PAYLOAD_INIT_PLD_SHIFT (0U)
/*! PLD - One payload word. */
#define PAYLOAD_CFG_BODY_PAYLOAD_INIT_PLD(x) \
    (((uint32_t)(((uint32_t)(x)) << PAYLOAD_CFG_BODY_PAYLOAD_INIT_PLD_SHIFT)) & PAYLOAD_CFG_BODY_PAYLOAD_INIT_PLD_MASK)
/*! @} */

/* The count of PAYLOAD_CFG_BODY_PAYLOAD_INIT */
#define PAYLOAD_CFG_BODY_PAYLOAD_INIT_COUNT (4U)

/*! @name PAYLOAD_REFL - 32bit word of payload for the reflector for the step. */
/*! @{ */

#define PAYLOAD_CFG_BODY_PAYLOAD_REFL_PLD_MASK (0xFFFFFFFFU)
#define PAYLOAD_CFG_BODY_PAYLOAD_REFL_PLD_SHIFT (0U)
/*! PLD - One payload word. */
#define PAYLOAD_CFG_BODY_PAYLOAD_REFL_PLD(x) \
    (((uint32_t)(((uint32_t)(x)) << PAYLOAD_CFG_BODY_PAYLOAD_REFL_PLD_SHIFT)) & PAYLOAD_CFG_BODY_PAYLOAD_REFL_PLD_MASK)
/*! @} */

/* The count of PAYLOAD_CFG_BODY_PAYLOAD_REFL */
#define PAYLOAD_CFG_BODY_PAYLOAD_REFL_COUNT (4U)

/*!
 * @}
 */ /* end of group PAYLOAD_CFG_BODY_Register_Masks */

/*!
 * @}
 */ /* end of group PAYLOAD_CFG_BODY_Peripheral_Access_Layer */

/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
#if (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#else
#pragma pop
#endif
#elif defined(__CWCC__)
#pragma pop
#elif defined(__GNUC__)
/* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma language = default
#else
#error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */

/* ----------------------------------------------------------------------------
   -- Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Bit_Field_Generic_Macros Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
 * @{
 */

#if defined(__ARMCC_VERSION)
#if (__ARMCC_VERSION >= 6010050)
#pragma clang system_header
#endif
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma system_include
#endif

/**
 * @brief Mask and left-shift a bit field value for use in a register bit range.
 * @param field Name of the register bit field.
 * @param value Value of the bit field.
 * @return Masked and shifted value.
 */
#define NXP_VAL2FLD(field, value) (((value) << (field##_SHIFT)) & (field##_MASK))
/**
 * @brief Mask and right-shift a register value to extract a bit field value.
 * @param field Name of the register bit field.
 * @param value Value of the register.
 * @return Masked and shifted bit field value.
 */
#define NXP_FLD2VAL(field, value) (((value) & (field##_MASK)) >> (field##_SHIFT))

/*!
 * @}
 */ /* end of group Bit_Field_Generic_Macros */

/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */

#endif /* NXP_XCVR_LCL_STEP_STRUCTS_H_ */
