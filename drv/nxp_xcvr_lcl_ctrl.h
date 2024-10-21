/*
 * Copyright 2020-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef NXP_XCVR_LCL_CTRL_H
/* clang-format off */
#define NXP_XCVR_LCL_CTRL_H
/* clang-format on */

#include "fsl_common.h"
#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_ext_ctrl.h"      /* Include support for antenna control and pattern match */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450 )
#include "nxp_xcvr_lcl_config.h"    /* RSM timing configurations */
#endif

/*!
 * @addtogroup xcvr_localization Localization Routines
 * @{
 */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470 )
#define CONNRF_1163_IF_COMP 1
#else
#define CONNRF_1163_IF_COMP 1
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(__cplusplus)
    extern "C" {
#endif

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)  && (RF_OSC_26MHZ == 0) /* RSM only supported on Gen 4.5 or higher radios & with RF_OSC = 32MHz */

#ifndef XCVR_SKIP_RSM_SETTINGS_CHECK
#define XCVR_SKIP_RSM_SETTINGS_CHECK (1)
#endif /* XCVR_SKIP_RSM_SETTINGS_CHECK */

/* Workaround for PKT RAM double buffering issue on KW47 at end of TX RAM. Last location is not usable */
/* This symbol should be used in all double buffering related code to enforce not using the last location */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 470) 
//#define TX_PACKET_RAM_PACKET_RAM_COUNT_DBL_BUFFER   (TX_PACKET_RAM_PACKET_RAM_COUNT-1U)
#define TX_PACKET_RAM_PACKET_RAM_COUNT_DBL_BUFFER   (TX_PACKET_RAM_PACKET_RAM_COUNT-20U-1U)
#else
#define TX_PACKET_RAM_PACKET_RAM_COUNT_DBL_BUFFER   (TX_PACKET_RAM_PACKET_RAM_COUNT)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 470) */

/* Long (64 bit) PN support is disabled by default but can be enabled at project level */
 #ifndef SUPPORT_RSM_LONG_PN
 #define SUPPORT_RSM_LONG_PN        (0)
 #endif /* SUPPORT_RSM_LONG_PN */

#define NUM_TSM_U32_REGISTERS   (sizeof(XCVR_TSM_Type)/4U)

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) 
#define XCVR_RSM_OVERALL_MAX_SEQ_LEN     (160U)      /*!< Maximum overall sequence length (inclusive) for RSM sequences. No sequence can be longer than this due to Fstep RAM array size. */
#define XCVR_RSM_FCS_PKPK_MAX_STEP_COUNT (160U)      /*!< Maximum number of FCS, Pk-Pk, or Pk-Tn-Tn-Pk steps within any RSM sequence.  This is due to PN and RTT RAM array sizes. */
#else
#define XCVR_RSM_OVERALL_MAX_SEQ_LEN     (128U)      /*!< Maximum overall sequence length (inclusive) for RSM sequences. No sequence can be longer than this due to Fstep RAM array size. */
#define XCVR_RSM_FCS_PKPK_MAX_STEP_COUNT (106U)      /*!< Maximum number of FCS, Pk-Pk, or Pk-Tn-Tn-Pk steps within any RSM sequence.  This is due to PN and RTT RAM array sizes. */
#endif
#ifndef XCVR_RSM_MIN_SEQ_LEN
#define XCVR_RSM_MIN_SEQ_LEN             (1U)        /*!< Minimum sequence length (inclusive) for RSM sequences */
#endif /* XCVR_RSM_MIN_SEQ_LEN */
#define XCVR_RSM_MAX_NUM_ANT             (32U)        /*!< Maximum number of antennae supported by LCL Antenna Control (assumes external decode logic).  */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) 
/* Names kept the same as KW45 for re-use; values are defined with some defaults for test purposes */
// TODO: remove these symbols, they only apply to KW45; KW47 needs a different mechanism.
#define RSM_RTT_RAM         (RX_PACKET_RAM_BASE+0x80U)  /*!< Round Trip Time Buffer Base Address */
#define RSM_RTT_RAM_ENTRY_SZ (4U)                        /*!< Size (bytes) of one entry in the Round Trip Time Buffer */
#define RSM_RTT_RAM_COUNT   (RX_PACKET_RAM_PACKET_RAM_COUNT*4U/RSM_RTT_RAM_ENTRY_SZ)  /*!< Round Trip Time Buffer count of elements */

#else /* KW45 version */
#define RSM_PCBD_RAM        (RX_PACKET_RAM_BASE)        /*!< Coarse Tune Best Diff Buffer Base Address */
#define RSM_PCBD_ENTRY_SZ   (1U)                        /*!< Size (bytes) of one entry in the Coarse Tune Best Diff Buffer  */
#define RSM_PCBD_RAM_COUNT  (128U)                      /*!< Coarse Tune Best Diff Buffer count of elements */
#define RSM_RTT_RAM         (RX_PACKET_RAM_BASE+0x80U)  /*!< Round Trip Time Buffer Base Address */
#define RSM_RTT_RAM_ENTRY_SZ (4U)                        /*!< Size (bytes) of one entry in the Round Trip Time Buffer */
#define RSM_RTT_RAM_COUNT   (424U/RSM_RTT_RAM_ENTRY_SZ)  /*!< Round Trip Time Buffer count of elements */
#define RSM_PN_RAM          (RX_PACKET_RAM_BASE+0x230U)  /*!< PN (both 32 and 64) Buffer Base Address */
#define RSM_PN_RAM_32_ENTRY_SZ (8U)                      /*!< Size (bytes) of one entry in the PN  32  Buffer  */
#define RSM_PN_RAM_32_COUNT (848U/RSM_PN_RAM_32_ENTRY_SZ) /*!< PN  32  Buffer count of elements */
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
#define RSM_PN_RAM_64_ENTRY_SZ (16U)                     /*!< Size (bytes) of one entry in the PN  64  Buffer  */
#define RSM_PN_RAM_64_COUNT (848U/RSM_PN_RAM_64_ENTRY_SZ) /*!< PN  64  Buffer count of elements */
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */
#define RSM_FSTEP_RAM       (RX_PACKET_RAM_BASE+0x580U)  /*!< Frequency Step Buffer Base Address */
#define RSM_FSTEP_ENTRY_SZ  (5U)                         /*!< Size (bytes) of one entry in the  Frequency Step Buffer */
#define RSM_FSTEP_RAM_COUNT (640U/RSM_FSTEP_ENTRY_SZ)    /*!< Frequency Step Buffer count of elements */
#endif  /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */

#define GAMMA_ROWS          (6U)                         /*!< Number of rows in Gamma array for RTT fractional delay estimation */
#define GAMMA_COLS          (4U)                         /*!< Number of columns in Gamma array for RTT fractional delay estimation */

#define BLE_MIN_FREQ 2402U
#define BLE_MAX_FREQ 2480U
#define RSM_HADM_MAX_CHAN_INDEX 78U /*!< HADM channel indexes run from 0 to 78 to represent frequencies 2402 to 2480MHz in 1MHz increments */

#define T_RD                (5U)
#define T_FM_USEC       (80U)  /*!< Channel Sounding spec only permits 80usec T_FM */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  
/* KW45 increment settings for RSM timing registers (usec per LSB) */
#define T_FM_INCMT  (10U)           /*!< 10usec per bit in T_FM field programming */
#define T_PM_INCMT  (10U)           /*!< 10usec per bit in T_PM field programming */
#define T_PM_REG_OFFSET (1U)       /*!< One increment offset. value 0 is 10us */
#define T_IP_INCMT  (5U)           /*!< 5usec per bit in T_IP field programming */
#define T_FC_INCMT  (5U)           /*!< 5usec per bit in T_FC field programming */
#define T_S_INCMT  (5U)            /*!< 5usec per bit in T_S field programming */
#define T_FC_MAX (155U)
#define T_FC_MIN (50U)
#define T_FC_MODULO (5U)
#define T_IP_MAX (155U)
#define T_IP_MIN (5U)
#define T_IP_MODULO (5U)
#define T_PM_MAX (640U)
#define T_PM_MIN (10U)
#define T_PM_MODULO (10U)
#define T_PM_FLD_COUNT (4U)  /*!< 4 T_PM register bitfields are present */
#define T_FM_FLD_COUNT (4U)  /*!< 4 T_FM register bitfields are present */
#define ZERO_BASIS (1U)          /*!< A zero value in this register actually means 1 of whatever timing is being programmed */
#else
/* KW47 switched to 1usec per LSB for all timings and widened many fields */
/* KW47 timing registers do not have any cases where zero entry means 1 increment as KW45 had. Zero entry is never allowed! */
#define T_FM_INCMT  (1U)           /*!< 1usec per bit in T_FM field programming */
#define T_PM_INCMT  (1U)           /*!< 1usec per bit in T_PM field programming */
#define T_PM_REG_OFFSET (0U)       /*!< no offset. value 0 is 0us */
#define T_IP_INCMT  (1U)            /*!< 1usec per bit in T_FM field programming */
#define T_FC_INCMT  (1U)           /*!< 1usec per bit in T_FM field programming */
#define T_S_INCMT  (1U)             /*!< 1usec per bit in T_S field programming */
#define T_FC_MAX (255U)
#define T_FC_MIN (30U)
#define T_FC_MODULO (1U)
#define T_IP_MAX (255U)
#define T_IP_MIN (20U)
#define T_IP_MODULO (1U)
#define T_PM_MAX (1023U)
#define T_PM_MIN (10U)
#define T_PM_MODULO (1U)
#define T_PM_FLD_COUNT (2U)  /*!< 2 T_PM register bitfields are present */
#define T_FM_FLD_COUNT (1U)  /*!< 1 T_FM register bitfields are present */
#define ZERO_BASIS (0U)          /*!< A zero value in this register actually means 0 of whatever timing is being programmed */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)  */

#define SAMPLING_RATE_FACTOR_2MBPS (2U)      /* Sampling rate x 2 @2Mbps */
#define RX_SAMPLING_RATE (4U)                /* 4 MHz @1Mbps */
#define TX_SAMPLING_RATE (8U)                /* 8 MHz @1Mbps */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
#define TX_DATA_FLUSH_DLY_1MBPS     (3U)
#define TX_DATA_FLUSH_DLY_2MBPS     (3U)
#define RX_SYNC_DLY_1MBPS                  (5U) /* Must change when RX_SETTLING_LATENCY_1MBPS changes */
#define RX_SYNC_DLY_2MBPS                  (4U) /* Must change when RX_SETTLING_LATENCY_2MBPS changes */
#define RX_SETTLING_LATENCY_1MBPS  (6U)
#define RX_SETTLING_LATENCY_2MBPS  (4U)
#else
#define TX_DATA_FLUSH_DLY_1MBPS     (2U)
#define TX_DATA_FLUSH_DLY_2MBPS     (1U)
#define RX_SYNC_DLY_1MBPS                  (6U)
#define RX_SYNC_DLY_2MBPS                  (7U)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)   */

/* Channel Sounding Test Commands */
#if (0)
#define CS_TEST_AA_INIT     (0xCD31EF64U)
#define CS_TEST_AA_REFL     (0xD6844E8EU)
#else
#define CS_TEST_AA_INIT     (0x36B25785U)
#define CS_TEST_AA_REFL     (0xA1E16E78U)
#endif
/*! @brief  RSM control mode settings. */
typedef enum
{
    XCVR_RSM_SQTE_MODE       = 0U, /*!< Secure Quick Tone Exchange mode (AKA HADM) */
    XCVR_RSM_PDE_MODE        = 1U,  /*!< Phase Distance Estimation mode */
    XCVR_RSM_SQTE_STABLE_PHASE_TEST_MODE       = 2U, /*!< Secure Quick Tone Exchange mode with settings for stable phase testing */
    XCVR_RSM_MODE_INVALID /* Must always be last! */
}   XCVR_RSM_MODE_T;

/*! @brief  RSM RX/TX mode settings. */
typedef enum
{
    XCVR_RSM_RX_MODE       = 0U, /*!< Configure RSM to start with Receive. AKA the SQTE "Reflector" role or the PDE "RD" role */
    XCVR_RSM_TX_MODE       = 1U,  /*!<  Configure RSM to start with Transmit. AKA the SQTE "Initiator" role or the PDE "MD" role */
    XCVR_RSM_RXTX_MODE_INVALID /* Must always be last! */
}   XCVR_RSM_RXTX_MODE_T;

/*! @brief  RSM Trigger Selection  settings. */
typedef enum
{
    XCVR_RSM_TRIG_SW            = 0U,  /*!< Trigger the RSM module immediately (software triggered). */
    XCVR_RSM_TRIG_CRC_VLD       = 1U,  /*!<  Trigger the RSM module upon CRC Valid detection. */
    XCVR_RSM_TRIG_AA_FND        = 2U,  /*!<  Trigger the RSM module upon Access Address detection from the PHY. */
    XCVR_RSM_TRIG_TX_DIG_EN     = 3U,  /*!<  Trigger the RSM module upon TX digital enable from TSM. */
    XCVR_RSM_TRIG_SEQ_SPARE3    = 4U,  /*!<  Trigger the RSM module upon seq_spare3 assertion from TSM. */
    XCVR_RSM_TRIG_PAT_MATCH     = 5U,   /*!<  Trigger the RSM module upon pattern match detection from the localization module. */
    XCVR_RSM_TRIG_NBU_TRIG        = 6U,   /*!<  Trigger the RSM module uby NBU signal from LL hardware. */
    XCVR_RSM_TRIG_INVALID /* Must always be last! */
}   XCVR_RSM_TRIG_T;

/*! @brief  RSM Calibration Configuration. Selects the calibrations to include in RSM TX and RX sequences. */
typedef enum
{
    XCVR_RSM_TSM_DEFAULT        = 0U, /*!< Configure TSM settings programming to default settings */
    XCVR_RSM_TSM_FULL_CAL       = 1U, /*!<  Configure TSM settings programming to full calibration: perform HPM and RCCAL and DCOC cal in both RX and TX warmups for RSM (Requires T_FC=150usec!) */
    XCVR_RSM_TSM_PART_CAL       = 2U,  /*!<  ConfigureTSM settings programming to  assert seq_lo_pup_vlo_rx/tx in both TX and RX, and to enable HPM in RX (use for faster T_FC cases) */
    XCVR_RSM_CAL_MODE_INVALID /* Must always be last! */
}   XCVR_RSM_TSM_CAL_MODE_T;

/*! @brief  RSM HPM Calibration Configuration. Selects the HPM calibrations time to include in RSM TX and RX sequences. */
typedef enum
{
    XCVR_RSM_TSM_HPM_CAL_BYP        = 0U, /*!< Configure TSM settings programming to bypass HPM cal  (used only if HPM CAL is completely skipped) */
    XCVR_RSM_TSM_HPM_52US_CAL       = 1U, /*!<  Configure TSM settings programming to 52usec HPM cal (default value, almost always used) */
    XCVR_RSM_TSM_HPM_102US_CAL      = 2U,  /*!<  ConfigureTSM settings programming to 102usec HPM cal (require PLL settings as well to go with this value). */
    XCVR_RSM_cAL_TIME_INVALID /* Must always be last! */
}   XCVR_RSM_TSM_HPM_CAL_TIME_T;

/*! @brief  RSM SQTE RATE settings. */
typedef enum
{
    XCVR_RSM_RATE_1MBPS       = 0U, /*!< RSM rate of 1Mbps. Must match how XCVR is configured! */
    XCVR_RSM_RATE_2MBPS       = 1U,  /*!<  RSM rate of 2Mbps. Must match how XCVR is configured! */
    XCVR_RSM_RATE_INVALID /* Must always be last! */
}   XCVR_RSM_SQTE_RATE_T;

#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
/*! @brief  RSM SQTE PN length. */
typedef enum
{
    XCVR_RSM_SQTE_PN32       = 0U, /*!< A 32 bit PN sequence is used in SQTE packets. THis setting is programmed in the PHY.  */
    XCVR_RSM_SQTE_PN64       = 1U,  /*!< A 64 bit PN sequence is used in SQTE packets. THis setting is programmed in the PHY. */
    XCVR_RSM_RTT_LEN_INVALID /* Must always be last! */
}   XCVR_RSM_SQTE_RTT_LEN_T;
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */

/*! @brief RXDIG IQ/Phase averaging window enumeration type. */
/* Must match the birfield programming for RX_IQ_PH_AVG_WIN in XCVR_RXDIG->CTRL1 regsiter */
typedef enum
{
    XCVR_RSM_AVG_WIN_DISABLED    = 0U, /*!< No IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_4_SMPL      = 1U, /*!< 4 sample IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_8_SMPL      = 2U, /*!< 8 sample IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_16_SMPL     = 3U, /*!< 16 sample IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_32_SMPL     = 4U, /*!< 32 sample IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_64_SMPL     = 5U, /*!< 64 sample IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_128_SMPL    = 6U, /*!< 128 sample IQ/Phase averaging performed */
    XCVR_RSM_AVG_WIN_256_SMPL    = 7U, /*!< 256 sample IQ/Phase averaging performed */
}   XCVR_RSM_AVG_WIN_LEN_T;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* Only applies to KW47 */
/*! @brief RXDIG PCT averaging window enumeration type. This is a second stage, chained after the RX_IQ_PH_AVG_WIN averager and it used to generate a single PCT per antenna slot. */
/* Must match the bitfield programming for RX_IQ_AVG_WIN_PCT in XCVR_RXDIG->CTRL1 regsiter */
typedef enum
{
    XCVR_RSM_PCT_AVG_WIN_DISABLED    = 0U, /*!< No averaging performed */
    XCVR_RSM_PCT_AVG_WIN_4_SMPL      = 1U, /*!< 4 sample averaging performed */
    XCVR_RSM_PCT_AVG_WIN_8_SMPL      = 2U, /*!< 8 sample averaging performed */
    XCVR_RSM_PCT_AVG_WIN_16_SMPL     = 3U, /*!< 16 sample averaging performed */
    XCVR_RSM_PCT_AVG_WIN_ERROR         = 4U, /*!< Error limit. */
}   XCVR_RSM_PCT_AVG_WIN_LEN_T;

/*! @brief  RSM RTT_TYPE enumeration type. */
typedef enum
{
    XCVR_RSM_RTT_NO_PAYLOAD               = 0U, /*!< No payload in the RTT step. */
    XCVR_RSM_RTT_32BIT_SOUNDING        = 1U, /*!< 32 bit sounding sequence */
    XCVR_RSM_RTT_96BIT_SOUNDING        = 2U, /*!< 96 bit sounding sequence */
    XCVR_RSM_RTT_32BIT_RANDOM           = 3U, /*!< 32 bit random sequence */
    XCVR_RSM_RTT_64BIT_RANDOM           = 4U, /*!< 64 bit random sequence */
    XCVR_RSM_RTT_96BIT_RANDOM           = 5U, /*!< 96 bit random sequence */
    XCVR_RSM_RTT_128BIT_RANDOM         = 6U, /*!< 128 bit random sequence */
    XCVR_RSM_RTT_ERROR                          = 7U, /*!< Error value */
}   XCVR_RSM_RTT_TYPE_T;

/*! @brief PA ramp time  */
/*  @note The 3usec ramp time requires a reprogramming of the PA ramp table to reach max in 3 usec but the last usec is at a single max power value. */
typedef enum
{
    XCVR_RSM_PA_RAMP_0_USEC               = 0U, /*!< Zero PA ramp time. Maps directly the PA_RAMP_SEL bitfield. */
    XCVR_RSM_PA_RAMP_1_USEC               = 1U, /*!< 1usec PA ramp time. Maps directly the PA_RAMP_SEL bitfield. */
    XCVR_RSM_PA_RAMP_2_USEC               = 2U, /*!< 2usec PA ramp time. Maps directly the PA_RAMP_SEL bitfield. */
    XCVR_RSM_PA_RAMP_4_USEC               = 3U, /*!< 4usec PA ramp time. Maps directly the PA_RAMP_SEL bitfield. */
    XCVR_RSM_PA_RAMP_3_USEC               = 4U, /*!< 4usec PA ramp time. Digital duration is still 4usec but ramp table must be reprogrammed to hit max by 3usec. */
    XCVR_RSM_PA_RAMP_ERROR                 = 5U, /*!< Invalid PA ramp time.  */
}   XCVR_RSM_PA_RAMP_TIME_T;

/*! @brief  PLL PIC mode enumeration type. */
typedef enum
{
    XCVR_RSM_PIC_DISABLED                = 0U, /*!< Legacy mode for PLL PIC, not enabled. */
    XCVR_RSM_PIC_FAST_ONLY             = 1U, /*!< PIC Fast mode only */
    XCVR_RSM_PIC_FAST_SLOW            = 2U, /*!< PIC Fast plus Slow mode */
    XCVR_RSM_PIC_ERROR                          = 7U, /*!< Error value */
}   XCVR_RSM_PIC_MODE_TYPE_T;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */

/*! @brief PA ramp time  */
/*  @note The 3usec ramp time requires a reprogramming of the PA ramp table to reach max in 3 usec but the last usec is at a single max power value. */
typedef enum
{
    XCVR_RSM_IQ_OUT_DIS                        = 0U, /*!< Disabled. */
    XCVR_RSM_IQ_OUT_IF_MIXER              = 1U, /*!< Select the IF mixer output for IQ capture point. */
    XCVR_RSM_IQ_OUT_CIC                        = 2U, /*!< Select the CIC filter output for IQ capture point. */
    XCVR_RSM_IQ_OUT_ACQ_CHF               = 3U, /*!< Select the acquisition channel filter output for IQ capture point. */
    XCVR_RSM_IQ_OUT_SRC                        = 4U, /*!< Select the SRC output for IQ capture point. */
    XCVR_RSM_IQ_OUT_CFO_MIXER            = 5U, /*!< Select the CFO mixer output for IQ capture point. */
    XCVR_RSM_IQ_OUT_FRAC_CORR            = 6U, /*!< Select the Fractional Correction output for IQ capture point. */
    XCVR_RSM_IQ_OUT_DC_RESID               = 7U, /*!< Select the DC residual output for IQ capture point. */
    XCVR_RSM_IQ_OUT_ERROR                    = 8U, /*!< Error value. */
}   XCVR_RSM_IQ_OUT_SEL_T;


/* Define callback function for RSM interrupt. */
/*! *********************************************************************************
* \brief  This callback is a pointer to a user routine to be called when RSM interrtupt happens.
*
*  This function is the user callback for DSB interrupts.
*
* \param[in] userData - pointer to user data to be passed to the callback function
* \param[in] abort - boolean indicating if an abort flag was asserted.
* \param[in] rsm_csr - the contents of the RSM Control and Status register for identifying interrupt type.
*
* \note This callback is called for any RSM interrupt, whether an abort flag was raised, an intermediate interrupt was triggered,
* or the end-of-sequence interrupt was asserted.
***********************************************************************************/
typedef void (*rsm_int_callback)(void *userData, bool abort, uint32_t rsm_csr);


/*! @brief RSM main configuration structure. This structure covers common configurations and some PDE configurations. A child structure contains SQTE only configurations. */
typedef struct
{
    XCVR_RSM_MODE_T op_mode;                 /*!< Operating mode for the RSM */
    bool sniffer_mode_en;                /*!< Sniffer mode enable; Only permits RX, not TX. Receives both roles (init and refl) to monitor exchanges. Only applies to KW47, error is generated if set to true for KW45. */
    uint8_t num_steps;                       /*!< Number of steps for the RSM sequence, including FCS for the SQTE case. Max is 128 but the PN sequence storage may also limit the number of steps. A value of 0 is not valid. */
    uint8_t t_fc;                            /*!< T_FC timing value in usec */
    uint8_t t_ip1;                           /*!< T_IP1 timing value in usec */
    uint8_t t_ip2;                           /*!< T_IP2 timing value in usec */
    uint16_t t_pm0;                           /*!< T_PM0 timing value in usec. The T_PM1 field will be set to T_PM0+10. Valid range is 10usec to 630usec (accounting for T_PM1 will be 10 usec longer). This value represents the entire T_PM period, covering one or more antenna slots and also any required tone extension slot. */

    uint8_t rxdig_dly;                       /*!< Used in longer sequences to optimize enable of the RX digital. Can cause a problem in warmup if this is incorrect. Compensating for first T_FC per the standard since RSM triggers on TSM warmup. */
    uint8_t txdig_dly;                       /*!< Used in longer sequences to optimize enable of the TX digital. Can cause a problem in warmup if this is incorrect. Compensating for first T_FC per the standard since RSM triggers on TSM warmup. */
    XCVR_RSM_TRIG_T trig_sel;                /*!< Selects the trigger mode for the RSM module. */
    uint16_t trig_delay;                     /*!< Max value = 2047. SQTE mode: delay from trigger to when rx_en or tx_en is asserted. PDE mode: delay from trigger to when the first rx2tx or tx2rx occurs. */
    uint8_t num_ant_path;                    /*!< Number of antenna paths, must range from 1 to 4. */
    lclTSw_t t_sw;                                  /*!< The antenna switching time to be used when num_ant_path > 1. Zero is used when num_ant_path ==1 */
    rsm_int_callback user_callback;          /*!< User defined callback to be called to handle interrupts. */
#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
    XCVR_RSM_SQTE_RTT_LEN_T rtt_len;         /*!< SQTE ONLY: Choose the length of PN packets used in SQTE packet exchanges.  */
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */
    XCVR_RSM_SQTE_RATE_T rate;               /*!< SQTE ONLY: Indicates to the RSM the data rate being used by the XCVR. Must match the XCVR configuration.  */
    XCVR_RSM_AVG_WIN_LEN_T averaging_win;   /*!< The number of IQ or phase samples to average in RXDIG to produce one output sample. */
    bool use_rsm_dma_mask;                  /*!< Selects to use the RSM DMA mask signal rather than the LCL DMA mask. Only valid when 1 antenna is used. */
    uint8_t rsm_dma_dly_pm;                 /*!< DMA delay from end of RX warmup to assertion of DMA mask signal in usec. Applies to PM_RX state only. RSM_DMA_DLY field. */
    uint8_t rsm_dma_dly_fm_ext;             /*!< DMA delay from end of RX warmup to assertion of DMA mask signal in usec. Applies to FM_RX or EXT_RX states only. RSM_DMA_DLY0 field. Active even if RSM is not providing the DMA mask for PM states.*/
    uint16_t rsm_dma_dur_pm;                /*!< DMA mask duration in usec. Applies to PM_RX state only. RSM_DMA_DUR field. */
    uint16_t rsm_dma_dur_fm_ext;            /*!< DMA mask duration in usec. Applies to FM_RX or EXT_RX states only. RSM_DMA_DUR0 field. Active even if RSM is not providing the DMA mask for PM states. */
    bool disable_rx_sync;                       /*!< Choose whether RX SYNC feature should be disabled. Allows reflector role full sequence execution without an initiator. */
    XCVR_RSM_IQ_OUT_SEL_T iq_out_sel;  /*!< Choose the point in the RX chain where IQ samples are captured. */
    XCVR_RSM_RXTX_MODE_T role;          /*!< Role to be used for RSM operation. Interacts with the trig_sel field because the role cannot be applied in XCVR_LCL_RsmInit() if the trig_sel is set to software triggered. */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) /* Only applies to KW47 */
    uint16_t mode0_timeout_usec;  /*!< Timeout (in usec) for RSM in reflector role to wait for a Mode 0 step transmission from initiator. */
    XCVR_RSM_PCT_AVG_WIN_LEN_T pct_averaging_win;   /*!< The second stage of averaging to produce one output sample. First stage of averaging is in averaging_win and corresponds to RX_IQ_PH_AVG_WIN bitfield. */
    XCVR_RSM_RTT_TYPE_T rtt_type;    /*!< RTT Type, determines payload size for RTT steps. */
    XCVR_RSM_PA_RAMP_TIME_T pa_ramp_time; /*!< PA ramp time in usec, 0, 1, 2, 4, & 3 are valid values */
    bool enable_inpr;                       /*!< Choose whether Inline Phase Return feature should be enabled. */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */
} xcvr_lcl_rsm_config_t;

/*! @brief RSM Frequency Step configuration structure. */
typedef struct
{
    uint8_t ext_channel_num_ovrd_lsb;            /*!< Channel number override LSB for this Frequency Step */
    uint8_t ext_channel_num_ovrd_msb;            /*!< Channel number override LSB for this Frequency Step */
    uint8_t ext_ctune;                           /*!< CTUNE factor for this Frequency Step */
    uint8_t ext_hmp_cal_factor_lsb;              /*!< HPM_CAL factor LSB for this Frequency Step */
    uint8_t tpm_step_format_hmp_cal_factor_msb;  /*!< Mixed field with T_PM_SEL, STEP_FORMAT, and HPM_CAL factor MSB for this Frequency Step */
} xcvr_lcl_fstep_t;

/*! @brief RSM HPM CAL interpolation structure. */
typedef struct
{
    uint16_t hpm_cal_factor_2442;   /*!< HPM_CAL_FACTOR used to calculate Kcal_2442, used in other algorithms */
    uint16_t eff_cal_freq;          /*!< The effective frequency for the HPM cal, based on COUNT1 and COUNT2 values. Used in other algorithms */
} xcvr_lcl_hpm_cal_interp_t;

/* Bitfield access macros for the uint8_t elements in the xcvr_lcl_fstep_t structure */
/*! @brief  RSM Step Format enumeration type. */
typedef enum
{
    XCVR_RSM_STEP_FCS           = 0U,  /*!< Frequency Check Sequence step type */
    XCVR_RSM_STEP_PK_PK         = 1U,  /*!<  Pk-Pk step type */
    XCVR_RSM_STEP_TN_TN         = 2U,  /*!<  Tn-Tn step type */
    XCVR_RSM_STEP_PK_TN_TN_PK   = 3U,   /*!<  Pk-Tn-Tn-Pk step type */
    XCVR_RSM_STEP_ERROR = 4U    /*!<  Error check enumeration */
}   XCVR_RSM_FSTEP_TYPE_T;

/*! @brief  RSM T_PM & T_FM selection enumeration type. */
typedef enum
{
    XCVR_RSM_T_PM0_SEL           = 0U, /*!< T_PM0 selected */
    XCVR_RSM_T_PM1_SEL           = 1U, /*!< T_PM1 selected */
    XCVR_RSM_T_PM2_SEL           = 2U, /*!< T_PM2 selected */
    XCVR_RSM_T_PM3_SEL           = 3U, /*!< T_PM3 selected */
    XCVR_RSM_T_FM0_SEL           = 0U, /*!< T_FM0 selected (when step is FCS type) */
    XCVR_RSM_T_FM1_SEL           = 1U, /*!< T_FM0 selected (when step is FCS type) */
}   XCVR_RSM_T_PM_FM_SEL_T;

/*! @brief  RSM antenna slot capture time selection enumeration type. */
typedef enum
{
    XCVR_RSM_T_CAPTURE_10_SEL           = 10U, /*!< 10usec slot time per antenna/TONE_EXT slot */
    XCVR_RSM_T_CAPTURE_20_SEL           = 20U, /*!< 20usec slot time per antenna/TONE_EXT slot */
    XCVR_RSM_T_CAPTURE_40_SEL           = 40U, /*!< 40usec slot time per antenna/TONE_EXT slot */
    XCVR_RSM_T_CAPTURE_80_SEL           = 80U, /*!< 80usec slot time per antenna/TONE_EXT slot */
    XCVR_RSM_T_CAPTURE_652_SEL         = 652U, /*!< 652usec slot time for special case of stable phase test */
    XCVR_RSM_T_CAPTURE_INVALID           /*!< Invalid setting */
}   XCVR_RSM_T_CAPTURE_SEL_T;

/* Fstep defines */
#define XCVR_RSM_HPM_CAL_MSB_MASK   (0xFU)
#define XCVR_RSM_HPM_CAL_MSB_SHIFT  (0U)
#define XCVR_RSM_HPM_CAL_MSB(x)     (((uint8_t)(((uint8_t)(x)) << XCVR_RSM_HPM_CAL_MSB_SHIFT)) & XCVR_RSM_HPM_CAL_MSB_MASK)
#define XCVR_RSM_STEP_FORMAT_MASK   (0x30U)
#define XCVR_RSM_STEP_FORMAT_SHIFT  (4U)
#define XCVR_RSM_STEP_FORMAT(x)     (((uint8_t)(((uint8_t)(x)) << XCVR_RSM_STEP_FORMAT_SHIFT)) & XCVR_RSM_STEP_FORMAT_MASK)
#define XCVR_RSM_T_PM_FM_SEL_MASK   (0xC0U)
#define XCVR_RSM_T_PM_FM_SEL_SHIFT  (6U)
#define XCVR_RSM_T_PM_FM_SEL(x)     (((uint8_t)(((uint8_t)(x)) << XCVR_RSM_T_PM_FM_SEL_SHIFT)) & XCVR_RSM_T_PM_FM_SEL_MASK)
/* RTT defines; All should be applied to the 4 octets of RTT data assembled to a uint32_t */
#define XCVR_RSM_RTT_VALID_MASK         (0x00000001U)
#define XCVR_RSM_RTT_VALID_SHIFT        (0U)
#define XCVR_RSM_RTT_FOUND_MASK         (0x00000002U)
#define XCVR_RSM_RTT_FOUND_SHIFT        (1U)
#define XCVR_RSM_RTT_CFO_MASK           (0x0003FFFCU)
#define XCVR_RSM_RTT_CFO_SHIFT          (2U)
#define XCVR_RSM_RTT_INT_ADJ_MASK       (0x000C0000U)
#define XCVR_RSM_RTT_INT_ADJ_SHIFT      (18U)
#define XCVR_RSM_RTT_HAM_DIST_SAT_MASK  (0x00300000U)
#define XCVR_RSM_RTT_HAM_DIST_SAT_SHIFT (20U)
#define XCVR_RSM_RTT_P_DELTA_MASK       (0xFFC00000U)
#define XCVR_RSM_RTT_P_DELTA_SHIFT      (22U)

/*! @brief PN short configuration storage structure. */
typedef struct
{
    uint32_t init_to_reflect_pn;                                /*!< PN for Initiator-to-Reflector packet */
    uint32_t reflect_to_init_pn;                                /*!< PN for Reflector-to-Initiator packet */
} xcvr_lcl_pn32_config_t;

/*! @brief PN long configuration storage structure. */
typedef struct
{
    uint32_t init_to_reflect_pn_lsb;                                /*!< PN for Initiator-to-Reflector packet least significant portion */
    uint32_t init_to_reflect_pn_msb;                                /*!< PN for Initiator-to-Reflector packet most significant portion */
    uint32_t reflect_to_init_pn_lsb;                                /*!< PN for Reflector-to-Initiator packet least significant portion */
    uint32_t reflect_to_init_pn_msb;                                /*!< PN for Reflector-to-Initiator packet most significant portion */
} xcvr_lcl_pn64_config_t;

/*! @brief Round Trip Time rawresults structure (data is packed together). */
typedef struct
{
    uint8_t rtt_data_b0_success;                                   /*!< LSB (7 bits of byte 0) of RTT data plus success bit in bit 0 */
    uint8_t rtt_data_b1;                                           /*!< Byte 1, next 8 bits of RTT data  */
    uint8_t rtt_data_b2;                                           /*!< Byte 2, next 8 bits of RTT data  */
    uint8_t rtt_data_b3;                                           /*!< Byte 3, next 8 bits of RTT data  */
} xcvr_lcl_rtt_data_raw_t;

/*! @brief Round Trip Time unpacked results structure  */
typedef struct
{
    bool rtt_vld;                                                   /*!<  */
    bool rtt_found;                                                 /*!< HARTT operation is done and a valid PN pattern was detected */
    int32_t cfo;                                                   /*!< The high accuracy CFO computed by the HARTT block through the CORDIC algorithm. Signed, reported in Hz. */
    uint8_t int_adj;                                                /*!< An integer adjustment of the timing which takes a value different of 0 when the early-late mechanism in the HARTT block chooses a peak different of the one chosen in the acquisition module (possible values are {-1,0,+1}).  */
    uint8_t ham_dist_sat;                                           /*!<  Computed Hamming distance saturated to 2 bits, format is ufix2.*/
    uint16_t p_delta;                                               /*!<  Difference between the squared correlation magnitude values, pm-pp provided by the HARTT block, format is sfix10En9.*/
} xcvr_lcl_rtt_data_t;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  /*KW47 specific defines */
/*! @brief NADM unpacked results structure  */
typedef struct
{
    uint8_t rssi_nb;
    uint8_t fm_corr_value;
    bool fm_corr_vld; 
    uint8_t fm_symb_err_value;
    bool fm_symb_err_vld;                                                 
    uint16_t nadm_pdelta;
} xcvr_lcl_nadm_data_t;
#endif

/*! @brief PLL Calibration results storage for HPM CAL. */
typedef struct
{
    uint16_t hpm_cal_val;                                           /*!< External HPM CAL value is uint16_t. */
#if (defined(CTUNE_MANUAL_CAL) && (CTUNE_MANUAL_CAL == 1))
    uint8_t ctune_cal_val;                                          /*!< External CTUNE value is uint8_t. */
#endif /* (defined(CTUNE_MANUAL_CAL) && (CTUNE_MANUAL_CAL == 1)) */
} xcvr_lcl_pll_cal_data_t;

/*! @brief Channel number type to specify frequency (according to the setting of the PLL's CHAN_MAP[HOP_TBL_CFG_OVRD] bitfield). */
typedef uint16_t channel_num_t;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
/*! @brief Structure for 1 set of TQI register settings (since different values apply for different capture slots)  */
typedef struct
{
    uint8_t iq_depth;                /*!< IQ_DEPTH register setting for TQI */
    uint8_t mag_depth;            /*!< MAG_DEPTH register setting for TQI */
    uint16_t t1;                        /*!< T1 threshold register setting for TQI */
    uint16_t t2;                        /*!< T2 threshold register setting for TQI */
} xcvr_lcl_tqi_entry_t;

/*! @brief Structure for storing register settings for TQI registers for a single data rate (values vary between 1Mbps and 2Mbps)  */
typedef struct
{
    xcvr_lcl_tqi_entry_t t_slot_10usec_tqi;  /*<! TQI register settings values for 10usec capture slot (per antenna) */
    xcvr_lcl_tqi_entry_t t_slot_20usec_tqi;  /*<! TQI register settings values for 20usec capture slot (per antenna) */
    xcvr_lcl_tqi_entry_t t_slot_40usec_tqi;  /*<! TQI register settings values for 40usec capture slot (per antenna) */
} xcvr_lcl_tqi_setting_tbl_t;
#endif

/* RSM-related register backup structure. Stores registers that must be changed in different peripherals for RSM to work. */
typedef struct
{
#if (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))        
    /* XCVR_MISC */
    uint32_t XCVR_MISC_DMA_CTRL;
    uint32_t XCVR_MISC_LCL_CFG0;
    uint32_t XCVR_MISC_LCL_CFG1;
    uint32_t XCVR_MISC_LCL_TX_CFG0;
    uint32_t XCVR_MISC_LCL_TX_CFG1;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
    uint32_t XCVR_MISC_RSM_CTRL6;
#else
    uint32_t XCVR_MISC_LCL_TX_CFG2;
#endif
    uint32_t XCVR_MISC_LCL_RX_CFG0;
    uint32_t XCVR_MISC_LCL_RX_CFG1;
    uint32_t XCVR_MISC_LCL_RX_CFG2;
    uint32_t XCVR_MISC_LCL_PM_MSB;
    uint32_t XCVR_MISC_LCL_PM_LSB;
    uint32_t XCVR_MISC_LCL_GPIO_CTRL0;
    uint32_t XCVR_MISC_LCL_GPIO_CTRL1;
    uint32_t XCVR_MISC_LCL_GPIO_CTRL2;
    uint32_t XCVR_MISC_LCL_GPIO_CTRL3;
    uint32_t XCVR_MISC_LCL_GPIO_CTRL4;
    uint32_t XCVR_MISC_LCL_DMA_MASK_DELAY;
    uint32_t XCVR_MISC_LCL_DMA_MASK_PERIOD;
    uint32_t XCVR_MISC_RSM_CTRL0;
    uint32_t XCVR_MISC_RSM_CTRL1;
    uint32_t XCVR_MISC_RSM_CTRL2;
    uint32_t XCVR_MISC_RSM_CTRL3;
    uint32_t XCVR_MISC_RSM_CTRL4;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    uint32_t XCVR_MISC_RSM_CTRL5;
    uint32_t XCVR_MISC_RSM_CTRL6;
    uint32_t XCVR_MISC_RSM_CTRL7;
    uint32_t XCVR_MISC_RSM_INT_ENABLE;
#endif
#endif /* (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1)) */

    /* XCVR_2P4GHZ_PHY */
    uint32_t XCVR_2P4GHZ_PHY_RTT_CTRL;
    uint32_t XCVR_2P4GHZ_PHY_RTT_REF;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    uint32_t XCVR_2P4GHZ_PHY_DMD_CTRL1;
    uint32_t XCVR_2P4GHZ_PHY_DMD_CTRL2;
#endif
    /* XCVR_TXDIG */
    uint32_t XCVR_TX_DIG_DATA_PADDING_CTRL;
    uint32_t XCVR_TX_DIG_DATA_PADDING_CTRL1;
    
    uint32_t XCVR_TX_DIG_GFSK_CTRL;
    
    uint32_t XCVR_TX_DIG_PA_CTRL;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    uint32_t PA_RAMP_TBL0;
    uint32_t PA_RAMP_TBL1;
    uint32_t PA_RAMP_TBL2;
    uint32_t PA_RAMP_TBL3;
#endif
    /* XCVR_PLL */
    uint32_t XCVR_PLL_DIG_HPM_BUMP;
    uint32_t XCVR_PLL_DIG_MOD_CTRL;
    uint32_t XCVR_PLL_DIG_CHAN_MAP;

    uint32_t XCVR_PLL_DIG_HPM_CTRL;
    
    uint32_t XCVR_PLL_DIG_HPM_SDM_RES;
    uint32_t XCVR_PLL_DIG_LPM_CTRL;
    uint32_t XCVR_PLL_DIG_LPM_SDM_CTRL1;
    uint32_t XCVR_PLL_DIG_DELAY_MATCH;
    uint32_t XCVR_PLL_DIG_TUNING_CAP_TX_CTRL;
    uint32_t XCVR_PLL_DIG_TUNING_CAP_RX_CTRL;
    
    uint32_t XCVR_PLL_OFFSET_CTRL;
    
    /* XCVR_TSM (used in PLL overrides) */
    uint32_t XCVR_TSM_OVRD0;
    uint32_t XCVR_TSM_OVRD1;
    uint32_t XCVR_TSM_OVRD2;
    uint32_t XCVR_TSM_OVRD3;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    uint32_t XCVR_TSM_OVRD4;
#endif
    
    /* XCVR_RXDIG */
    uint32_t XCVR_RX_DIG_CTRL1;
    uint32_t XCVR_RX_DIG_DFT_CTRL;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
    uint32_t XCVR_RX_DIG_AGC_CTRL;
#endif
    
    uint32_t XCVR_RX_DIG_RCCAL_CTRL1;
        
    /* RADIO_CTRL */
    uint32_t RADIO_CTRL_RF_CTRL;
} rsm_reg_backup_t;

typedef uint32_t tsm_u32_backup_array_t[NUM_TSM_U32_REGISTERS]; /*!< Backup array of uint32_t to store entire TSM register set. */

/* RSM interrupts are only supported on the NBU CPU */
#if defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)
/*! @brief Define callback function pointer type for RSM interrupt.
 *
 * This callback function is called in the RSM interrupt handle.
 *
 * @param userData Data available from callback.
 *
 * @note This function pointer type is only available for the NBU CPU as the CM33 CPU does not have connection to RSM interrupt line
 */
typedef void (*rsm_sw_callback)(void *userData, bool abort_flag, uint32_t CSRval);

/*! @brief RSM handler structure*/
typedef struct
{
    rsm_sw_callback user_callback; /*!< Callback function */
    void *userData;                /*!< User data available from callback */
} rsm_sw_handler_t;

/*! @brief RSM IRQ enable/disable masks; Intended to be OR'd together to enable or disable multiple interrupts at a time. */
#define  XCVR_LCL_RSM_IRQ_EN_ALL_BITS  (XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_EN_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_EN_MASK | \
                                XCVR_MISC_RSM_CSR_RSM_IRQ_FC_EN_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_EN_MASK | \
                                XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_EN_MASK) /* all of the interrupt ENABLE bits. Useful also in creating an error check mask */
#define  XCVR_LCL_RSM_IRQ_STAT_ALL_BITS (XCVR_MISC_RSM_CSR_RSM_IRQ_IP1_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_IP2_MASK | \
                                XCVR_MISC_RSM_CSR_RSM_IRQ_FC_MASK | XCVR_MISC_RSM_CSR_RSM_IRQ_EOS_MASK | \
                                XCVR_MISC_RSM_CSR_RSM_IRQ_ABORT_MASK) /* all of the interrupt STATUS bits. Useful also in creating an error check mask */

#endif /* defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)) */

#define OFFSET_NEG_1MHZ 0x100U /* HOP_TBL_CFG_OVRD format #2 numerator offset for -1MHz */
#define MAKE_MAPPED_CHAN_OVRD2(hadm_chan, output) \
        uint16_t mapped_chan_num = hadm_chan>>1U; \
        if ((hadm_chan & 0x1U) == 0x1U) /* original HADM channel was an odd number */ \
        { \
            mapped_chan_num++; /* go to next channel up (2MHz higher) to allow -1MHz to hit the target channel */ \
            mapped_chan_num |= (uint16_t)(OFFSET_NEG_1MHZ << 7U); /* Apply -1MHz */ \
        } \
        output = mapped_chan_num;


        
extern xcvr_lcl_hpm_cal_interp_t hpm_cal_2442_data;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef GCOV_DO_COVERAGE /* local except when testing code coverage */
xcvrLclStatus_t XCVR_LCL_RsmCheckSeqLen(uint16_t length, uint16_t maxlen);
bool XCVR_LCL_RsmCheckDmaDuration(uint16_t dma_duration, XCVR_RSM_SQTE_RATE_T rate, XCVR_RSM_AVG_WIN_LEN_T avg_win);
xcvrLclStatus_t XCVR_LCL_RsmCheckDmaMask(const xcvr_lcl_rsm_config_t * rsm_settings_ptr, xcvrLclStatus_t status_in);
uint8_t XCVR_LCL_CalcAdcOffset(uint8_t adc_offset_s7, uint8_t dig_corr_s8);
#endif /* !defined(GCOV_DO_COVERAGE) */


#if defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)   /* RSM interrupts are only supported on the NBU CPU */
/*!
 * @brief Register a callback from upper layers for the RSM interrupt.
 *
 * This function registers a callback from the upper layers for the radio to call when RSM interrupt occurs.
 *
 * @param[in] fptr  The function pointer to a RSM callback.
 *
 * @note This function is only available for the NBU CPU as the CM33 CPU does not have connection to RSM interrupt line
 */
void XCVR_LCL_RsmRegisterCb (const rsm_sw_handler_t * user_rsm_handler); /* allow upper layers to provide RSM callback */


/*!
 * @brief Enable or disable the RSM interrupts.
 *
 * This function allows individually enabling and disabling the RSM interrupts.
 *
 * @param[in] mask  The OR'd mask of multiple interrupts to enable or disable. The mask must use the RSM CSR enable bits in their proper position from the register header file.
 * @param[in] irq_enabled  True == the interrupt will be enabled, false == the interrupt will be disabled.
 *
 * @note This function is only available for the NBU CPU as the CM33 CPU does not have connection to RSM interrupt line
 */
bool XCVR_LCL_RsmIrqEnDis (uint32_t mask, bool irq_enabled);

#endif /* defined(KW45B41Z82_NBU_SERIES) || defined(KW45B41Z83_NBU_SERIES)) */


/*!
 * @brief Function to validate the settings structure for Ranging State Machine prior to calling initialization.
 *
 * This function validates the Ranging State Machine (RSM) initialization settings structure. It is implemented as a separate function in order to simplify the init
 * routine and also to allow for validation ahead of time in case the init routine is in a critical timing path.
 *
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 *
 * @return The status of the validation.
 *
 */
xcvrLclStatus_t XCVR_LCL_ValidateRsmSettings(const xcvr_lcl_rsm_config_t * rsm_settings_ptr);

/*!
 * @brief Function to initialize the PLL for RSM ranging operation.
 *
 * This function initializes any customized PLL settings for RSM operaiton. It is intended to be used to as a helper for both ::XCVR_LCL_RsmInit() and for
 * ::XCVR_LCL_CalibratePll() to put the PLL in proper configuration before RSM operations.
 *
 * @param rate the data rate to be used in RSM operation.
 *
 * @return The status of the init process.
 *
 * @note This routine modifies a number of registers which must be saved and then restored before changing back to normal operation. The ::XCVR_LCL_RsmDeInit() function
 * is intended to return to the prior settings at the end of RSM operation. If interim restoration is needed then ::XCVR_LCL_RsmPLLBackup() and ::XCVR_LCL_RsmPLLRestore() can be used.
 */
 void XCVR_LCL_RsmPLLInit(XCVR_RSM_SQTE_RATE_T rate);

/*!
 * @brief Function to backup  the PLL settings modified by XCVR_LCL_RsmPLLInit.
 *
 * This function backs up the contents of PLL regsiters before the customized PLL settings for RSM operation.
 *
 * @param reg_backup_ptr the pointer to a settings structure for register backup.
 *
 * @note This routine backs up the registers modified by ::XCVR_LCL_RsmPLLInit().
 */
xcvrLclStatus_t XCVR_LCL_RsmPLLBackup(rsm_reg_backup_t * reg_backup_ptr);

/*!
 * @brief Function to restore the PLL to prior settings after overrides are no longer needed.
 *
 * This function restores prior settings after any customized PLL settings for RSM operaiton.
 *
 * @param reg_backup_ptr the pointer to a settings structure for register restore.
 *
 * @note This routine restores the registers modified by ::XCVR_LCL_RsmPLLInit().
 */
xcvrLclStatus_t XCVR_LCL_RsmPLLRestore(const rsm_reg_backup_t * reg_backup_ptr);

/*!
 * @brief Function to initialize support for Ranging State Machine.
 *
 * This function initializes the Ranging State Machine (RSM), TSM, TX_DIG, GEN4PHY, and other registers before TX or RX operations using the RSM module.
 * This function calls helper functions to implement some of the TSM timing
 *
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 *
 * @return The status of the init process.
 *
 * @note This routine modifies a number of registers which must be restored before changing back to normal operation. The ::XCVR_LCL_RsmDeInit() function
 * is intended to return to the prior settings.
 */
xcvrLclStatus_t XCVR_LCL_RsmInit(const xcvr_lcl_rsm_config_t * rsm_settings_ptr);


/*!
 * @brief Function to de-initialize support for Ranging State Machine.
 *
 * This function resets the registers touched by ::XCVR_LCL_RsmInit() to allow the radio to return to normal operating modes. After this routine
 * is called, the RSM will be disabled and the TSM, TX_DIG, GEN4PHY will all be returned to prior settings.
 *
 */
void XCVR_LCL_RsmDeInit(void);

/*!
 * @brief Function to cleanly stop completed or abort ongoing Ranging State Machine operations.
 *
 * @param abort_rsm Selects whether this is a stop or abort request. True == abort, False == stop.
 *
 * This function aborts any ongoing RSM operations prior to the normal end of sequence. It is also used to stop the RSM after normal completion
 * of a RSM sequence.
 *
 * @note This routine performs a wait for the RSM state machine to return to IDLE state to ensure it is safe to clear the abort bit or clear RX or TX enables.
 * @note This routine must be called every time that RSM is started via ::XCVR_LCL_RsmGo() as it restores registers that are reprogrammed by the
 * ::XCVR_LCL_RsmGo() routine to support the RSM. Failure to this will cause the normal Bluetooth or GENFSK radio operations to malfunction.
 */
void XCVR_LCL_RsmStopAbort(bool abort_rsm);

/*!
 * @brief Function to start the Ranging State Machine.
 *
 * This function starts the RSM in the desired (RX or TX) role. If the RSM has a hardware trigger selected then the RSM will start upon the assertion of that trigger. If the RSM has a
 * software trigger selected then the RSM will start immediately. For both hardware and software triggers, the RSM_TRIG_DLY will control the time delay to the actual start of the RSM sequence.
 *
 * @param role Role for the RSM, Initiator/Reflector or PD/MD, depending on op_mode setting.
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 *
 * @return The status of the init process.
 *
 * @pre The RSM must be initialized, TSM reconfigured, and all RAM based tables programmed before calling this routine.
 *
 * @note Every RSM sequence that is started must be either aborted or stopped in order to prevent unexpected executions of the RSM sequence due to hardware triggers. The function
 * ::XCVR_LCL_RsmStopAbort() must be used to perform the stop or abort (as needed) for every execution of the go routine.
 */
xcvrLclStatus_t XCVR_LCL_RsmGo(XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t * rsm_settings_ptr);

/*!
 * @brief Function to snapshot the TSM timing registers to a storage structure.
 *
 * This function captures the state of the TSM timing registers and is used both before and after calling ::XCVR_LCL_RsmInit(). When
 * called before, it stores the state of the TSM for normal operation. When called after, it stores the state of the TSM for RSM operations.
 * Both cases are intended to allow for later restore of a specific set of TSM timings without performing additional calculations.
 *
 * @param curr_tsm_timings the for storage of the TSM timing values read from hardware registers.
 *
 * @return The status of the TSM timings read process.
 *
 */

xcvrLclStatus_t XCVR_LCL_GetTsmTimings(xcvr_lcl_tsm_config_t *backup_tsm_timings);


#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
/*!
 * @brief Function to snapshot or restore  the TSM timing registers to a storage structure in PKT RAM using fast IPS DMA.
 *
 * This function captures tor restores he state of the TSM timing registers with a fast DMA block storing to one of the PKT RAM banks.
 * It is used to quickly backup and restore TSM registers without the slow access time penalty for normal XCVR register accesses.
 *
 * @param desc_ptr pointer to the descriptor structure describing the sequences of registers to backup to PKT RAM. 
 * @param num_entries Number of entries in the desciptor structure, not including the zero ending descriptor. 
 * @param pkt_ram_bank Selects the TX or RX packet RAM bank to use for storage of the download. 
 * @param pkt_ram_index_offset The word offset into the PKT RAM bank where the descriptors should be loaded and the register backup stored. 
 * @param restore If set to true then restore from PKT RAM is done. Otherwise, backup is to PKT RAM done.
 *
 * @return The status of the TSM timings fast backup process.
 *
 * @pre The caller must have called the XCVR_ValidateFastPeriphDescrip() routine to validate the descriptor structure is correct and PKT RAM limits are not exceeded. The function 
 * XCVR_FastPeriphDescrip_WordCount() can be used to get the word count that will be used in the PKT RAM for storage of the data.
 *
 */
xcvrLclStatus_t XCVR_LCL_FastBackupRestore(uint32_t * desc_ptr, uint8_t num_entries, PKT_RAM_BANK_SEL_T pkt_ram_bank, uint16_t pkt_ram_index_offset, bool restore);
#define TSM_DESCRIP_COUNT (2)
extern const uint32_t tsm_fast_descrip_comp[TSM_DESCRIP_COUNT] ; /* Make descriptor visible extenally for validation */
#if (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))        
#define PLL_DESCRIP_COUNT (18)
#else
#define PLL_DESCRIP_COUNT (15)
#endif /* (defined(BACKUP_RSM_LCL) && (BACKUP_RSM_LCL==1))         */
extern const uint32_t pll_rsm_fast_descrip_comp[PLL_DESCRIP_COUNT] ; /* Make descriptor visible extenally for validation */
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  */


/*!
 * @brief Function to apply the new (pre-calculated) TSM timing values or restore timings from backup for RSM operations .
 *
 * This function updates a subset of new TSM timing register values for specific RSM operations. The input structure is the relevant
 *  TSM register changes only. It is also used to restore TSM timings from a backup structure (replaces deprecated function 
 * XCVR_LCL_SetTsmTimings).
 *
 * @param new_tsm_timings the pointer to a structure for output of TSM timing values.
 *
 * @return The status of the TSM timings update process.
 *
 */
xcvrLclStatus_t XCVR_LCL_ReprogramTsmTimings(const xcvr_lcl_tsm_config_t * new_tsm_timings);

/*!
 * @brief Function to compute FAST Start rx and tx jump point to achieve desired T_FC/T_IP.
 *
 * This function write the TSM FAST control register to skip a part of the WU sequence depending on the rsm role
 * to achieve T_FC and T_IP timing. The TSM is not compatible with independant T_IP1 and T_IP2, so min(T_IP1, T_IP2)
 * is programmed.
 *
 * @param role The RSM role ( inititor or reflector ).
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 *
 * @return The status of the function (xcvrLclStatus_t).
 *
 */
xcvrLclStatus_t XCVR_LCL_Set_TSM_FastStart( XCVR_RSM_RXTX_MODE_T role, const xcvr_lcl_rsm_config_t * rsm_settings_ptr);

/*!
 * @brief Function to backup the state of various XCVR registers changed by RSM init.
 *
 * This function backs up registers in multiple XCVR blocks to store their state before or during RSM operations for
 * later restoration.
 *
 * @param reg_backup_ptr the pointer to a settings structure for register backup.
 *
 * @return The status of the backup.
 *
 */
xcvrLclStatus_t XCVR_LCL_RsmRegBackup(rsm_reg_backup_t * reg_backup_ptr);

/*!
 * @brief Function to restore the state of various XCVR registers changed by RSM init.
 *
 * This function restores registers to multiple XCVR blocks to replace their state to what it was before or during RSM operations
 *
 * @param reg_backup_ptr the pointer to a settings structure for register restore.
 *
 * @return The status of the restore.
 *
 */
xcvrLclStatus_t XCVR_LCL_RsmRegRestore(const rsm_reg_backup_t * reg_backup_ptr);

/*!
 * @brief Function to program the Frequency Step structure in Packet RAM for RSM operations.
 *
 * This function programs the Frequency Step structure with the frequency value, CTUNE value, HPM cal value
 * T_PM setting and step format for each frequency step in a RSM sequence.
 *
 * @param fstep_settings the pointer to a structure for frequency step programming, assumed to be an array.
 * @param num_steps the number of frequency steps to program.
 *
 * @return The status of the frequency step programming process.
 *
 */
xcvrLclStatus_t XCVR_LCL_SetFstepRam(const xcvr_lcl_fstep_t * fstep_settings, uint16_t num_steps);

/*!
 * @brief Function to program the Short PseudoNoise structure in Packet RAM for RSM operations.
 *
 * This function programs the short pseudonoise structure with Initiator and Reflector 32-bit values.
 *
 * @param pn_values the pointer to a structure for frequency step programming, assumed to be an array.
 * @param num_steps the number of pseudonoise steps to program.
 *
 * @return The status of the pseudonoise step programming process.
 *
 */
xcvrLclStatus_t XCVR_LCL_SetPnRamShort(const xcvr_lcl_pn32_config_t * pn_values, uint16_t num_steps);

#if defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1)
/*!
 * @brief Function to program the Long PseudoNoise structure in Packet RAM for RSM operations.
 *
 * This function programs the short pseudonoise structure with Initiator and Reflector 64-bit values.
 *
 * @param pn_values the pointer to a structure for pseudonoise step programming, assumed to be an array.
 * @param num_steps the number of pseudonoise steps to program.
 *
 * @return The status of the pseudonoise step programming process.
 *
 */
xcvrLclStatus_t XCVR_LCL_SetPnRamLong(const xcvr_lcl_pn64_config_t * pn_values, uint16_t num_steps);
#endif /* defined(SUPPORT_RSM_LONG_PN) && (SUPPORT_RSM_LONG_PN == 1) */

/*!
 * @brief Function to read the CTUNE_BEST_DIFF values from Packet RAM for RSM operations.
 *
 * This function reads the CTUNE_BEST_DIFF values from Packet RAM to be used for tuning .
 *
 * @param ctune_results the pointer to an array of uint8_t to store CTUNE_BEST_DIFF values.
 * @param num_steps the number of CTUNE_BEST_DIFF values to read.
 *
 * @return The status of the CTUNE_BEST_DIFF values read process.
 *
 */
xcvrLclStatus_t XCVR_LCL_GetCtuneResults(uint8_t * ctune_results, uint16_t num_steps);

/*!
 * @brief Function to unpack Round Trip Time data structures read from Packet RAM for RSM operations.
 *
 * This function unpacks the raw data structure that was read from RTT Packet RAM and places the data into a formatted structure.
 *
 * @param rtt_results the packed RTT results data read from Packet RAM.
 * @param rtt_unpacked the structure to contain the unpacked RTT data.
 * @param rate the data rate for the captured RTT packet, for conversion to Hz of the CFO.
 *
 * @return The status of the CTUNE_BEST_DIFF values read process.
 *
 */
xcvrLclStatus_t XCVR_LCL_UnpackRttResult(const xcvr_lcl_rtt_data_raw_t * rtt_results, xcvr_lcl_rtt_data_t * rtt_unpacked, XCVR_RSM_SQTE_RATE_T rate);

/*!
 * @brief Function to read the Round Trip Time values from Packet RAM for RSM operations.
 *
 * This function reads the Round Trip Time values from Packet RAM to be used for calculating distance..
 *
 * @param rtt_results the pointer to an array of structures to store Round Trip Time values in raw format.
 * @param num_steps the number of Round Trip Time values to read.
 *
 * @return The status of the Round Trip Time values read process.
 *
 */
xcvrLclStatus_t XCVR_LCL_GetRttResults(xcvr_lcl_rtt_data_raw_t * rtt_results, uint16_t num_steps);

/*!
 * @brief Function to format the input parameters into a single Fstep structure.
 *
 * This formats the channel number, CTUNE data, HPM CAL factor, step format and T_PM_SEL values into the Fstep structure.
 *
 * @param fstep_entry the pointer to a structure for frequency step programming.
 * @param channel_num the channel number to be applied.
 * @param ctune the ctune value to be applied.
 * @param hpm_cal the HPM CAL value to be applied.
 * @param step_format the step format to be applied.
 * @param t_pm_sel the T_PM timing selection to be applied.
 *
 * @return The status of the function, success or error.
 *
 */
xcvrLclStatus_t XCVR_LCL_MakeFstep(xcvr_lcl_fstep_t * fstep_entry,
                                        uint16_t channel_num,
                                        uint8_t ctune,
                                        uint16_t hpm_cal,
                                        XCVR_RSM_FSTEP_TYPE_T step_format,
                                        XCVR_RSM_T_PM_FM_SEL_T t_pm_sel);



#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
/*!
 * @brief Function to validate the settings for the LCL block before configuration of that block.
 *
 * This function initializes the LCL block for Channel Sounding for KW47. 
 *
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 * @param t_capture the selection of the capture time for a single antenna slot.
 *
 * @return The status of the settings check.
 *
 */
xcvrLclStatus_t XCVR_LCL_ValidateLclSettings(xcvr_lcl_rsm_config_t * rsm_settings_ptr, XCVR_RSM_T_CAPTURE_SEL_T t_capture);

/*!
 * @brief Function to initialize the LCL block for Channel Sounding use cases.
 *
 * This function initializes the LCL block for Channel Sounding. 
 *
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 * @param t_capture the selection of the capture time for a single antenna slot.
 * @param toneAntennaIDs_p pointer to a list of antenna ids mappings for RF_GPO output.
 *
 * @return The status of the init process.
 *
 */
xcvrLclStatus_t XCVR_LCL_ConfigLclBlock(xcvr_lcl_rsm_config_t * rsm_settings_ptr, XCVR_RSM_T_CAPTURE_SEL_T t_capture, uint8_t *toneAntennaIDs_p, bool ena_antsw_pa_ramping);
#else

/*!
 * @brief Function to initialize the LCL block for Channel Sounding use cases.
 *
 * This function initializes the LCL block for Channel Sounding. 
 *
 * @param rsm_settings_ptr the pointer to a settings structure for RSM initialization.
 * @param ant_slot_time the selection of the capture time for a single antenna slot.
 *
 * @return The status of the init process.
 *
 */xcvrLclStatus_t XCVR_LCL_ConfigLclBlock(xcvr_lcl_rsm_config_t * rsm_settings_ptr,  XCVR_RSM_T_CAPTURE_SEL_T  ant_slot_time);
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */    

/*!
 * @brief Function to perform PLL calibrations to capture CTUNE and HPM data for later use.
 *
 * This function performs a number of RX warmups at a list of frequencies in order to calibrate the PLL and
 * capture the CTUNE and HPM calibrations for later use. Any number of frequencies is supported.
 * This routine has multiple usage scenarios, such as brute force calibration at every target frequency or alternatively
 * calibration at a smaller number of points to feed an interpolation algorithm for faster generation of cal values.
 * Both the CTUNE and HPM calibration values are captured during this process.
 *
 * @param freq_list the pointer to an array of frequencies to tune to and capture calibration values. This must follow the format
 * defined in the PLL's CHAN_MAP[HOP_TBL_CFG_OVRD] bitfield and match the current setting of the field. Frequency values need not be sorted.
 * @param cal_results the pointer to an array of structures to store the calibration results in raw format.
 * @param num_freqs the number of frequencies at which calibration should be performed.
 * @param update_curve_fit selects for updating the curve fit data storage when the calibration frequency is 2442MHz. Error is triggered if 2442MHz is not in the frequency list.
 * @param rate the data rate to be used in RSM operation.
 *
 * @return The status of the PLL calibration process.
 *
 */
xcvrLclStatus_t XCVR_LCL_CalibratePll(const channel_num_t * hadm_chan_idx_list, xcvr_lcl_pll_cal_data_t * cal_results, uint16_t num_freqs, bool update_curve_fit, XCVR_RSM_SQTE_RATE_T rate);

/*!
 * @brief Function to calculate interpolated values for HPM CAL based on previously calculated equations for approximations.
 *
 * This function calculates interpolated values for HPM CAL values at different frequencies based on data captured in the XCVR_LCL_CalibratePll() routine when the update_curve_fit
 * parameter is set to true. These equations are generated from a single HPM_CAL measurement at 2442MHz and are used to generate many additional
 * frequency points data without needing additional PLL calibrations.
 *
 * @param hadm_chan_idx_list the pointer to an array of HADM channel index values that correspond to the cal_results array. Values may be in any order.
 * @param cal_results the pointer to an array of structures to store the interpolation results in raw format matching the frequency array.
 * @param num_freqs the number of frequencies at which interpolation should be performed.
 * @param hpm_cal_interp the HPM CAL value and effective cal frequency to be used for the basis of the interpolation.
 *
 * @return The status of the interpolation process.
 *
 * @pre XCVR_LCL_RsmInit() must be called successfully prior to using this routine.
 * @note The interpolation assumes data in the curve fit description structure is valid and performs no check on the input data other than NULLPTR check.
 *
 */
xcvrLclStatus_t XCVR_LCL_InterpolatePllCal(const uint16_t * hadm_chan_idx_list, xcvr_lcl_pll_cal_data_t * cal_results, const uint16_t num_freqs, const xcvr_lcl_hpm_cal_interp_t * hpm_cal_interp);

/*!
 * @brief Function to trigger the start of a manual DCOC calibration prior to RSM operations.
 *
 * This function triggers a RX warmup and a manual DCOC process in the receiver. The result of this process is that the DCOC registers are properly programmed for use in RSM operations.
 *
 * @param rate - the data rate.
 *
 * @return The RF_CTRL register value for use in the XCVR_LCL_CalibrateDcocComplete() routine.
 *
 */
void XCVR_LCL_CalibrateDcocStart(XCVR_RSM_SQTE_RATE_T rate);

/*!
 * @brief Function to trigger a manual DCOC calibration prior to RSM operations.
 *
 * This function triggers a RX warmup and a manual DCOC process in the receiver. The result of this process is that the DCOC registers are properly programmed for use in RSM operations.
 *
 * @return The status of the compensation process.
 *
 * @note This routine implements a wait for completion of the DCOC process to ensure the calibration works properly. The DCOC is configured to happen within the TSM sequence
 * so a wait for RX WU is ensuring calibration completed. Other code can be run in between the XCVR_LCL_CalibrateDcocStart() and XCVR_LCL_CalibrateDcocComplete() in order to
 * optimize system timing. In this case, the wait for RX WU should fall through immediately. 
 */
xcvrLclStatus_t XCVR_LCL_CalibrateDcocComplete(void);

/*!
 * @brief Function to configure TSM override signals to support keeping phase continuous in PLL.
 *
 * This function sets up the TSM overrides needed to keep the PLL TX and RX divider states consistent across multiple Channel Sounding events/subevents.
 *
 */
void XCVR_LCL_ContPhaseOvrd(void);

/*!
 * @brief Function to release TSM override signals used to keep phase continuous or measure non-continuous phase in PLL.
 *
 * This function releases all the TSM overrides configured in either of ::XCVR_LCL_ContPhaseOvrd() or ::XCVR_LCL_SetupManualDcoc()
 * or ::XCVR_LCL_EnablePhaseMeasure().
 *
 */
void XCVR_LCL_AllPhaseRelease(void);

/*!
 * @brief Function to override DCOC ADC and DAC to improve amplitude for measurement of  the current PLL phase angle.
 *
 * This function forces the DCOC
 * ADC and DAC to offsets calculated from a previous DCOC calibration status result in order to improve the amplitude
 * of the resulting phase measurement.
 *
 * @return returns the value for DCOC_CTRL2 to be used to override the ADC and DAC for DCOC
 * 
 * @note This routine requires a valid DCOC calibration prior be completed and the status results still available. This function
 * should be called only once before a calling XCVR_LCL_EnablePhaseMeasure(void). It returns a uint32_t value that is to be used for 
 * the DCOC_CTRL2 override (by calling XCVR_LCL_OverrideDcoc). XCVR_LCL_AllPhaseRelease() must be
 * called after all phase measurement is complete in order to remove the ADC and DAC overrides that are setup in XCVR_LCL_SetupManualDcoc().
 *
 * @code
#define RESID_TEST_NUM		(10U)
    typedef struct
    {
        int8_t i_resid;
        int8_t q_resid;
    } xcvr_dc_resid_t;
    xcvrLclStatus_t lclstatus;
    static xcvr_dc_resid_t dc_resid_continuous[RESID_TEST_NUM];
    static xcvr_dc_resid_t dc_resid_discontinuous[RESID_TEST_NUM];
    uint8_t count;
    uint32_t temp_resid;
    uint32_t temp_dcoc_ctrl2_val;
    uint32_t temp_rf_ctrl_val;

    // Test #1 - Measuring phase with continuous phase overrides
    temp_rf_ctrl_val = XCVR_LCL_CalibrateDcocStart(XCVR_RSM_RATE_1MBPS);
    lclstatus = XCVR_LCL_CalibrateDcocComplete(temp_rf_ctrl_val);
    assert(lclstatus == gXcvrLclStatusSuccess);
    temp_dcoc_ctrl2_val = XCVR_LCL_SetupManualDcoc();
    XCVR_LCL_OverrideDcoc(temp_dcoc_ctrl2_val, TRUE); // Make DCOC overrides active
    XCVR_LCL_ContPhaseOvrd();  // Enable overrides to keep phase continuous
    for (count =0;count<RESID_TEST_NUM;count++)
    {
        XCVR_LCL_EnablePhaseMeasure(); // Sets overrides required to measure phase
        temp_rf_ctrl_val = XCVR_LCL_CalibrateDcocStart(XCVR_RSM_RATE_1MBPS);
        lclstatus = XCVR_LCL_CalibrateDcocComplete(temp_rf_ctrl_val);
        assert(lclstatus == gXcvrLclStatusSuccess);
        temp_resid = XCVR_RX_DIG->DCOC_DIG_CORR_RESULT;
        lclstatus = XCVR_LCL_ProcessPhaseMeasure(&dc_resid_continuous[count].i_resid, &dc_resid_continuous[count].q_resid, temp_resid);
        assert(lclstatus == gXcvrLclStatusSuccess);
    }
    XCVR_LCL_AllPhaseRelease(); // Release all overrides, both measurement and continuous phase enablement
    XCVR_LCL_OverrideDcoc(temp_dcoc_ctrl2_val, FALSE); // Release overrides

    // Test #2 - Measuring phase without continous phase overrides
    temp_rf_ctrl_val = XCVR_LCL_CalibrateDcocStart(XCVR_RSM_RATE_1MBPS);
    lclstatus = XCVR_LCL_CalibrateDcocComplete(temp_rf_ctrl_val);
    assert(lclstatus == gXcvrLclStatusSuccess);
    temp_dcoc_ctrl2_val = XCVR_LCL_SetupManualDcoc();
    XCVR_LCL_OverrideDcoc(temp_dcoc_ctrl2_val, TRUE); // Make DCOC overrides active
    for (count =0;count<RESID_TEST_NUM;count++)
    {
        XCVR_LCL_EnablePhaseMeasure(); // Sets overrides required to measure phase
        temp_rf_ctrl_val = XCVR_LCL_CalibrateDcocStart(XCVR_RSM_RATE_1MBPS);
        lclstatus = XCVR_LCL_CalibrateDcocComplete(temp_rf_ctrl_val);
        assert(lclstatus == gXcvrLclStatusSuccess);
        temp_resid = XCVR_RX_DIG->DCOC_DIG_CORR_RESULT;
        lclstatus = XCVR_LCL_ProcessPhaseMeasure(&dc_resid_discontinuous[count].i_resid, &dc_resid_discontinuous[count].q_resid, temp_resid);
        assert(lclstatus == gXcvrLclStatusSuccess);
    }
    XCVR_LCL_AllPhaseRelease(); // Release all overrides, both measurement and continuous phase enablement
    XCVR_LCL_OverrideDcoc(temp_dcoc_ctrl2_val, FALSE); // Release overrides
    temp_rf_ctrl_val = XCVR_LCL_CalibrateDcocStart(XCVR_RSM_RATE_1MBPS);
    lclstatus = XCVR_LCL_CalibrateDcocComplete(temp_rf_ctrl_val);
    assert(lclstatus == gXcvrLclStatusSuccess);
   @endcode
 */
uint32_t XCVR_LCL_SetupManualDcoc(void);

/*!
 * @brief Function to c override DCOC DAC and ADC to manual values.
 *
 * This function forces the DCOC_CTRL2 register to a new value and sets the override bits to make these manual values active
 *
 * @param dcoc_ctrl2_value override value for both ADC and DAC values
 * @param override if TRUE, performs the override; if FALSE, releases the override
 *
 * @note
 */
void XCVR_LCL_OverrideDcoc(uint32_t dcoc_ctrl2_value, bool override);

/*!
 * @brief Function to configure TSM override signals to measure the current PLL phase angle.
 *
 * This function sets up the TSM overrides needed to prepare for measuring the current PLL phase. It must be called every time
 * before a phase measurement is performed (i.e. before the DCOC calibration)
 *
 * @note
 */
void XCVR_LCL_EnablePhaseMeasure(void);

/*!
 * @brief Function performs a measurement of the current PLL phase angle.
 *
 * This function uses the DC residual as a measurement of the PLL phase angle for comparison with a prior measurement.
 *
 * @param i_resid pointer to the location to store the DC residual measurement for the I channel
 * @param q_resid pointer to the location to store the DC residual measurement for the Q channel
 * @param dc_resid_val the 32 bit contents of the XCVR_RX_DIG->DCOC_DIG_CORR_RESULT register for processing. This is passed as a parameter
 * to ensure testability of the function.
 *
 * @return the status of the measurement process
 *
 * @note The I and Q residual values are intended to be used to calculate a phase angle as a measurement of the PLL phase. The absolute value
 * is not meaningful but comparison between two different measurements may be used to determine if the PLL has the same or inverted phase
 * (from one measurement to the next).
 * After completion of this routine, a new DCOC calibration must be triggered in order to restore the state of the DCOC for subsequent operations.
 */
xcvrLclStatus_t XCVR_LCL_ProcessPhaseMeasure(int8_t * i_resid, int8_t * q_resid, uint32_t dc_resid_val);

/*!
 * @brief Function to apply a Carrier Frequency Offset compensation to the PLL for RSM operations.
 *
 * This function takes and input Carrier Frequency Offset compensation in Hz and calculates an offset to apply to the PLL numerator to implement the compensation
 *
 * @param cfo_in_hz the signed CFO to compensate in Hz. Positive CFO value == negative PLL frequency adjustment; Negative CFO value == positive PLL frequency adjustment; Zero == no adjustment.
 *
 * @return The status of the compensation process.
 *
 *
 */
xcvrLclStatus_t XCVR_LCL_RsmCompCfo(int32_t cfo_in_hz);

/*!
 * @brief Function to read back a Carrier Frequency Offset compensation from the PLL for RSM operations.
 *
 * This function returns the Carrier Frequency Offset compensation in Hz based on the value in the PLL_OFFSET_CTRL register.
 *
 * @return The amount of compensation currently programmed, in Hz.
 *
 * @note Due to the use of integer divide in this routine and in ::XCVR_LCL_RsmCompCfo(), the returned value may not be exactly the original requested compensation.
 */
int32_t XCVR_LCL_RsmReadCfoComp(void); /* Read back current CFO compensation value (Hz) */


/*!
 * @brief Function to map from a HADM channel index to the channel number value to use for FSTEP programming.
 *
 * This function maps from a HADM channel index to the channel number value used to program the FSTEP channel number field according to HOP_TBL_CFG_OVRD format #3.
 *
 * @param hadm_chan_index The HADM standard specified channel index, ranging from 0 to 78 to select frequencies of 2402MHz up to 2480MHz.
 * @param fstep_chan_num the pointer to the location where the channel number value should be stored.
 *
 * @return The status of mapping process.
 *
 */
xcvrLclStatus_t XCVR_LCL_MakeChanNumFromHadmIndex(uint8_t hadm_chan_index, uint16_t * fstep_chan_num);

/*!
 * @brief Function to return the dma buffer size and dma sequence length (us) of a configured rsm sequence.
 *
 * This function reads the rsm register configuration and iterate over the step settings to compute the total sequence buffer
 * size needed for dma iq capture and the dma sequence length in us ( to be used in antena switch configuration ).
 * The dma buffer size corresponds to the number of iq samples to be captured.
 *
 * @param[in]  fstep_settings frequency step list pointer (xcvr_lcl_fstep_t), the first element of a frequency step configuration array.
 * @param[in]  num_steps The number of step to be used
 * @param[in]  role (XCVR_RSM_RXTX_MODE_T) rsm mode : XCVR_RSM_TX_MODE or XCVR_RSM_RX_MODE
 * @param[out] dma_buffer_size (uint16_t) address to store the dma buffer size corresponding to the current configuration
 * @param[out] dma_seq_length_us (uint16_t) address to store the dma sequence length in us corresponding to the current configuration
 * @param[in] ant_cnt (uint8_t) count of antenna active in this sequence.
 * @return The status of the function (xcvrLclStatus_t).
 *
 */
xcvrLclStatus_t XCVR_LCL_GetRSMCaptureBufferSize(const xcvr_lcl_fstep_t * fstep_settings,
                                                 uint8_t num_steps,
                                                 XCVR_RSM_RXTX_MODE_T role,
                                                 uint16_t * dma_buffer_size,
                                                 uint16_t * dma_seq_length_us,
                                                 uint8_t ant_cnt);

/*!
 * @brief Function to count the number of FCS, Pk-Pk , and Pk-Tn-Tn-Pk steps within an overall frequency step list.
 *
 * This function counts the number of FCS, Pk-Pk , and Pk-Tn-Tn-Pk steps within an overall frequency step list in
 * order to verify that PN RAM and RTT RAM will not be overrun. This is due to the difference in the Fstep RAM
 * length and the length of the PN and RTT RAMs.
 *
 * @param fstep_settings the pointer to a structure for frequency step programming, assumed to be an array.
 * @param num_steps the number of frequency steps in the input list.
 *
 * @return The count of FCS, Pk-Pk , and Pk-Tn-Tn-Pk steps within the overall frequency step list. A value of 0xFFU is returned in error cases.
 *
 */
uint8_t XCVR_LCL_CountPnRttSteps(const xcvr_lcl_fstep_t * fstep_settings, uint16_t num_steps);

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)  
/*!
 * @brief Function to enable PLL clock switching for semi-coherent PLL functions.
 *
 * This function configures PLL clock swicthing and sigma delta code adjustment to compensate for analog cycle wraps.
 *
 * @param mode Themode of operation. 0==disabled; 1==fast mode; 2=slow with code adjustment.
 *
 * @return The status of the clock switch setting.
 *
 */
xcvrLclStatus_t XCVR_LCL_EnaLpmClkSwitch(uint8_t mode);

/*!
 * @brief Function to enable PLL PIC functionality for semi-coherent PLL functions.
 *
 * This function configures PLL PIC feature in different modes .
 *
 * @param mode The mode of operation.
 * @param slow_bw_reduction Selects whether to apply bandwidth reduction in Fast-Slow mode (only).
 *
 */
xcvrLclStatus_t XCVR_LCL_EnaPic(XCVR_RSM_PIC_MODE_TYPE_T mode, bool slow_bw_reduction);

/*!
 * @brief Function to enable PLL divider synchronization for semi-coherent PLL functions.
 *
 * This function configures PLL dividers under TSM control (rather than static signals keeping dividers running continuously) .
 *
 * @param enable Select whether to enable or disable the feature.
 *
 */
void XCVR_LCL_EnaDividerSync(bool enable);
#endif

#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)  && (RF_OSC_26MHZ == 0) */

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* NXP_XCVR_LCL_CTRL_H */
