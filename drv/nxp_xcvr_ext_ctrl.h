/*
 * Copyright 2018-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef NXP_XCVR_EXT_CTRL_H
/* clang-format off */
#define NXP_XCVR_EXT_CTRL_H
/* clang-format on */

#include "fsl_common.h"
#include "nxp2p4_xcvr.h"

/*!
 * @addtogroup xcvr_pa_coex PA, FEM, FAD, Localization and Co-existence Routines
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef TEST_BUILD_COEX
#define TEST_BUILD_COEX     (0)  /* Testing ability to build coex support software */
#endif /* TEST_BUILD_COEX */

#if defined(gMWS_UseCoexistence_d)
#if gMWS_UseCoexistence_d
#include "MWS.h"
#endif /* gMWS_UseCoexistence_d */
#else
#define gMWS_UseCoexistence_d 0
#endif /* defined(gMWS_UseCoexistence_d) */

#if (TEST_BUILD_COEX) /* Only used to test building coexistence without the MWS include file */
#define gMWS_CoexRfActiveAssertTime_d       (100U)
#define gMWS_CoexPrioSignalTime_d           (40U)
/* Supported coexistence models */
#define gMWS_Coex_Status_Prio_d             (0)
#define gMWS_Coex_Prio_Only_d               (1)
/* Coexistence model to be handled */
#ifndef gMWS_Coex_Model_d
#define gMWS_Coex_Model_d    gMWS_Coex_Status_Prio_d
#endif /* gMWS_Coex_Model_d */
#endif /* TEST_BUILD_COEX */

/* Maximum value accepted for some LCL configuration */
#define XCVR_LCL_SAMPLES_MAX_LEN 32U

 /*! @brief  Coexistence arbitration priority settings. */
typedef enum
{
    XCVR_COEX_LOW_PRIO  = 0U,
    XCVR_COEX_HIGH_PRIO = 1U
}   XCVR_COEX_PRIORITY_T;

/*! @brief  PA/FEM control mode settings. */
typedef enum
{
    XCVR_ANTX_SINGLE_MODE       = 0U,
    XCVR_ANTX_DUAL_MODE         = 1U
}   XCVR_ANTX_MODE_T;

/*! @brief  PA/FEM GPIO vs TSM settings. */
typedef enum
{
    XCVR_FAD_TSM_GPIO           = 0U,
    XCVR_FAD_STATE_MACHINE      = 1U
}   XCVR_FAD_NOT_GPIO_MODE_T;

/*! @brief  PA/FEM RX/TX polarity settings. */
typedef enum
{
    XCVR_FAD_ACTIVE_HIGH        = 0U,
    XCVR_FAD_ACTIVE_LOW         = 1U
}   XCVR_RX_TX_POLARITY_MODE_T;

/*! @brief  PA/FEM RX/TX polarity settings. */
typedef enum
{
    XCVR_FAD_OVRD_SEL_ANT_A     = 0U,
    XCVR_FAD_OVRD_SEL_ANT_B     = 1U
}   XCVR_FAD_OVRD_ANT_A_B_SEL_MODE_T;

/*!@brief TX & RX PA/FEM function settings for PA/FEM control over coexistence pins.  */
typedef enum
{
    NO_FUNC             = 0U,
    TX_FUNC             = 1U,
    RX_FUNC             = 2U,
    BOTH_TX_RX_FUNC     = 3U
} tx_rx_coex_pin_func_t;

/*! @brief PA/FEM  configuration structure. */
typedef struct
{
    XCVR_ANTX_MODE_T op_mode;                           /*!< operating mode for the PA/FEM interface */
    uint8_t ant_sel_pins_enable;                        /*!< control whether ANT_A & ANT_B pads should be active (0=Disabled, 1=Enabled) */
    uint8_t tx_rx_switch_pins_enable;                   /*!< control whether TX and RX_SWITCH pads should be active (0=Disabled, 1=Enabled) */
    uint8_t high_z_enable;                              /*!< control whether FEM/PA  pads should use High Z (0=Disabled, 1=Enabled) */
    uint8_t use_fad_state_machine;                      /*!< control whether FAD state machine is active (0=Disabled, 1=Enabled) */
    XCVR_FAD_NOT_GPIO_MODE_T ant_a_pad_control;         /*!< control whether ANT_A pad should be controlled by FAD state machine or TSM GPIO  */
    XCVR_FAD_NOT_GPIO_MODE_T ant_b_pad_control;         /*!< control whether ANT_B pad should be controlled by FAD state machine or TSM GPIO  */
    XCVR_FAD_NOT_GPIO_MODE_T tx_switch_pad_control;     /*!< control whether TX_SWITCH pad should be controlled by FAD state machine or TSM GPIO  */
    XCVR_FAD_NOT_GPIO_MODE_T rx_switch_pad_control;     /*!< control whether RX_SWITCH pad should be controlled by FAD state machine or TSM GPIO  */
    uint8_t pa_tx_wu;                                   /*!< Number of usec to start external PA warmup ahead of internal PA warmup (ramp) start  */
    uint8_t pa_tx_wd;                                   /*!< Number of usec to start external PA warmdown ahead of internal PA warmdown (ramp) completion  */
    uint8_t lna_rx_wu;                                  /*!< Number of usec to start external LNA warmup ahead of internal LNA warmup  start  */
    uint8_t lna_rx_wd;                                  /*!< Number of usec to start external LNA warmdown ahead of internal LNA warmdown completion  */
    XCVR_RX_TX_POLARITY_MODE_T tx_switch_pol_control;   /*!< control whether TX_SWITCH pad should be active high or low  (0=Active HIGH, 1=Active LOW) */
    XCVR_RX_TX_POLARITY_MODE_T rx_switch_pol_control;   /*!< control whether RX_SWITCH pad should be active high or low  (0=Active HIGH, 1=Active LOW) */
} xcvr_pa_fem_config_t;


/*! @brief status return codes for LCL module. */
typedef enum
{
    gXcvrLclStatusSuccess           = 0U,   /*!< Success */
    gXcvrLclStatusInvalidArgs       = 1U,   /*!< Invalid arguments */
    gXcvrLclStatusInvalidLength     = 2U,   /*!< Invalid length */
    gXcvrLclStatusInvalidDuration   = 3U,   /*!< Invalid duration */
    gXcvrLclStatusFail                      /*!< Fail */
} xcvrLclStatus_t;

/*! @brief modes for LCL module. Also used by COEX module. */
typedef enum
{
    gLclRxTxNone        = 0U, /*!< neither RX nor TX mode */
    gLclTxMode          = 1U,   /*!< RX mode */
    gLclRxMode          = 2U,   /*!< TX mode */
    gLclRxTxMode        = 3U, /*!< RX & TX mode */
    gLclRxTxInvalid     = 4U /*!< RX & TX invalid */
} lclRxTxMode_t;

/*! @brief RX trigger types for LCL module. */
typedef enum
{
    lclRxTriggerSoftware        = 0U,    /*!< Software Trigger */
    lclRxTriggerPatternFound    = 1U,    /*!< Localization control: pattern found */
    lclRxTriggerCrcComplete     = 2U,    /*!< CRC Complete */
    lclRxTriggerCrcPass         = 3U,    /*!< CRC Pass */
    lclRxTriggerCtePresent      = 4U,    /*!< GenericLL: cte_present */
    lclRxTriggerAccessAddressFound = 5U, /*!< Gen4 PHY: aa_fnd_to_ll */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    lclTxTriggerRsmLclRxTrig    = 6U,     /*!< Ranging Sequence Manager lcl_rx_trigger */
#endif /* #if defined(RADIO_IS_GEN_4P5) */
    lclRxTriggerInvalid                   /*!< Invalid */
} lclRxTrigger_t;

/*! @brief TX trigger types for LCL module. */
typedef enum
{
    lclTxTriggerSoftware        = 0U,     /*!< Software Trigger */
    lclTxTriggerPatternFound    = 1U,     /*!< Localization control: pattern found */
    lclTxTriggerCrcComplete     = 2U,     /*!< CRC Complete */
    lclTxTriggerPaWuComplete    = 3U,     /*!< PA Warmup Complete */
    lclTxTriggerRbmeTxDonePre   = 4U,     /*!< RBME TX Done Pre (last bit is out of RBME) */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    lclTxTriggerBtleCteEn       = 5U,     /*!< Bluetooth LE CTE Enable signal  */
    lclTxTriggerRsmLclTxTrig    = 6U,     /*!< Ranging Sequence Manager lcl_tx_trigger */
#endif /* #if defined(RADIO_IS_GEN_4P5) */
    lclTxTriggerInvalid                   /*!< Invalid */
} lclTxTrigger_t;

/*!< Antenna switching T_SW values supported. Enum values are intended to be cast to uint8_t for calculations. */
typedef enum 
{
    lclTSw_0usec        = 0U,       /*!< 0 microsecond antenna switching time */
    lclTSw_1usec        = 1U,       /*!< 1 microsecond antenna switching time */
    lclTSw_2usec        = 2U,       /*!< 2 microsecond antenna switching time */
    lclTSw_4usec        = 4U,       /*!< 4 microsecond antenna switching time */
    lclTSw_10usec      = 10U,     /*!< 10 microsecond antenna switching time */
    lclTSw_Invalid                       /*!< Invalid */
} lclTSw_t;

/*!< T_PM_SLOT values supported. These values are used to set the phase measurement time for a single antenna slot. Enum values are intended to be cast to uint8_t for calculations. */
typedef enum 
{
    lclTPm_20usec        = 20U,       /*!< 20 microsecond phase measurement time per antenna slot */
    lclTPm_40usec        = 40U,       /*!< 40 microsecond phase measurement time per antenna slot */
    lclTPm_Invalid                       /*!< Invalid */
} lclTPm_t;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
/*! @brief Mask selection for DMA/DBG engines during RX data capture (applies to both DMA and DBG RAM captures */
typedef enum
{
    lclRxMaskPct                = 1U,       /*!< mask to select PCTs in capture */
    lclRxMaskFmRx            = 2U,       /*!< mask to select Frequency Compensation (FM_RX) in capture */
    lclRxMaskPmRx            = 4U,       /*!< mask to select RX Tones (PM_RX) in capture */
    lclRxMaskDtRxt            = 8U,       /*!< mask to select RX Packets (DT_RX) in capture */
    lclRxMaskInvalid                        /*!< Invalid */
} lclRxDmaDbgSignalVldMask_t;

/*! @brief Mask selection for DMA/DBG engines during RX data capture (applies to both DMA and DBG RAM captures */
typedef enum
{
    lclDmaMaskNoCenter                = 0U,       /*!< No centering of the DMA mask */
    lclDmaMask1usecCenter           = 1U,       /*!< Centered 1usec long window */
    lclDmaMask2usecCenter           = 2U,       /*!< Centered 2usec long window */
    lclDmaMask4usecCenter           = 3U,       /*!< Centered 4usec long window */
    lclDmaMask8usecCenter           = 4U,       /*!< Centered 8usec long window */
    lclDmaMask16usecCenter           = 5U,       /*!< Centered 16usec long window */
    lclDmaMask32usecCenter           = 6U,       /*!< Centered 32usec long window */
    lclDmaMask64usecCenter           = 7U,       /*!< Centered 64usec long window */
    lclDmaMask128usecCenter         = 8U,       /*!< Centered 128usec long window */
    lclDmaMask256usecCenter         = 9U,       /*!< Centered 256usec long window */
    lclDmaMask512usecCenter         = 10U,       /*!< Centered 512usec long window */
    lclDmaMask1024usecCenter       = 11U,       /*!< Centered 1024usec long window */
    lclDmaMaskInvalidCenter           = 12U,       /*!< Invalid window */
} lclRxDmaCenterMask_t;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
#define LCL_NUM_HI_LO_INTERVALS     (4)
#else
#define LCL_NUM_HI_LO_INTERVALS     (1)
#endif /* #if defined(RADIO_IS_GEN_4P5) */

/*! @brief Antenna Switching configuration structure for LCL module. */
typedef struct
{
    uint16_t numberOfSwitchesInPattern;         /*!< Number of antenna switches in localization pattern */
    bool lantSwInvert;                          /*!< Selects polarity on lant_sw signal */
    bool lantSwWiggle;                          /*!< Enables SW wiggle output on lant_sw signal */
    lclTxTrigger_t txTrig;                      /*!< TX trigger configuration */
    lclRxTrigger_t rxTrig;                      /*!< RX trigger configuration */

    uint8_t  samplesPerInterval;                /*!< Number of samples per interval */
    uint8_t  lowPeriodDuration[LCL_NUM_HI_LO_INTERVALS];   /*!< Duration of switching period (low) in intervals */
    uint8_t  highPeriodDuration[LCL_NUM_HI_LO_INTERVALS];  /*!< Duration of sampling period (high) in intervals */
    uint16_t triggerDelay;                      /*!< Delay between trigger and antenna switching start in intervals */
    uint8_t  triggerOffset;                     /*!< Extra delay between trigger and antenna switching start in samples */
} lclAntennaSwitchingPatternConfig_t;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
/*! @brief Antenna Switching LUT configuration structure for LCL module. */
typedef struct
{
    uint8_t antennaLut[32];                    /*!< Look-up table for the state of the GPIO signals for antenna control output. */
    uint8_t lutWrapPoint;                      /*!< The number of the last LUT entry that should be used before wrapping back to 0. */
} lclAntennaSwitchingLutConfig_t;
#endif /* #if defined(RADIO_IS_GEN_4P5) */


/*! @brief Pattern Matching Number of Bytes Enumerator. */
typedef enum
{
    lclPatternMatchNumBytes4 = 0U,       /*!< pattern to match 4 bytes long */
    lclPatternMatchNumBytes5 = 1U,       /*!< pattern to match 5 bytes long */
    lclPatternMatchNumBytes6 = 2U,       /*!< pattern to match 6 bytes long */
    lclPatternMatchNumBytes8 = 3U,       /*!< pattern to match 8 bytes long */
    lclPatternMatchNumBytesInvalid = 4U, /*!< Invalid pattern match length */
} lclPatternMatchNumBytes_t;

/*! @brief Pattern matching configuration structure for LCL module. */
typedef struct
{
    lclPatternMatchNumBytes_t  numBytes;         /*!< The pattern can be 4, 5, 6, or 8 bytes long */
    uint8_t  pattern[8];                         /*!< Pattern arrray */
} lclPatternMatchConfig_t;

/*! @brief BLE CTE Localization Method Enumerator. */
typedef enum
{
    lclBleCteMethodAoA,
    lclBleCteMethodAoD,
    lclBleCteMethodInvalid
} lclBleCteLocMethod_t;

/*! @brief BLE CTE Localization Role Enumerator. */
typedef enum
{
    lclBleCteRoleTransmitter,
    lclBleCteRoleReceiver,
    lclBleCteRoleInvalid
} lclBleCteLocRole_t;

/*! @brief BLE CTE datarate (1M/2M). */
typedef enum
{
    lclBleCteRate1M,
    lclBleCteRate2M,
    lclBleCteRateInvalid
} lclBleCteLocRate_t;







/*! @brief Define callback function for LANT_SW.
 *
 * This callback function is called in the LANT_SW interrupt handle.
 *
 * @param userData Data available from callback.
 */
typedef void (*lant_sw_callback)(void *userData);

/*! @brief LANT_SW handler structure*/
typedef struct
{
    lant_sw_callback user_callback; /*!< Callback function */
    void *userData;                 /*!< User data available from callback */
} lant_sw_handler_t;

/* Coexistence enums and structs */

/*! @brief status return codes for COEX module. */
typedef enum
{
    gXcvrCoexStatusSuccess           = 0U,   /*!< Success */
    gXcvrCoexStatusInvalidArgs       = 1U,   /*!< Invalid arguments */
    gXcvrCoexStatusInvalidTime       = 2U,   /*!< Invalid timing setting */
    gXcvrCoexStatusIncompleteConfig  = 3U,   /*!< Incomplete configuration for the requested operation. */
    gXcvrCoexStatusFail                      /*!< Fail */
} xcvrCoexStatus_t;

/*! @brief Selections for RF_ACTIVE source. */
typedef enum
{
    coexRfactRfmc       = 0U,               /*!< Select RFMC to control RF_ACTIVE */
    coexRfactTsmLl      = 1U,               /*!< Select TSM or LL to control RF_ACTIVE */
    coexRfactBtClkReq   = 2U,               /*!< Select bt_clk_req from Bluetooth LL to control RF_ACTIVE */
    coexRfactInvalid                        /*!< Invalid RF_ACTIVE source selection */
} coexRfactSrc_t;

/*! @brief Selections for RF_NOT_ALLOWED input pin. Setting is applied to RFNA_IBE bitfield. */
typedef enum
{
    coexRfNotAllowPinDis        = 0U,               /*!< Disable RF_NOT_ALLOWED input pin */
    coexRfNotAllowPinPta16      = 1U,               /*!< Select PTA16 for RF_NOT_ALLOWED input pin. */
    coexRfNotAllowPinPta17      = 2U,               /*!< Select PTA17 for RF_NOT_ALLOWED input pin. */
    coexRfNotAllowPinPta22      = 3U,               /*!< Select PTA22 for RF_NOT_ALLOWED input pin. */
    coexRfNotAllowPinPtc7       = 4U,               /*!< Select PTC7 for RF_NOT_ALLOWED input pin. */
    coexRfNotAllowPinPtd6       = 5U,               /*!< Select PTD6 for RF_NOT_ALLOWED input pin. */
    coexRfNotAllowPinInvalid                        /*!< Invalid entry flag for RF_NOT_ALLOWED input pin. */
} coexRfNotAllowPin_t;

/*! @brief Selections for RF_NOT_ALLOWED link layer enable bits. These bits are defined to be OR'd together to use in the RF_NOT_ALLOWED_EN bitfield */
typedef enum
{
    coexRfNotAllowNoLLSel       = 0U,               /*!< No LL selected for RF_NOT_ALLOWED  */
    coexRfNotAllowLLBluetooth   = 1U,               /*!< Enable bit for RF_NOT_ALLOWED to Bluetooth LL  */
    coexRfNotAllowLLZigbee      = 4U,               /*!< Enable bit for RF_NOT_ALLOWED to Zigbee LL  */
    coexRfNotAllowLLGenfsk      = 8U                /*!< Enable bit for RF_NOT_ALLOWED to GENFSK LL  */
} coexRfNotAllowLL_t;

/*! @brief COEX signal inversion configuration structure*/
typedef struct
{
    bool rfna_invert;                               /*!< When set to true, inverts the RF_NOT_ALLOWED signal in RFMC muxing logic */
    bool rfact_invert;                              /*!< When set to true, inverts the RF_ACTIVE signal in RFMC muxing logic */
    bool rfstat_invert;                             /*!< When set to true, inverts the RF_STATUS signal in RFMC muxing logic */
    bool rfpri_invert[2];                           /*!< When set to true, inverts the RF_PRIORITY signals in RFMC muxing logic */
} coexRfSignalInvert_t;

/*! @brief RF_NOT_ALLOWED configuration structure*/
typedef struct
{
    coexRfNotAllowPin_t rfna_pin_enable;            /*!< Input pin for RF_NOT_ALLOWED. */
    uint8_t link_layer_rfna_select;      /*!< Enable bits for RF_NOT_ALLOWED signal to LLs. Value is the OR'd together of any ::coexRfNotAllowLL_t entries.  */
} coexRfNotAllowedConfig_t;

/*! @brief RF_ACTIVE TSM controlled use case configuration structure*/
typedef struct
{
    uint8_t rf_act_extend;                  /*!< RF_ACTIVE remains asserted this many microseconds after the end of TSM RX or TX sequences. Must be <=255. */
    uint8_t rf_act_tx_advance;              /*!< RF_ACTIVE aserts this many microseconds before the end of TX warm up. If this value is > than the number of microseconds before end of TX warm  ramp up in the TSM timing sequence then an error will be returned. */
    uint8_t rf_act_rx_advance;              /*!< RF_ACTIVE aserts this many microseconds before the RX digital is enabled. If this value is > than the number of microseconds before end of RX warm  ramp up in the TSM timing sequence then an error will be returned. */
} coexRfActiveTsmConfig_t;

/*! @brief RF_ACTIVE RFMC controlled use case configuration structure*/
typedef struct
{
    bool deassert_when_tsm_idle;            /*!< When set to true, RFMS will deassert RF_ACTIVE if the TSM is idle. Otherwise deassertion is on next low power entry. */
    uint8_t wakeup_delay;                   /*!< RF_ACTIVE asserts this many 32KHz ref. clocks after the XO is enabled.  Must be less <= 63. */
} coexRfActiveRfmcConfig_t;

/*! @brief RF_STATUS configuration structure*/
typedef struct
{
    uint8_t rf_stat_tx_advance;             /*!< RF_STATUS aserts this many microseconds before the end of TX warm up. If this value is > than the number of microseconds before end of TX warm  ramp up in the TSM timing sequence then an error will be returned. */
} coexRfStatusConfig_t;

/*! @brief RF_PRIORITY configuration structure*/
typedef struct
{
    uint8_t rf_pri_tx_advance;              /*!< RF_PRIORITY aserts this many microseconds before the end of TX warm up. If this value is > than the number of microseconds before end of TX warm  ramp up in the TSM timing sequence then an error will be returned. */
    uint8_t rf_pri_rx_advance;              /*!< RF_PRIORITY aserts this many microseconds before the RX digital is enabled. If this value is > than the number of microseconds before end of RX warm  ramp up in the TSM timing sequence then an error will be returned. */
    bool rf_pri_on_rf_stat;                 /*!< When set to true, RF_PRIORITY signal is muxed on  the RF_STATUS signal. */
} coexRfPriorityConfig_t;




/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN <= 400)/* Only applies for Gen 3.5 and Gen 4.0, not Gen 4.5 */
xcvrStatus_t XCVR_CoexistenceInit(void);
xcvrStatus_t XCVR_CoexistenceSetPriority(XCVR_COEX_PRIORITY_T rxPriority, XCVR_COEX_PRIORITY_T txPriority);
void XCVR_CoexistenceSaveRestoreTimings(uint8_t saveTimings);
#else   /* Gen 4.5 Coexistence interface */
/*!
 * @brief Function to setup common muxing selections for all coexistence signals to choose what controls the signals.
 *
 * This function chooses which module controls the coexistence signals ans selects the RF_ACTIVE signal source.
 *
 * @param tsm_controls_coex selects whether the TSM or LL blocks control coexistence. True indicates TSM control, false indicates LL control.
 * @param rf_active_src selects the module to control RF_ACTIVE.
 * @param config_ptr the pointer to a settings structure for setting the signal inversion controls. NULLPTR is a valid input and is used to indicate that no changes should be made to the signal inversion state.
 *
 * @return The status of the selection process.
 *
 * @warning The operation of TSM controlled coexistence signalling will not function properly during Localization/Ranging operations using
 * the Ranging Sequence Manager (RSM). Contol should be given to the LL signals or the coexistence signals should be overridden by software.
 */
xcvrCoexStatus_t XCVR_COEX_SelectController(bool tsm_controls_coex, coexRfactSrc_t rf_active_src, const coexRfSignalInvert_t * config_ptr);

/*!
 * @brief Function to initialize support for RF_NOT_ALLOWED coexistence signal .
 *
 * This function initializes the RF_NOT_ALLOWED coexistence signal.
 *
 * @param config_ptr the pointer to a settings structure for RF_NOT_ALLOWED initialization.
 *
 * @return The status of the init process.
 *
 */
xcvrCoexStatus_t XCVR_COEX_RfNotAllowedInit(const coexRfNotAllowedConfig_t * config_ptr );

/*!
 * @brief Function to override the state of the  RF_NOT_ALLOWED coexistence signal .
 *
 * This function overrides the state of the RF_NOT_ALLOWED coexistence signal. It allows software to force the radio into an RF not allowed state.
 *
 * @param override_en the enable for RF_NOT_ALLOWED override.
 * @param override_val the value to apply for the RF_NOT_ALLOWED override. Ignored when override_en is false.
 *
 */
void XCVR_COEX_RfNotAllowedOvrd(bool override_en, bool override_val);

/*!
 * @brief Function to read the state RF_NOT_ALLOWED coexistence signal .
 *
 * This function reads  the RF_NOT_ALLOWED coexistence signal, either from the raw state indication or a latched state bit.
 *
 * @param read_latched_status When true, the state of RF_NOT_ALLOWED is read from a latched state bit. When false, the state is read from the raw state bit.
 *
 * @return The status selected RF_NOT_ALLOWED state bit.
 *
 */
bool XCVR_COEX_GetRfNotAllowedStat(bool read_latched_status);

/*!
 * @brief Function to initialize support for RF_ACTIVE coexistence signal for control via TSM.
 *
 * This function initializes the RF_ACTIVE coexistence signal for control via TSM (instead of via RFMC or the LL signals).
 *
 * @param config_ptr the pointer to a settings structure for RF_ACTIVE initialization.
 *
 * @return The status of the init process.
 *
 */
xcvrCoexStatus_t XCVR_COEX_RfActiveTsmInit(const coexRfActiveTsmConfig_t * config_ptr);

/*!
 * @brief Function to initialize support for RF_ACTIVE coexistence signal for control via RFMC.
 *
 * This function initializes the RF_ACTIVE coexistence signal for control via RFMC (instead of via TSM or the LL signals).
 *
 * @param config_ptr the pointer to a settings structure for RF_ACTIVE initialization.
 *
 * @return The status of the init process.
 *
 * @note The amount of timing advance for both TX and RX is limited to the respective TX and RX end of sequence values
 * in the TSM->END_OF_SEQ register. That is, the TSM cannot assert the RF_ACTIVE signal before the TSM is even triggered
 * by the LL to start counting.
 *
 */
xcvrCoexStatus_t XCVR_COEX_RfActiveRfmcInit(const coexRfActiveRfmcConfig_t * config_ptr);


/*!
 * @brief Function to override the state of the  RF_ACTIVE coexistence signal .
 *
 * This function overrides the state of the RF_ACTIVE coexistence signal. It allows software to force the state of the RF activity indication.
 *
 * @param override_en the enable for RF_ACTIVE override.
 * @param override_val the value to apply for the RF_ACTIVE override. Ignored when override_en is false.
 *
 */
void XCVR_COEX_RfActiveOvrd(bool override_en, bool override_val);

/*!
 * @brief Function to initialize support for RF_STATUS coexistence signal for control via TSM.
 *
 * This function initializes the RF_STATUS coexistence signal for control via TSM (instead of via the LL signals).
 *
 * @param config_ptr the pointer to a settings structure for RF_STATUS initialization.
 *
 * @return The status of the init process.
 *
 * @note The amount of timing advance for both TX and RX is limited to the respective TX and RX end of sequence values
 * in the TSM->END_OF_SEQ register. That is, the TSM cannot assert the RF_STATUS signal before the TSM is even triggered
 * by the LL to start counting.
 *
 */
xcvrCoexStatus_t XCVR_COEX_RfStatusTsmInit(const coexRfStatusConfig_t * config_ptr);

/*!
 * @brief Function to initialize support for RF_PRIORITY coexistence signal for control via TSM.
 *
 * This function initializes the RF_PRIORITY coexistence signal for control via TSM (instead of via the LL signals).
 *
 * @param config_ptr the pointer to a settings structure for RF_PRIORITY initialization.
 *
 * @return The status of the init process.
 *
 * @note The amount of timing advance for both TX and RX is limited to the respective TX and RX end of sequence values
 * in the TSM->END_OF_SEQ register. That is, the TSM cannot assert the RF_PRIORITY signal before the TSM is even triggered
 * by the LL to start counting.
 *
 */
xcvrCoexStatus_t XCVR_COEX_RfPriorityTsmInit(const coexRfPriorityConfig_t * config_ptr);

/*!
 * @brief Function to set the RX and TX  RF_PRIORITY levels.
 *
 * This function sets the priority level to be indicated for RX and TX operations.
 *
 * @param rxPriority the priority level for RX operations.
 * @param txPriority the priority level for TX operations.
 *
 * @return The status of the priority setting process.
 *
 */
xcvrCoexStatus_t XCVR_COEX_SetPriority(XCVR_COEX_PRIORITY_T rxPriority, XCVR_COEX_PRIORITY_T txPriority);

/*!
 * @brief Function to override the state(s) of the RF_STATUS &  RF_PRIORITY coexistence signals.
 *
 * This function overrides the state of the RF_STATUS &  RF_PRIORITY coexistence signals. It allows software to force the state of either or both signals.
 *
 * @param override_en_stat the enable for RF_STATUS override.
 * @param override_val_stat the value to apply for the RF_STATUS override. Ignored when override_en_stat is false.
 * @param override_en_pri the enable for RF_PRIORITY override.
 * @param override_val_pri the value to apply for the RF_PRIORITY override. Ignored when override_en_stat is false.
 *
 */
void XCVR_COEX_RfStatPrioOvrd(bool override_en_stat, bool override_val_stat,bool override_en_pri, bool override_val_pri);

/*!
 * @brief Function to clear all local state variables controlling coexistence signals.
 *
 * This function clears all local state variables that control coexistence signals. It is intended to support easy reset of the state for testing.
 *
 */
void XCVR_COEX_ClearSavedState(void);


#endif /* defined(RADIO_IS_GEN_3P5)  || defined(RADIO_IS_GEN_4P0) */

/*!
 * @brief Function to initialize support for external PA and FEM module control.
 *
 * This function initializes the PA and FEM control register and timing registers before TX or RX operations using those modules.
 *
 * @param pa_fem_settings_ptr the pointer to a settings structure for PA/FEM initialization.
 *
 * @return The status of the init process.
 *
 */
xcvrStatus_t XCVR_ExternalFadPaFemInit(xcvr_pa_fem_config_t * pa_fem_settings_ptr);

/*!
 * @brief Function to de-initialize support for external PA and FEM module control.
 *
 * This function resets the PA and FEM control register and timing registers to allow them to be used for other purposes if required.
 *
 * @return The status of the init process.
 *
 */
xcvrStatus_t XCVR_ExternalFadPaFemDeInit(void);

/*!
 * @brief Function to over-ride the antenna selection when FAD is active .
 *
 * This function enables the over-ride of the FAD antenna selection and sets the software requested selection.
 *
 * @return The status of the over-ride process.
 *
 */
void XCVR_ExtAntOvrd(XCVR_FAD_OVRD_ANT_A_B_SEL_MODE_T antenna_sel);

/*!
 * @brief Function to release any over-ride the antenna selection and return the antenna to  FAD control .
 *
 * This function enables the over-ride of the FAD antenna selection and sets the software requested selection.
 *
 */
void XCVR_ExtAntRelease(void);

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN <= 400) /* Only applies for Gen 3.5 and Gen 4.0, not Gen 4.5 */
/*!
 * @brief Function to initialize support for external PA and FEM module control using Coexistence pins.
 *
 * This function initializes the PA and FEM control register and timing registers before TX or RX operations using those modules. It
 * sets up to use one or both of the RF_STATUS and RF_PRIORITY pins for PA/FEM control instead of coexistence.
 *
 * @param test_settings The pointer to a settings structure for PA/FEM initialization.
 * @param rf_status_func t The function to be applies on the RF_STATUS pin.
 * @param rf_priority_func The function to be applies on the RF_PRIORITY pin.
 * @note
 *  This function is intended to be used when ::XCVR_ExternalFadPaFemInit() cannot be used (if preservation of debugger connection
 *  is required.
 *  This function does NOT perform any pin muxing settings, that must be performed by a higher layer of software.
 *
 */
xcvrStatus_t XCVR_FadPaFemOnCoexInit(xcvr_pa_fem_config_t * test_settings, tx_rx_coex_pin_func_t rf_status_func, tx_rx_coex_pin_func_t rf_priority_func);

/*!
 * @brief Function to de-initialize support for external PA and FEM module control over coexistence pins.
 *
 * This function resets the PA and FEM control register and timing registers to allow them to be used for other purposes if required.
 *
 */
void XCVR_FadPaFemOnCoexDeInit(void);
#endif /* defined(RADIO_IS_GEN_3P5) || defined(RADIO_IS_GEN_4P0) */

/*!
 * @brief Handles LCL module programming for AoA/AoD antenna switching.
 *
 * @param mode mode (RX/AoA or TX/AoD) for Antenna Switching
 * @param pConfig pointer to configuration structure
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchInit(lclRxTxMode_t mode, const lclAntennaSwitchingPatternConfig_t *pConfig);

/*!
 * @brief Read LCL module settings for AoA/AoD antenna switching
 *
 * @param mode mode (RX/AoA or TX/AoD) for Antenna Switching
 * @param pConfig pointer to configuration structure
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchRead(lclRxTxMode_t mode, lclAntennaSwitchingPatternConfig_t *pConfig);

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
/*!
 * @brief Handles LCL module programming of DMA mask delays.
 *
 * @param MaskDelay value of the mask delay expressed in number of intervals
 * @param MaskDelayOff value of the mask delay offset expressed in number of samples
 * @param MaskRefPeriod value of the mask reference period expressed in number of intervals
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchDmaMaskSet(uint16_t MaskDelay, uint8_t MaskDelayOff, uint8_t MaskRefPeriod);

/*!
 * @brief Handles LCL module reading of DMA mask delays.
 *
 * @param pMaskDelay pointer to mask delay parameter
 * @param pMaskDelayOff pointer to mask delay offset parameter
 * @param pMaskRefPeriod pointer to mask reference period parameter
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchDmaMaskGet(uint16_t *pMaskDelay, uint8_t *pMaskDelayOff, uint8_t *pMaskRefPeriod);

/*!
 * @brief Handles LCL module programming for AoA/AoD antenna switching LUT configuration.
 *
 * @param pConfig pointer to configuration structure
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchLUTConfigure(const lclAntennaSwitchingLutConfig_t *pConfig);

/*!
 * @brief Read LCL module settings for AoA/AoD antenna switching LUT configuration.
 *
 * @param pConfig pointer to configuration structure
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchLUTRead(lclAntennaSwitchingLutConfig_t *pConfig);
#endif

/*!
 * @brief Enable LCL module settings for AoA/AoD antenna switching
 *
 * @param enTXmode enable/disable TX mode
 * @param enRXmode enable/disable RX mode
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchEn(bool enTXmode, bool enRXmode);

/*!
 * @brief Block RX/TX for AoA/AoD antenna switching
 *
 * @param blockTXmode block/unblock TX mode
 * @param blockRXmode block/unblock RX mode
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchBlock(bool blockTXmode, bool blockRXmode);

/*!
 * @brief Read LCL mode settings for AoA/AoD antenna switching
 *
 * @param mode mode (RX/AoA or TX/AoD) for Antenna Switching
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_AntennaSwitchModeRead(lclRxTxMode_t *mode);

/*!
 * @brief Handles LCL module programming for patterm matching
 *
 * @param mode mode (RX or TX) for patterm matching
 * @param pConfig pointer to configuration structure
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_PatternMatchInit(lclRxTxMode_t mode, const lclPatternMatchConfig_t *pConfig);

/*!
 * @brief Read LCL module programming for patterm matching
 *
 * @param pConfig pointer to configuration structure
 * @return the status of the request, success or invalid parameter
 */
xcvrLclStatus_t XCVR_LCL_PatternMatchRead( lclPatternMatchConfig_t *pConfig );

/*!
 * @brief Read LCL module mode for patterm matching
 *
 * @param mode mode (RX or TX) for patterm matching
 * @return the status of the request, success or invalid parameter
 *
 */
xcvrLclStatus_t XCVR_LCL_PatternMatchModeRead(lclRxTxMode_t *mode);

/*!
 * @brief Enable LCL module mode for patterm matching
 *
 * @param mode mode (RX or TX) for patterm matching
 *
 */
xcvrLclStatus_t XCVR_LCL_PatternMatchEn(lclRxTxMode_t mode);

/*!
 * @brief Disable LCL module mode for patterm matching
 *
 * @param mode mode (RX or TX) for patterm matching
 *
 */
xcvrLclStatus_t XCVR_LCL_PatternMatchDisable(lclRxTxMode_t mode);

/*!
 * @brief Forces a manual trigger to the LCL module.
 *
 */
void XCVR_LCL_SwTriggerAssert(void);

/*!
 *
 * @brief This function turns off the localization module.
 *        Affects both antenna switching and pattern matching modes.
 *
 */
void XCVR_LCL_DeInit(void);

/*!
 * @brief Function read whether the localization module is enabled or not.
 *
 * This function reads the status of the localization module.
 *
 * @return The status of the localization module.
 *
 */
bool XCVR_LCL_GetEn(void);


#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
#if defined (SUPPORT_AOA_AOD_IN_XCVR) && (SUPPORT_AOA_AOD_IN_XCVR == 1)

/*!
 * @brief Handle LCL module programming for BLE AoA/AoD use cases.
 *
 * This function configures the required LCL HW registers that are not controlled by the BLE LL in case of AoA/AOD .
 *
 * @param[in] BLEDataRate Localization method to handle (1Mbps or 2Mbps, see type).
 * @param[in] BLELocalizationMethod Localization method to handle (AoA or AOD, see type).
 * @param[in] BLELocalizationRole Role of the device in the method (Emitter or receiver, see type).
 * @param[in] Switching_Pattern_Length Length of the pattern.
 * @param[in] Antenna_IDs Switching pattern to apply (table where antenna 0 is specified as 0, antenna 1 as 1 and so on...).
 *            Must contain as many entries as specified by Switching_Pattern_Length
 * @param[in] DMA_Buffer_Address Address of the buffer where to copy I/Q samples
 * @param[in] DMA_Buffer_Size Buffer size (16bit words)
 *
 * @return Status.
 *
 */
xcvrLclStatus_t XCVR_LCL_BLEAoDAoAInit(lclBleCteLocRate_t BLEDataRate,
                            lclBleCteLocMethod_t BLELocalizationMethod,
                            lclBleCteLocRole_t BLELocalizationRole,
                            uint8_t Switching_Pattern_Length,
                            const uint8_t *Antenna_IDs
#ifdef DMACONFIG
                            , uint32_t DMA_Buffer_Address,
                            uint32_t DMA_Buffer_Size
#endif
                              );

/*!
 * @brief Turns off BLE AoA/AoD .
 *
 * This function reset main LCL HW registers that have been set by XCVR_LCL_BLEAoDAoAInit.
 *
 */
void XCVR_LCL_BLEAoDAoADeInit(void);


/*!
 * @brief Program a new antenna to ant_lut_gpio translation table.
 *
 * Use this function to change the antenna to ant_lut_gpio translation table.
 * The table is the list of the values to apply to lant_lut_gpio[3:0] during the switchnig sequence.
 * The default table (if this API is not called) is {0x01, 0x02, 0x04, 0x08} corresponding to the trivial
 * case where each lant_lut_gpio bit control one antenna.
 * The maximum table length is 32.
 *
 * @param[in] TableLength Number on entries in the pattern.
 * @param[in] Table Pointer to the table containing the value of lant_lut_gpio[3:0] that is
 *            associated to each antenna .
 *
 @return None.
 *
 */
xcvrLclStatus_t XCVR_LCL_SetLUTAntennaTable(uint8_t TableLength, const uint8_t *Table);

/*!
 * @brief Get the current antenna to ant_lut_gpio translation table.
 *
 * Use this function to get the antenna to ant_lut_gpio translation table currently in use.
 * The table is the list of the values to apply to lant_lut_gpio[3:0] during the switchnig sequence.
 * The default table (if this API is not called) is {0x01, 0x02, 0x04, 0x08, 0x00, ...} corresponding to the trivial
 * case where each lant_lut_gpio bit control one antenna.
 * The whole table is returned (32 elements)
 *
 * @param[in] Table Pointer to the table where to write the current table containing the value of lant_lut_gpio[3:0]
 *            for each antenna .
 *
 @return None.
 *
 */
void XCVR_LCL_GetLUTAntennaTable(uint8_t *Table);
#endif /* defined (SUPPORT_AOA_AOD_IN_XCVR) && (SUPPORT_AOA_AOD_IN_XCVR == 1) */
#endif


#if defined(__cplusplus)
extern "C" {
#endif

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* NXP_XCVR_EXT_CTRL_H */
