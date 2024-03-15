/*
 * Copyright (c) 2015,2018, Freescale Semiconductor, Inc.
 * Copyright 2018-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef NB2P4_XCVR_H
/* clang-format off */
#define NB2P4_XCVR_H
/* clang-format on */

#include "fsl_device_registers.h"

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
#include "rfmc_ctrl.h"
#else
#include "rsim_ctrl.h"
#endif

#include <stdbool.h>

/*!
 * @addtogroup xcvr_main Radio Configuration Routines
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/
#if !defined(RADIO_IS_GEN_3P5) && !defined(NXP_RADIO_GEN) 
#error "XCVR compilation requires a valid radio generation flag to be defined."
#endif

/* Workaround to fix SDK header names */
#ifndef GEN4PHY
#define GEN4PHY XCVR_2P4GHZ_PHY
#endif

#ifndef RBME
#define RBME D_IP_PLL_NARROWBAND_SYN
#endif

/* Workaround for not exposed DFT register in DBGRAM/DMA */
#if defined(RADIO_IS_GEN_3P5)
#define XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL_MASK    (0xF00U)
#define XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL_SHIFT   (8U)
#define XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL(x)      (((uint32_t)(((uint32_t)(x)) << XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL_SHIFT)) & XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL_MASK)
#endif

/* Workaround for not exposed HPMCAL_CTRL register in XCVR_PLL */
#if !defined(HPMCAL_CTRL)

/** Peripheral XCVR_PLL_DIG base pointer */
#define XCVR_PLL_DIG2                             ((XCVR_PLL_DIG2_Type *)XCVR_PLL_DIG_BASE)

/* Workaround for missing Zigbee/802.15.4 interrupt in CM33 */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
#ifndef RF_Zigbee_IRQn
#define RF_Zigbee_IRQn              53               /**< Radio Frequency 2.4GHz - Zigbee Link Layer interrupt */
#endif
#endif /* defined(RADIO_IS_GEN_4P5) */

/* defining only the first part of the XCVR_PLL_DIG structure to make a version with HPMCAL_CTRL available. Everything else is appended with dummy or deleted */
typedef struct {
  __IO uint32_t HPM_BUMPdummy;                          /**< PLL HPM Analog Bump Control, offset: 0x0 */
  __IO uint32_t MOD_CTRLdummy;                          /**< PLL Modulation Control, offset: 0x4 */
  __IO uint32_t CHAN_MAPdummy;                          /**< PLL Channel Mapping, offset: 0x8 */
       uint8_t RESERVED_0dummy[4];
  __IO uint32_t CHAN_MAP_EXTdummy;                      /**< PLL Channel Mapping Extended, offset: 0x10 */
       uint8_t RESERVED_1dummy[4];
  __IO uint32_t LOCK_DETECTdummy;                       /**< PLL Lock Detect Control, offset: 0x18 */
  __IO uint32_t HPM_CTRLdummy;                          /**< PLL High Port Modulator Control, offset: 0x1C */
  __IO uint32_t HPMCAL_CTRL;                       /**< PLL High Port Calibration Control, offset: 0x20 */
} XCVR_PLL_DIG2_Type;


/*! @name HPMCAL_CTRL - PLL High Port Calibration Control */
/*! @{ */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MASK (0x1FFFU)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_SHIFT (0U)
/*! HPM_CAL_FACTOR - High Port Modulation Calibration Factor
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MASK)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_BUMPED_MASK (0x2000U)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_BUMPED_SHIFT (13U)
/*! HPM_CAL_BUMPED - HPM_CAL_BUMPED
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_BUMPED(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_BUMPED_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_BUMPED_MASK)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_COUNT_SCALE_MASK (0x4000U)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_COUNT_SCALE_SHIFT (14U)
/*! HPM_CAL_COUNT_SCALE - HPM_CAL_COUNT_SCALE
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_COUNT_SCALE(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_COUNT_SCALE_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_COUNT_SCALE_MASK)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HP_CAL_DISABLE_MASK (0x8000U)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HP_CAL_DISABLE_SHIFT (15U)
/*! HP_CAL_DISABLE - Disable HPM Manual Calibration
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HP_CAL_DISABLE(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HP_CAL_DISABLE_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HP_CAL_DISABLE_MASK)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MANUAL_MASK (0x1FFF0000U)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MANUAL_SHIFT (16U)
/*! HPM_CAL_FACTOR_MANUAL - Manual HPM Calibration Factor
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MANUAL(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MANUAL_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_FACTOR_MANUAL_MASK)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_SKIP_MASK (0x20000000U)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_SKIP_SHIFT (29U)
/*! HPM_CAL_SKIP - HPM_CAL_SKIP
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_SKIP(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_SKIP_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_SKIP_MASK)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE_MASK (0x40000000U)
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE_SHIFT (30U)
/*! HPM_CAL_ARRAY_SIZE - High Port Modulation Calibration Array Size
 *  0b0..128
 *  0b1..256
 */
#define XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE(x) (((uint32_t)(((uint32_t)(x)) << XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE_SHIFT)) & XCVR_PLL_DIG2_HPMCAL_CTRL_HPM_CAL_ARRAY_SIZE_MASK)
/*! @} */

#endif /* !defined(HPMCAL_CTRL) */

/* Error check compile flags */
#if !((RF_OSC_26MHZ == 1U) || (RF_OSC_26MHZ == 0U))
/* Code requires this flag to strictly be 0 or 1 */
#error "Invalid flag setting for RF OSC 26MHz selection. Only 0 or 1 is permitted."
#endif /* RF_OSC_26MHZ > 1 */
/**************************************/

#define NULLPTR ((void *)(0x0))  /* MISRA safe definition for NULL */

 /* Default RF OSC definition. Allows for compile time clock frequency definition */
#ifdef CLOCK_MAIN

#else
#if RF_OSC_26MHZ == 1
#define CLOCK_MAIN (EXT_CLK_26_MHZ) /* See ext_clock_config_t for this value */
#else
#define CLOCK_MAIN (EXT_CLK_32_MHZ) /* See ext_clock_config_t for this value */
#endif /* RF_OSC_26MHZ == 1 */
#endif /* CLOCK_MAIN */

#ifndef FPGA_TARGET /* make sure the FPGA_TARGET flag is defined to support XCVR compilation */
#define FPGA_TARGET (0)
#endif /* FPGA_TARGET */

#define B0(x)   (((uint32_t)(((uint32_t)(x)) << 0)) & 0xFFU)
#define B1(x)   (((uint32_t)(((uint32_t)(x)) << 8)) & 0xFF00U)
#define B2(x)   (((uint32_t)(((uint32_t)(x)) << 16)) & 0xFF0000U)
#define B3(x)   (((uint32_t)(((uint32_t)(x)) << 24)) & 0xFF000000U)

/* TSM and LL related delay values */
#if defined(RADIO_IS_GEN_3P5)
#define TX_RX_ON_DELinit       (0x00009686UL) /* TX_ON_DELAY defined differently than prior radios, must be 150usec. */
#define TX_RX_SYNTH_init_A0    (0x00002318UL) /* TX_RX_SYNTH initialization for KW37 A0 silicon */
#define TX_RX_SYNTH_init_B0    (0x00006918UL) /* TX_RX_SYNTH initialization for KW37 B0 silicon */

#define TX_PATH_DELAY_2MBPS    (0x1U)       /*!<  Gen 3.5 - TX Path delay for bits from controller to antenna for 1Mbps */
#define TX_PATH_DELAY_1MBPS    (0x2U)       /*!<  Gen 3.5 - TX Path delay for bits from controller to antenna for 2Mbps */
#define TX_PATH_DELAY_S2_CODED (0x2U)       /*!<  Gen 3.5 - TX Path delay for bits from controller to antenna for S2 coded */
#define TX_PATH_DELAY_S8_CODED (0x2U)       /*!<  Gen 3.5 - TX Path delay for bits from controller to antenna for S8 coded */

#define RX_PATH_DELAY_2MBPS    (0x7U)       /*!<  Gen 3.5 - RX Path delay for last bit from antenna to controller for 1Mbps */
#define RX_PATH_DELAY_1MBPS    (0xEU)       /*!<  Gen 3.5 - RX Path delay for last bit from antenna to controller for 2Mbps */
#define RX_PATH_DELAY_S2_CODED (0x10U)      /*!<  Gen 3.5 - RX Path delay for last bit from antenna to controller for S2 coded */
#define RX_PATH_DELAY_S8_CODED (0x10U)      /*!<  Gen 3.5 - RX Path delay for last bit from antenna to controller for S8 coded */
#else
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
#define TX_RX_ON_DELinit       (0x00009256UL) /* TX_ON_DELAY defined differently than prior radios, must be 86usec. Used for Mindtree BLE LL */
#if defined(HDI_MODE) && (HDI_MODE == 1)
#define TX_RX_SYNTH_init       (0x00007056UL) /* HDI MODE tuning */
#else
#define TX_RX_SYNTH_init       (0x0000745BUL) /* Used for Mindtree BLE LL */
#endif /* HDI_MODE */

#define TX_PATH_DELAY_2MBPS    (0x1U)       /*!<  Gen 4.0 - TX Path delay for bits from controller to antenna for 1Mbps */
#define TX_PATH_DELAY_1MBPS    (0x2U)       /*!<  Gen 4.0 - TX Path delay for bits from controller to antenna for 2Mbps */
#define TX_PATH_DELAY_S2_CODED (0x2U)       /*!<  Gen 4.0 - TX Path delay for bits from controller to antenna for S2 coded */
#define TX_PATH_DELAY_S8_CODED (0x2U)       /*!<  Gen 4.0 - TX Path delay for bits from controller to antenna for S8 coded */

#define RX_PATH_DELAY_2MBPS    (0x7U)       /*!<  Gen 4.0 - RX Path delay for last bit from antenna to controller for 1Mbps */
#define RX_PATH_DELAY_1MBPS    (0xEU)       /*!<  Gen 4.0 - RX Path delay for last bit from antenna to controller for 2Mbps */
#define RX_PATH_DELAY_S2_CODED (0x10U)      /*!<  Gen 4.0 - RX Path delay for last bit from antenna to controller for S2 coded */
#define RX_PATH_DELAY_S8_CODED (0x10U)      /*!<  Gen 4.0 - RX Path delay for last bit from antenna to controller for S8 coded */
#else /* defined (RADIO_IS_GEN_4P5) */
#define TX_PATH_DELAY_2MBPS    (0x1U)       /*!<  Gen 4.5 - TX Path delay for bits from controller to antenna for 1Mbps */
#define TX_PATH_DELAY_1MBPS    (0x1U)       /*!<  Gen 4.5 - TX Path delay for bits from controller to antenna for 2Mbps */
#define TX_PATH_DELAY_S2_CODED (0x1U)       /*!<  Gen 4.5 - TX Path delay for bits from controller to antenna for S2 coded */
#define TX_PATH_DELAY_S8_CODED (0x1U)       /*!<  Gen 4.5 - TX Path delay for bits from controller to antenna for S8 coded */

#define RX_PATH_DELAY_2MBPS    (0x7U)       /*!<  Gen 4.5 - RX Path delay for last bit from antenna to controller for 1Mbps */
#define RX_PATH_DELAY_1MBPS    (0xEU)       /*!<  Gen 4.5 - RX Path delay for last bit from antenna to controller for 2Mbps */
#define RX_PATH_DELAY_S2_CODED (0x12U)      /*!<  Gen 4.5 - RX Path delay for last bit from antenna to controller for S2 coded */
#define RX_PATH_DELAY_S8_CODED (0x17U)      /*!<  Gen 4.5 - RX Path delay for last bit from antenna to controller for S8 coded */

#endif /* defined(RADIO_IS_GEN_4P0) */

#endif /* defined(RADIO_IS_GEN_3P5) */

#define PA_SETTLING_TIME       (0x4U)       /*!< Time for PA to settle after ramp up and before on-the-air time. */
#define PHY_EN_DELAY           (28U)        /* Additional delay to add to PHY_EN signal to account for DC settling after RX_WU */
#ifdef gXcvrAddTxOffset_d /* allow higher level software to control whether offset is added or not */
#define tx_ofst_usec           (36U)        /* offset value for TX warmup to extend TSM sequence on A0 KW37 silicon for Bluetooth LE */
#else
#define tx_ofst_usec           (0U)         /* zero offset value for TX warmup for GENFSK only use cases */
#endif /* ifdef gXcvrAddTxOffset_d */
#define RX_WD_TIME_TGT         (0x8U)       /*!< Mid point value for the RX warmdown time register */
#define TX_WD_TIME_TGT         (0x8U)       /*!< Mid point value for the TX warmdown time register */


#ifndef XCVR_DAC_TRIM_RETRY_COUNT
#define XCVR_DAC_TRIM_RETRY_COUNT   (1U)     /*!< Default retry count for DCOC DAC trim process to 1 */
#endif

#ifndef XTAL_TEMP_COMP_LUT_SIZE
#define XTAL_TEMP_COMP_LUT_SIZE     (20)    /*!< Default LUT size for temp compensation of XTAL trim (signed, but must be positive) */
#endif

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
/* Packet RAM to IPS register descriptor access macros */
/* Address field for the Packet RAM to IPS register descriptor */
#define PR2IPS_ADDR_MASK       (0x00FFFFFFU)
#define PR2IPS_ADDR_SHIFT      (0U)
#define PR2IPS_ADDR(x)         (((uint32_t)(((uint32_t)(x)) << PR2IPS_ADDR_SHIFT)) & PR2IPS_ADDR_MASK)
/* Count field for the Packet RAM to IPS register descriptor */
#define PR2IPS_COUNT_MASK       (0x3F000000U)
#define PR2IPS_COUNT_SHIFT      (24U)
#define PR2IPS_COUNT(x)         (((uint32_t)(((uint32_t)(x)) << PR2IPS_COUNT_SHIFT)) & PR2IPS_COUNT_MASK)/* Address field for the Packet RAM to IPS register descriptor */
/* Type field for the Packet RAM to IPS register descriptor */
#define PR2IPS_TYPE_MASK       (0xC0000000U)
#define PR2IPS_TYPE_SHIFT      (30U)
#define PR2IPS_TYPE(x)         (((uint32_t)(((uint32_t)(x)) << PR2IPS_TYPE_SHIFT)) & PR2IPS_TYPE_MASK)
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */


/*! @brief Error codes for the XCVR driver. */
typedef enum
{
  gXcvrSuccess_c = 0,
  gXcvrInvalidParameters_c,
  gXcvrUnsupportedOperation_c,
  gXcvrTrimFailure_c,
  gXcvrInvalidConfiguration_c,
  gXcvrConfigurationFailure_c,
} xcvrStatus_t;

/*! @brief  Clock configuration enum type. */
typedef enum
{
  EXT_CLK_32_MHZ = 0,
  EXT_CLK_26_MHZ = 1,
} ext_clock_config_t;

/*! @brief  Radio operating mode setting types. */
typedef enum
{
  BLE_MODE = 0,

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
    OQPSK_802P15P4 = 1,
#else
    RESERVED1_MODE = 1, /*!< Reserved  */
#endif /* defined(RADIO_IS_GEN_4P5) */
  RESERVED2_MODE = 2, /*!< Reserved  */

  /* BT=0.5, h=** */
  GFSK_BT_0p5_h_0p5  = 3, /*!< BT=0.5, h=0.5  */
  GFSK_BT_0p5_h_0p32 = 4, /*!< BT=0.5, h=0.32 */
  GFSK_BT_0p5_h_0p7  = 5, /*!< BT=0.5, h=0.7  */
  GFSK_BT_0p5_h_1p0  = 6, /*!< BT=0.5, h=1.0  */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
  /* BT=** h=0.5 */
  GFSK_BT_0p3_h_0p5  = 7, /*!< BT=0.3, h=0.5  */
  GFSK_BT_0p7_h_0p5  = 8, /*!< BT=0.7, h=0.5  */
#else
  RESERVED3_MODE  = 7, /*!< Reserved  */
  RESERVED4_MODE  = 8, /*!< Reserved  */
#endif /* defined(RADIO_IS_GEN_4P0) */

  MSK = 9,
  BLE_LR = 10,
  FSK_H_0p5 = 11,
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
  GFSK_BT_1p0_h_0p5  = 12, /*!< BT=1.0, h=0.5  */
  GFSK_BT_2p0_h_0p5 = 13, /*!< BT=2.0, h=0.32 */
#else
  RESERVED5_MODE  = 12, /*!< Reserved  */
  RESERVED6_MODE  = 13, /*!< Reserved  */
#endif /* defined(RADIO_IS_GEN_4P5) */
  NUM_RADIO_MODES = 14,
} radio_mode_t;

/*! @brief  Coding rate selections. */
typedef enum
{
    BLE_UNCODED = 0, /*!< Bluetooth LE Uncoded */
    BLE_CODED_S2 = 1, /*!< Bluetooth LE Long Range  S=2 */
    BLE_CODED_S8 = 2, /*!< Bluetooth LE Long Range  S=8 */
    ANT_UNCODED = 3, /*!< ANT emulation */
    CODING_UNASSIGNED = 4, /* Unassigned Data Rate. Must match bit assignment in BITRATE field */
} coding_t;

/*! @brief  Data rate selections. */
typedef enum
{
    DR_1MBPS = 0, /*!< 1Mbps Data Rate. Must match bit assignment in BITRATE field */
    DR_500KBPS = 1, /*!< 500Kbps Data Rate. Must match bit assignment in BITRATE field */
    DR_250KBPS = 2, /*!< 250Kbps Data Rate. Must match bit assignment in BITRATE field */
    DR_2MBPS = 3, /*!< 2Mbps Data Rate. Must match bit assignment in BITRATE field */
    DR_UNASSIGNED = 4, /* Unassigned Data Rate. Must match bit assignment in BITRATE field */
} data_rate_t;

/*! @brief  Link layer types. */
typedef enum
{
    BLE_LL = 0, /*!< Bluetooth Low Energy Link Layer. Must match bit assignment in RADIO1_IRQ_SEL */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    ZIGBEE_LL = 1, /*!< 802.15.4/Zigbee Link Layer */
#else
    RESERVED1_LL = 1, /*!< Reserved */
#endif /* defined(RADIO_IS_GEN_4P5) */
    RESERVED2_LL = 2, /*!< Reserved */
    GENFSK_LL = 3, /*!< Generic FSK Link Layer. Must match bit assignment in RADIO1_IRQ_SEL */
    UNASSIGNED_LL = 4, /*!< No Link Layer Assigned.  Must match bit assignment in RADIO1_IRQ_SEL */
} link_layer_t;

/*! @brief  Control settings for Fast Antenna Diversity */
typedef enum
{
  NONE = 0,
  FAD_ENABLED = 1,
  LPPS_ENABLED = 2
} FAD_LPPS_CTRL_T;

/*! @brief  XCVR Panic codes for indicating panic reason. */
typedef enum
{
  WRONG_RADIO_ID_DETECTED = 1,
  CALIBRATION_INVALID = 2,
  RADIO_INIT_FAILURE = 3,
} XCVR_PANIC_ID_T;

/*! @brief  Initialization or mode change selection for config routine. */
typedef enum
{
  XCVR_MODE_CHANGE = 0,
  XCVR_FIRST_INIT = 1,
} XCVR_INIT_MODE_CHG_T;

/*! @brief  ACTIVE_LL field settings. */
typedef enum
{
    XCVR_ACTIVE_LL_BTLE = 0,         /*!<  BTLE Link Layer active. */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    XCVR_ACTIVE_LL_ZIGBEE_LL = 1,    /*!< 802.15.4/Zigbee Link Layer active */
#else
    XCVR_ACTIVE_LL_RESERVED = 1,     /*!< Reserved, do not use. */
#endif /* defined(RADIO_IS_GEN_4P5) */
    XCVR_ACTIVE_LL_GENFSK = 2,       /*!<  Generic FSK Link Layer active. */
    XCVR_ACTIVE_LL_ALL_DISABLED = 3, /*!<  Disable all Link Layers, none active. */
    XCVR_ACTIVE_LL_MAX = 4           /*!<  Error. */
}   XCVR_ACTIVE_LL_SEL_T;

/*! @brief  BAND_SELECT field settings. */
typedef enum
{
    XCVR_BAND_SEL_BTLE = 0,          /*!<  BTLE normal band selected in PLL. */
    XCVR_BAND_SEL_BTLE_IN_MBAN = 1,  /*!<  BTLE in MBAN band selected in PLL. */
    XCVR_BAND_SEL_BTLE_OVLP_MBAN = 2,/*!<  BTLE overlap MBAN band selected in PLL. */
    XCVR_BAND_SEL_RESERVED1 = 3,     /*!<  Reserved band selection */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    XCVR_BAND_SEL_802_15_4_ISM = 4,  /*!<  802.15.4 ISM band selection */
#else
    XCVR_BAND_SEL_RESERVED2 = 4,     /*!<  Reserved band selection */
#endif /* defined(RADIO_IS_GEN_4P5) */
    XCVR_BAND_SEL_802_15_4_J = 5,    /*!<  802.15.4jband selection */
    XCVR_BAND_SEL_DFT = 6,           /*!<  DFT band selection */
    XCVR_BAND_SEL_GENERIC = 7,       /*!<  Generic band selection */
    XCVR_BAND_SEL_MAX = 8            /*!<  Error. */
}   XCVR_BAND_SEL_T;

/*! @brief  RADIO_VER field settings. */
typedef enum
{
    XCVR_RADIO_GEN_2P0 = 0xBU,         /*!<  2.4GHz Radio 2.0 . */
    XCVR_RADIO_GEN_2P1 = 0x8U,         /*!<  2.4GHz Radio 2.1 . */
    XCVR_RADIO_GEN_3P0 = 0x4U,         /*!<  2.4GHz Radio 3.0 . */
    XCVR_RADIO_GEN_3P1 = 0x5U,         /*!<  2.4GHz Radio 3.1 . */
    XCVR_RADIO_GEN_3P5 = 0xCU,         /*!<  2.4GHz Radio 3.5 . */
    XCVR_RADIO_GEN_3P5_B0 = 0x6U,      /*!<  2.4GHz Radio 3.5, B0 silicon revision. */
    XCVR_RADIO_GEN_4P0 = 0x4000U,      /*!<  2.4GHz Radio 4.0 . */
    XCVR_RADIO_GEN_4P5 = 0x4500U,      /*!<  2.4GHz Radio 4.5 . */
    XCVR_RADIO_GEN_INVALID = 0xFFFFU   /*!<  Error. */
}   XCVR_RADIO_GEN_T;

/*! @brief  Packet RAM selection (TX or RX) enum type for PR2IPS usage. */
typedef enum
{
  TX_PKT_RAM_SEL = 0U, /*!<  Select the TX PKT_RAM bank for usage for Fast Peripheral Reg upload/download. */
  RX_PKT_RAM_SEL = 1U, /*!<  Select the RX PKT_RAM bank for usage for Fast Peripheral Reg upload/download. */
} pkt_ram_bank_sel_t;

/*! @brief  Packet RAM 2 IPS transfer type field. */
typedef enum
{
  PR2IPS_END_OF_DESCRIP = 0U, /*!<  Denotes the last PR2IPS descriptor in a sequence. */
  PR2IPS_SINGLE_XFER = 1U,       /*!<  Selects a single transfer in the PR2IPS descriptor. */
  PR2IPS_BURST_XFER = 2U         /*!<  Selects a burst transfer in the PR2IPS descriptor. */
} pr2ips_xfer_t;

/*! @brief Current configuration of the radio. */
typedef struct
{
    radio_mode_t radio_mode;
    data_rate_t data_rate;
    data_rate_t alt_data_rate;
    coding_t coding_mode;
} xcvr_currConfig_t;

/*!
 * @brief XCVR RX_DIG channel filter coefficient storage
 * Storage of the c
oefficients varies from 6 bits to 10 bits so all use int16_t for storage.
 */
typedef struct
{
    uint32_t dummy;
} xcvr_rx_chf_coeffs_t;

/*!
 * @brief XCVR DCOC DAC trim register value storage
 * Storage of the entire 32 bit register contents for all DCOC DAC trim registers.
 */
typedef struct
{
    uint32_t dcoc_bba_step_reg; /*!< XCVR_RX_DIG->DCOC_BBA_STEP contents */
    uint32_t dcoc_tza_step_0_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_0 contents */
    uint32_t dcoc_tza_step_1_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_1 contents */
    uint32_t dcoc_tza_step_2_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_2 contents */
    uint32_t dcoc_tza_step_3_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_3 contents */
    uint32_t dcoc_tza_step_4_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_4 contents */
    uint32_t dcoc_tza_step_5_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_5 contents */
    uint32_t dcoc_tza_step_6_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_6 contents */
    uint32_t dcoc_tza_step_7_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_7 contents */
    uint32_t dcoc_tza_step_8_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_8 contents */
    uint32_t dcoc_tza_step_9_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_9 contents */
    uint32_t dcoc_tza_step_10_reg; /*!< XCVR_RX_DIG->DCOC_TZA_STEP_10 contents */
    uint32_t dcoc_dac_init_reg; /*!< XCVR_RX_DIG->DCOC_DAC_INIT contents */
} xcvr_DcocDacTrim_t;

/*!
 * @brief RBME CRC configuration register value storage
 * Storage of the CRC register contents for customizing CRC settings.
 */
typedef struct
{
    uint32_t crcw_cfg;          /*!< RBME->CRCW_CFG contents */
    uint32_t crcw_cfg3;         /*!< RBME->CRCW_CFG3 contents */
    uint32_t crc_init;          /*!< RBME->CRC_INIT contents */
    uint32_t crc_poly;          /*!< RBME->CRC_POLY contents */
} rbme_crc_config_t;

/*!
 * @brief RBME Whitener configuration register value storage
 * Storage of the Whitener register contents for customizing Whitener settings.
 */
typedef struct
{
    uint32_t whiten_cfg;        /*!< RBME->WHITEN_CFG contents */
    uint32_t whiten_poly;       /*!< RBME->WHITEN_POLY contents */
    uint32_t whiten_sz_thr;     /*!< RBME->WHITEN_SZ_THR contents */
    uint32_t wht_cfg;           /*!< RBME->WHT_CFG contents */
} rbme_whiten_config_t;

/*!
 * @brief XCVR PLL channel configuration structure
 * This structure is used to store the integer portion, numerator, and denominator values for setting a PLL channel.
 */
typedef struct
{
    uint32_t intpart;
    uint32_t numerator;
} xcvr_pllChannel_t;

/*!
 * @brief XCVR XTAL temp compensation configuration structure
 * This structure is used to store the table for temperature compensation of the XTAL trim.
 */
typedef struct
{
    int8_t min_temp_degc;        /*!< The temperature corresponding to the first LUT entry, in deg C */
    int8_t max_temp_degc;        /*!< The temperature corresponding to the last LUT entry, in deg C */
    uint8_t trim_below_min_temp; /*!< The CDAC value to use when temp is below the min temp for the LUT */
    uint8_t trim_above_max_temp; /*!< The CDAC value to use when temp is above the max temp for the LUT */
    int8_t temp_step_degc;      /*!< The temperature step between each LUT entry, in deg C */
    uint8_t xtal_trim_lut[XTAL_TEMP_COMP_LUT_SIZE];     /*!< The CDAC values for the LUT, stored in ascending temperature, from min to max with temp_step increments */
} xcvr_temp_comp_lut_t;


/*!
 * @brief Panic callback function type
 *
 * The panic callback function is defined by system if system need to be informed of XCVR fatal errors.
 * refer to #XCVR_RegisterPanicCb
 */
typedef void (*panic_fptr)(uint32_t panic_id, uint32_t location, uint32_t extra1, uint32_t extra2);

/**
 * @brief 32-bit floating-point type definition.
 */
#ifndef FLOAT_32_T_DEFINED
#define FLOAT_32_T_DEFINED
typedef float float32_t;
#endif /* float32_t */

/**
 * @brief 64-bit floating-point type definition.
 */
#ifndef FLOAT_64_T_DEFINED
#define FLOAT_64_T_DEFINED
typedef double float64_t;
#endif /* float64_t */

/* This include must be down here rather than up top due to references to structs defined above */
#include "nxp_xcvr_mode_config.h"



/* These exported values only apply to Gen 3.5 and Gen 4.0 */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN <= 400)
/* Expose TX_RX_ON_DELAY and SYNTH_DELAY for BLE LL to access for setting registers appropriately */
extern const uint16_t tx_rx_on_delay;
extern uint16_t tx_rx_synth_delay; /* Not constant, allows for A0/B0 KW37 support */
#endif /* !defined(RADIO_IS_GEN_4P5) */


/* Asembly NOP function encapsulation */
static inline void ASM_NOP(void)
{
    __asm("nop");
};


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes an XCVR instance.
 *
 * This function initializes the XCVR module according to the configuration structure settings. This the only function call required to
 * start up the XCVR in most situations. It includes a DCOC DAC trim step that is left out in the ::XCVR_InitNoDacTrim() version.
 *
 * @param[in] xcvr_config  Pointer to a pointer to the complete radio settings structure.
 * @param[in] rbme_config  Pointer to a pointer to the RBME settings structure for coding, CRC, whitening setup.
 * @return Status of the call.
 * @note This function encompasses the ::XCVR_RadioStartup(() and ::XCVR_RadioRegSetup() and XCVR_CalculateDcocDacTrims() functions.
 */
xcvrStatus_t XCVR_Init(const xcvr_config_t ** xcvr_config,
                        const xcvr_coding_config_t ** rbme_config);

/*!
 * @brief Initializes an XCVR instance but omits DCOC DAC trim process.
 *
 * This function initializes the XCVR module according to the configuration structure settings. This function call must be combined with
 * the XCVR_SetDcocDacTrims() function to properly start up the XCVR. This allows a stored set of DCOC DAC trims to be used rather
 * than running the trim process during every startup.
 *
 * @param[in] xcvr_config  Pointer to a pointer to the complete radio settings structure.
 * @param[in] rbme_config  Pointer to a pointer to the RBME settings structure for coding, CRC, whitening setup.
 * @return Status of the call.
 * @note This function encompasses everything in the ::XCVR_Init() function but omits the call to XCVR_CalculateDcocDacTrims().
 */
xcvrStatus_t XCVR_InitNoDacTrim(const xcvr_config_t ** xcvr_config,
                                    const xcvr_coding_config_t ** rbme_config);

/*!
 * @brief Deinitializes an XCVR instance.
 *
 * This function gates the XCVR module clock and sets all registers to their reset values.
 *
 */
void XCVR_Deinit(void);

/*!
 * @brief Performs all register setup for XCVR initialization or mode change.
 *
 * This function sets up all of the registers in the XCVR module with user-defined settings.
 *
 * @param[in] xcvr_config  Pointer to a pointer to the full XCVR settings structure.
 * @return Status of the call.
 */
xcvrStatus_t XCVR_RadioRegSetup(const xcvr_config_t ** xcvr_config);

/*!
 * @brief Initializes an XCVR instance.
 *
 * This function initializes the XCVR module with user-defined settings.
 *
 * @param[in] xcvr_config  Pointer to a pointer to the complete radio settings structure.
 * @param[in] rbme_config  Pointer to a pointer to the complete RBME settings structure.
 * @param[in]  first_init Indicates whether the call is to initialize (== XCVR_FIRST_INIT) or the call is to perform a mode change (== XCVR_MODE_CHANGE)
 * @return Status of the call.
 */
xcvrStatus_t XCVR_Configure(const xcvr_config_t ** xcvr_config,
                                const xcvr_coding_config_t ** rbme_config,
                                XCVR_INIT_MODE_CHG_T first_init);

/*!
 * @brief Initializes the RBME block.
 *
 * This function initializes the RBME module with user-defined settings to configure coding, CRC, whitening, etc.
 *
 * @param[in] rbme  Pointer to a pointer to the RBME settings structure for coding, CRC, whitening setup.
 * @return Status of the call.
 */
xcvrStatus_t XCVR_RBME_Configure(const xcvr_coding_config_t ** rbme);

/*!
 * @brief Startup the radio.
 *
 * This function brings the radio into RUN mode and enables the radio clocks.
 *
 */
void XCVR_RadioStartup(void);

/*!
 * @brief Set XCVR register to reset value.
 *
 * This function set XCVR registers to their reset values.
 *
 */
void XCVR_Reset(void);

/*!
 * @brief Change the operating mode of the radio.
 *
 * This function changes the XCVR to a new radio operating mode.
 *
 * @param[in] xcvr_config  Pointer to a pointer to the complete XCVR settings structure.
 * @param[in] rbme_config  Pointer to a pointer to the complete RBME settings structure.
 * @return Status of the call.
 */
xcvrStatus_t XCVR_ChangeMode (const xcvr_config_t ** xcvr_config,
                                    const xcvr_coding_config_t ** rbme_config);

/*!
 * @brief Get the current configuration of the RBME CRC and Whitening.
 *
 * This function fetches the current configuration of the CRC and Whitening registers in the RBME
 *
 * @param[inout] crc_config  Pointer to a structure to be updated with the CRC configurations. NULLPTR if not fetching CRC settings.
 * @param[inout] whiten_config  Pointer to a structure to be updated with the Whitener configurations. NULLPTR if not fetching Whitener settings.
 * @note This API will return meaningless results if called before the radio and RBME are initialized.
 */
void XCVR_GetCRCWConfig(rbme_crc_config_t * crc_config, rbme_whiten_config_t * whiten_config );

/*!
 * @brief Sets the  configuration of the RBME CRC and Whitening.
 *
 * This function sets the register configuration of the CRC and Whitening registers in the RBME
 *
 * @param[inout] crc_config  Pointer to a structure containing the CRC configurations. NULLPTR if not setting CRC settings.
 * @param[inout] whiten_config  Pointer to a structure containing the Whitener configurations. NULLPTR if not setting Whitener settings.
 * @note This API will return meaningless results if called before the radio and RBME are initialized.
 */
void XCVR_SetCRCWConfig(const rbme_crc_config_t * crc_config, const rbme_whiten_config_t * whiten_config );


/*!
 * @brief Enable Narrowband RSSI measurement.
 *
 * This function enables the narrowband RSSI measurement
 *
 * @param IIRnbEnable If set to true, causes the NB RSSI to be enabled, false == disabled.
 */
void XCVR_EnaNBRSSIMeas(bool IIRnbEnable);


/*!
 * @brief Register a callback from upper layers.
 *
 * This function registers a callback from the upper layers for the radio to call in case of fatal errors.
 *
 * @param[in] fptr  The function pointer to a panic callback.
 */
void XCVR_RegisterPanicCb ( panic_fptr fptr ); /* allow upper layers to provide PANIC callback */


/*!
 * @brief Panic function for XCVR related fatal errors.
 *
 * This function executes a callback to the upper layers in case of fatal errors. It enters an endless loop if no
 * callback function has been registered by ::XCVR_RegisterPanicCb().
 *
 * @param[in] panic_id  The ID for the failure.
 * @param[in] panic_address  The address of the routine calling the panic.
 */
void XcvrPanic(XCVR_PANIC_ID_T panic_id, uint32_t panic_address);

#if defined(RADIO_IS_GEN_3P5) /* Gen 4 and later radios don't include DAC trims */
/*!
 * @brief Change the mapping of the radio IRQs.
 *
 * This function changes the mapping of the radio LL IRQ signals to the 2.4G Radio INT0 and 2.4G Radio INT1 lines.
 *
 * @param[in] irq0_mapping  The LL which should be mapped to the INT0 line.
 * @param[in] irq1_mapping  The LL which should be mapped to the INT1 line.
 * @return Status of the mapping request.
 * @note The radio_mode_t parameters map to ::link_layer_t selections for the LL which is connected to the INT line.
 * @warning
 *  The same LL must NOT be mapped to both INT lines.
 */
xcvrStatus_t XCVR_SetIRQMapping(radio_mode_t irq0_mapping, radio_mode_t irq1_mapping);

/*!
 * @brief Get the mapping of the one of the radio IRQs.
 *
 * This function reads the setting for the mapping of one of the radio LL IRQ signals to the 2.4G Radio INT0 and 2.4G Radio INT1 lines.
 *
 * @param[in] int_num  The number, 0 or 1, of the INT line to fetched.
 * @return The mapping setting of the specified line.
 * @note Any value passed into this routine other than 0 will be treated as a 1.
 */
link_layer_t XCVR_GetIRQMapping(uint8_t int_num);
#endif /* defined(RADIO_IS_GEN_3P5)  */ /* Gen 4 and later radios don't include DAC trims */

/*!
 * @brief Get the current configuration of the XCVR.
 *
 * This function fetches the current configuration (radio mode and radio data rate) of the XCVR to allow LL to properly config data rates, etc
 *
 * @param[inout] curr_config  Pointer to a structure to be updated with the current mode and data rate.
 *
 * @return The status of the request, success or invalid parameter (null pointer).
 *
 * @note This API will return meaningless results if called before the radio is initialized.
 */
xcvrStatus_t XCVR_GetCurrentConfig(xcvr_currConfig_t * curr_config);

/*!
 * @brief Controls setting the active link layer.
 *
 * This function enables the upper layers set which link layer (BTLE or GENFSK) should be made active next.
 *
 * @param[in] active_ll  The link layer to be selected.
 *
 * @return The status of the change.
 *
 * @note The caller is responsible for ensuring that the link layer that is active has no ongoing transactions and
 *  is in the idle state before switching.
 */
xcvrStatus_t XCVR_SetActiveLL(XCVR_ACTIVE_LL_SEL_T active_ll);


/*!
 * @brief Controls fetching the current active link layer.
 *
 * This function enables the upper layers get which link layer (BTLE or GENFSK) is currently active.
 *
 * @return The active link layer.
 */
XCVR_ACTIVE_LL_SEL_T  XCVR_GetActiveLL(void);

/*!
 * @brief Controls setting the PLL band selection.
 *
 * This function enables the upper layers set whichband should be selected in the PLL.
 *
 * @param[in] band  The band to be selected.
 *
 * @return The status of the change.
 *
 * @note The caller is responsible for ensuring that the radio no ongoing transactions and
 *  is in the idle state before switching.
 */
xcvrStatus_t XCVR_SetPLLBand(XCVR_BAND_SEL_T band);

/*!
 * @brief Fetches  the current selected band in the PLL.
 *
 * This function enables the upper layers get which frequency band is currently selected.
 *
 * @return The selected band.
 */
XCVR_BAND_SEL_T  XCVR_GetPLLBand(void);

/*!
 * @brief Controls setting the seed for the MSK to FSK transform in the PHY.
 *
 * This function sets the seed value for the PHY RX MSK twiddle function according to the
 * last bit of the expected preamble.
 *
 * @param[in] last_preamble_bit  The last bit of the preamble (in transmit order).
 *
 * @return The status of the change.
 */
xcvrStatus_t XCVR_SetMSKPreambleSeed(uint8_t last_preamble_bit);



/*! @}*/

/*!
 * @addtogroup xcvr_trim Radio Trim and Test Routines
 * @{
 */

/*******************************************************************************
 * Customer level trim functions
 ******************************************************************************/
/*!
 * @brief Controls setting the XTAL trim value.
 *
 * This function enables the upper layers set a crystal trim compensation facor
 *
 * @param[in] xtalTrim  The trim value to apply to the XTAL trimming register. Only the 7 LSB are valid, setting the 8th bit returns an error.
 * @return The status of the trim process.
 */
xcvrStatus_t XCVR_SetXtalTrim(uint8_t xtalTrim);


/*!
 * @brief Controls getting the XTAL trim value.
 *
 * This function enables the upper layers to read the current XTAL compensation factors.
 * The returned value is in the range 0 to 127 (7 bits).
 *
 * @return The XTAL trim compensation factor that is currently programmed.
 */
uint8_t  XCVR_GetXtalTrim(void);

/*!
 * @brief Controls temp compensating the XTAL trim value.
 *
 * This function enables the upper layers set a crystal trim compensation that include temperature compensation.
 *
 * @param[in] temp_comp_lut  The pointer to a LUT for the trim factors across temperature.
 * @param[in] temp_deg_c  The temperature in degrees C.
 * @return The status of the trim process.
 */
xcvrStatus_t XCVR_SetXtalTempComp(const xcvr_temp_comp_lut_t * temp_comp_lut, int8_t temp_deg_c);


/*!
 * @brief  Controls setting the RSSI adjustment.
 *
 * This function enables the upper layers to set an RSSI adjustment value.
 *
 * @param[in] adj  The adjustment value to apply to the RSSI adjustment register. The value must be a signed 8-bit value, in 1/4 dBm step.
 * @return The status of the RSSI adjustment.
 */
xcvrStatus_t XCVR_SetRssiAdjustment(int8_t adj);

/*!
 * @brief  Controls getting the RSSI adjustment.
 *
 * This function enables the upper layers to read the current XCVR RSSI adjustment value.
 *
 * @return The RSSI adjustment value as a signed 8-bit value, representing 1/4 dBm steps..
 */
int8_t  XCVR_GetRssiAdjustment(void);

#if defined(RADIO_IS_GEN_3P5)
/*!
 * @brief  Allows reading an RSSI estimate for the current channel (Wideband).
 *
 * This function enables the upper layers to read the current XCVR RSSI value value.
 *
 * @return The RSSI  value as a signed 8-bit value, representing 1dBm steps..
 * @note This routine forces RX warmup and waits while RSSI is measured before forcing warmdown.
 */
int8_t XCVR_GetInstantRssi(void);
#else
/*!
 * @brief  Allows reading an RSSI estimate for the current channel (Wideband).
 *
 * This function enables the upper layers to read the current XCVR RSSI value value.
 *
 * @return The RSSI  value as a signed 9-bit value (sign extended into 16 bits), representing 1dBm steps..
 * @note This routine forces RX warmup and waits while RSSI is measured before forcing warmdown.
 */
int16_t XCVR_GetInstantRssi(void);
#endif /* defined(RADIO_IS_GEN_3P5) */

/*!
 * @brief  Get the current RSSI value
 *
 * This function retrieves the current RSSI value. If NB_RSSI_AA_MATCH_OVRD_EN is set, the function triggers the
 * RSSI NB measurement and waits until done, else only returns the value triggered by the last packet received.
 *
 * @return The current RSSI value
 */
int16_t XCVR_GetRssiResult(void);

#if defined(RADIO_IS_GEN_3P5) /* Gen 4 and later radios don't include DAC trims */
/*!
 * @brief  Performs the DCOC DAC trim process.
 *
 * This function enables the upper layers to perform the DCOC DAC trim process.
 * The routine stores the results of the trim process at the location pointed to by the input pointer. The contents are invalid in case of error return values.
 *
 * @param[inout] trim_values  Pointer to the location in memory where the routine should store the trim results.
 * @return The status of the trim routine.
 */
xcvrStatus_t XCVR_CalculateDcocDacTrims(xcvr_DcocDacTrim_t * trim_values);

/*!
 * @brief  Controls setting the DCOC DAC trim register contents.
 *
 * This function enables the upper layers to write the current DCOC DAC trim register contents from a structure for usage in initialization of the radio.
 * The routine reads the contents of memory pointed to by the input pointer and applies those values to the trim registers in the radio.
 *
 * @param[in] trim_values  Pointer to the location in memory where the routine should get the trim settngs to be programming into registers.
 * @return The status of the trim setting.
 */
xcvrStatus_t XCVR_SetDcocDacTrims(const xcvr_DcocDacTrim_t * trim_values);

/*!
 * @brief  Controls getting the DCOC DAC trim register contents.
 *
 * This function enables the upper layers to read the current DCOC DAC trim register contents into a structure for storage.
 * The routine reads the trim registers in the radio and writes them out to the location pointed to by the input pointer.
 *
 * @param[inout] trim_values  Pointer to the location in memory where the routine should leave the trim settngs that are read from registers.
 * @return The status of the routine.
 */
xcvrStatus_t XCVR_GetDcocDacTrims(xcvr_DcocDacTrim_t * trim_values);
#endif /* defined(RADIO_IS_GEN_3P5)  */ /* Gen 4 and later radios don't include DAC trims */

/*!
 * @brief Controls setting the PLL to a particular channel using hardware tables.
 *
 * This function enables setting the radio channel for TX and RX. It allows setting the channel based on the hardware channel map tables.
 *
 * @param[in] channel The channel number to set based on protocol specific hardware channel tables.
 * @return The status of the channel over-ride.
  * @note The ::XCVR_ReleasePLLOverride() function must be called to release the PLL over-ride bits which are set by this function. This must be done before
 * any other PLL channel or frequency over-ride function is used OR before the link layer channel control is used.
 */
xcvrStatus_t XCVR_OverrideChannel (uint8_t channel);

/*!
 * @brief Controls setting the PLL to a particular channel using external SW channel table entry.
 *
 * This function enables setting the radio channel for TX and RX. It allows setting the channel based on a software defined table.
 *
 * @param[in] customChannel  If not NULLPTR then this pointer points to a struct containing the integer and numerator values for setting a custom channel.
 * @return The status of the channel over-ride.
  * @note The ::XCVR_ReleasePLLOverride() function must be called to release the PLL over-ride bits which are set by this function. This must be done before
 * any other PLL channel or frequency over-ride function is used OR before the link layer channel control is used.
 */
xcvrStatus_t XCVR_OverrideCustomChannel (const xcvr_pllChannel_t * customChannel);

#if defined(RADIO_IS_GEN_3P5) /* Gen 4 and later radios use low IF receivers and need custom override for RX and TX */
/*!
 * @brief Set an arbitrary frequency for RX and TX for the radio.
 *
 * This function sets the radio frequency used for RX and TX.
 *
 * @param[in] freq  Target frequency setting in Hz.
 * @return Status of the call.
 * @note The ::XCVR_ReleasePLLOverride() function must be called to release the PLL over-ride bits which are set by this function. This must be done before
 * any other PLL channel or frequency over-ride function is used OR before the link layer channel control is used.
 */
    xcvrStatus_t XCVR_OverrideFrequency (uint32_t freq);
#else
/*!
 * @brief Set an arbitrary frequency for RX (only) for the radio.
 *
 * This function sets the radio frequency used for RX. It is required due to the use of a low-IF receiver design which requires
 * an offset from the target frequency to be programmed. This routine makes use of the ::XCVR_OverrideTxFrequency() routine for common functionality.
 *
 * @param[in] freq  Target frequency setting in Hz.
 * @param[in] offset_freq An offset to be applied to the target frequency to either set the RF Intermediate Frequency (IF) or, in special cases, to set both the
 * IF and a much larger offset. Typically this offset should be 1MHz or 1.5MHz. A value of zero will return an error because RX would fail.
 * @return Status of the call.
 * @note The ::XCVR_ReleasePLLOverride() function must be called to release the PLL over-ride bits which are set by this function. This must be done before
 * any other PLL channel or frequency over-ride function is used OR before the link layer channel control is used.
 */
xcvrStatus_t XCVR_OverrideRxFrequency (uint32_t freq, int32_t offset_freq);

/*!
 * @brief Set an arbitrary frequency for TX (only) for the radio.
 *
 * This function sets the radio frequency used for TX. It does not work for RX frequency setting due to the use of a low-IF receiver design which requires
 * an offset from the target frequency to be programmed.
 *
 * @param[in] freq  Target frequency setting in Hz.
 * @return Status of the call.
 * @note The ::XCVR_ReleasePLLOverride() function must be called to release the PLL over-ride bits which are set by this function. This must be done before
 * any other PLL channel or frequency over-ride function is used OR before the link layer channel control is used.
 */
xcvrStatus_t XCVR_OverrideTxFrequency (uint32_t freq);
#endif /* defined(RADIO_IS_GEN_3P5)*/

/*!
 * @brief Releases any PLL channel/frequency over-rides.
 *
 * This function releases any over-rides controlling the PLL frequency and returns the PLL to Link Layer control. It releases the overrides set by
 * ::XCVR_OverrideChannel, ::XCVR_OverrideCustomChannel, ::XCVR_OverrideRxFrequency, or ::XCVR_OverrideTxFrequency.
 *
 * @note This function must be called in between any mix of ::XCVR_OverrideChannel, ::XCVR_OverrideCustomChannel, ::XCVR_OverrideRxFrequency, ::XCVR_OverrideTxFrequency usages.
 * It must also be called in between any of these override functions and link layerchannel register usage.
 */
void XCVR_ReleasePLLOverride(void);

#if defined(RADIO_IS_GEN_3P5) /* Gen 4 and later radios use low IF receivers and need custom override for RX and TX */
/*!
 * @brief Reads the current frequency for RX and TX for the radio.
 *
 * This function reads the radio frequency used for RX and TX.
 *
 * @return Current radio frequency setting in Hz.
 */
 #else
 /*!
  * @brief Reads the current frequency for RX and TX for the radio.
  *
  * This function reads the radio frequency used for RX and TX.
  *
  * @return Current radio frequency setting in Hz.
  * @note If the frequency was set using ::XCVR_OverrideRxFrequency() then this routine returns the sum of the target and offset frequencies.
  */
#endif /* defined(RADIO_IS_GEN_3P5)*/
uint32_t XCVR_GetFreq ( void );


/*!
 * @brief Force receiver warmup.
 *
 * This function forces the initiation of a receiver warmup sequence.
 *
 */
void XCVR_ForceRxWu ( void );


/*!
 * @brief Force receiver warmdown.
 *
 * This function forces the initiation of a receiver warmdown sequence.
 *
 */
void XCVR_ForceRxWd ( void );


/*!
 * @brief Force transmitter warmup.
 *
 * This function forces the initiation of a transmit warmup sequence.
 *
 */
void XCVR_ForceTxWu ( void );


/*!
 * @brief Force transmitter warmdown.
 *
 * This function forces the initiation of a transmit warmdown sequence.
 *
 */
void XCVR_ForceTxWd ( void );

/*!
 * @brief Wait (poll) for completion of receiver warmup.
 *
 * This function polls for the TSM to complete a  receive warmup sequence.
 *
 */
void XCVR_WaitRxWu ( void );

/*!
 * @brief Wait (poll) for completion of transmitter warmup.
 *
 * This function polls for the TSM to complete a  transmit warmup sequence.
 *
 */
void XCVR_WaitTxWu ( void );

/*!
 * @brief Wait (poll) for completion of transmitter or receiver warmdown.
 *
 * This function polls for the TSM to enter IDLE state after a RX or TX warmdown.
 *
 */
void XCVR_WaitRxTxWd(void);

/*!
 * @brief Starts transmit with a TX pattern register data sequence.
 *
 * This function starts transmitting using the DFT pattern register mode.
 *
 * @param[in] channel_num  The protocol specific channel to transmit on. Valid values are defined in the CHANNEL_NUM register documentation.
 * @param[in] xcvr_config  Pointer to a pointer to the complete radio settings structure.
 * @param[in] rbme_config  Pointer to a pointer to the RBME settings structure for coding, CRC, whitening setup.
 * @param[in] tx_pattern   The data pattern to transmit.
 * @return The status of the pattern reg transmit.
 * @note The XCVR_DftTxOff() function must be called to turn off TX and revert all settings. This routine calls XCVR_ChangeMode() with the desired radio mode
 *   and data rate.
 */
xcvrStatus_t XCVR_DftTxPatternReg(uint8_t channel_num, const xcvr_config_t ** xcvr_config, const xcvr_coding_config_t ** rbme_config, uint32_t tx_pattern);

/*!
 * @brief Starts transmit with a TX LFSR register data sequence.
 *
 * This function starts transmitting using the DFT LFSR register mode.
 *
 * @param[in] channel_num  The protocol specific channel to transmit on. Valid values are defined in the CHANNEL_NUM register documentation.
 * @param[in] xcvr_config  Pointer to a pointer to the complete radio settings structure.
 * @param[in] rbme_config  Pointer to a pointer to the RBME settings structure for coding, CRC, whitening setup.
 * @param[in] lfsr_length  The length of the LFSR sequence to use.
 * @return The status of the LFSR reg transmit.
 * @note The XCVR_DftTxOff() function must be called to turn off TX and revert all settings. This routine calls XCVR_ChangeMode() with the desired radio configuration.
 */
xcvrStatus_t XCVR_DftTxLfsrReg(uint8_t channel_num, const xcvr_config_t ** xcvr_config, const xcvr_coding_config_t ** rbme_config, uint8_t lfsr_length);

/*!
 * @brief Controls clearing all TX DFT settings.
 *
 * This function reverts all TX DFT settings from the test modes to normal operating mode.
 *
 */
void XCVR_DftTxOff(void);

/*!
 * @brief Controls a forced setting of the PA power level.
 *
 * This function enables setting the PA power level to a specific setting, overriding any link layer settings.
 *
 * @param[in] pa_power  The power level to set. Valid values are 0, 1, and even values from 2 to 0x3E, inclusive.
 * @return The status of the PA power over-ride.
 */
xcvrStatus_t XCVR_ForcePAPower(uint8_t pa_power);

/*!
 * @brief Controls releasing any forced setting of the PA power level.
 *
 * This function releases the forced setting of the PA power level returning control to the link layer settings.
 *
 */
void XCVR_ReleasePAPower(void);


/*!
 * @brief Starts CW TX.
 *
 * This function starts transmitting CW (no modulation).
 *
 * @param[in] rf_freq  The RF frequency to transmit on in Hz. Valid values are integer values from 2,360,000 to 2,487,000Hz, inclusive.
 *
 * @return The status of the CW transmit.
 */
xcvrStatus_t XCVR_DftTxCW(uint32_t rf_freq);

/*!
 * @brief Function to set AGC to manual and DCOC to auto.
 *
 * This function sets full manual control of AGC and allows DCOC automatic processing to operate normally.
 *
 * @param[in] man_agc_idx  The desired manual AGC index to use (26 to 0).
 * @post
 *    This function disables AGC as a side effect.
 */
xcvrStatus_t XCVR_SetManAgc(uint8_t man_agc_idx);

/*!
 * @brief Function to clear AGC manual operation.
 *
 * This function clears AGC manual mode and allows AGC and DCOC automatic processing to operate normally.
 *
 * @post
 *    This function leaves AGC enabled!
 */
void XCVR_ClearManAgc(void);

/*!
 * @brief Function to set PA power bump to higher power level.
 *
 * This function sets the XCVR LDO registers required to support high power PA operation.
 *
 * @note
 *    The power increase requires DCDC voltage to be increased as well. This is NOT done by the XCVR driver.
 */
void XCVR_bumpPaPowerUp(void);

/*!
 * @brief Function to return PA power bump to the default power level.
 *
 * This function sets the XCVR LDO registers back to the default power level for PA operation.
 *
 * @note
 *    The power level change requires DCDC voltage to be changed as well. This is NOT done by the XCVR driver.
 */
 void XCVR_releasePaPowerBump(void);

/*!
 * @brief Fetches the radio version for this silicon.
 *
 * This function reads the RADIO_VER field and returns an enumeration based on that value.
 *
 * @return The radio version of the silicon.
 */
uint16_t XCVR_ReadRadioVer(void);

#ifdef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
link_layer_t map_mode_to_ll(radio_mode_t mode);
uint16_t XCVR_IdentifyRadioVer(uint32_t radio_ver);
#endif /* !defined(GCOV_DO_COVERAGE) */

/*!
 * @brief Function to override the TX filter to reduce sideband power during TX.
 *
 * This function overrides the TX filter settings for 802.15.4 mode and is intended to be used when transmitting on 
 * channel 11 or 26 at power levels > 0dB. It allows the system to meet regulatory lower/upper band power limits.
 *
 * @param[in] level  The level of filtering to apply 0 through 4 are valid; Zero means release all filtering, 1 - 4 are increasing levels of filtering.
 *
 * @return The status of the filter configuration.
 *
 * @note Increasing levels of filtering have the trade-off of increasing the TX EVM.
 */
xcvrStatus_t XCVR_802p15p4_TxRegulatory(uint8_t level);

/*!
 * @brief Function to override the LDO ANT to force it enabled. It also clears the HiZ setting for LDO_ANT.
 *
 * This function overrides the LDO_ANT regulator to be enabled in order to reduce peaks in current drain for repeated on/off cycles
 * of the LDO_ANT.
 *
 * @note This routine also disables the HiZ mode of LDO_ANT. The XCVR_setLdoAntHiz() function is used to enable HiZ and disable
 * the LDO_ANT override settings.
 */
void XCVR_forceLdoAntEnable(void);

/*!
 * @brief Function to release the overrides for the LDO ANT and enable the HiZ mode.
 *
 * This function releases the overrides the LDO_ANT regulator and enables the HiZ mode. This prevents the discharge of the large
 * capacitor connected to the ouputs of the LDO_ANT.
 *
 * @note This routine also enables the HiZ mode of LDO_ANT. The XCVR_forceLdoAntEnable() function is used to disable HiZ and set
 * the LDO_ANT override settings.
 */
void XCVR_setLdoAntHiz(void);

/*!
 * @brief Function to release the overrides for the LDO ANT *and* release the HiZ mode.
 *
 * This function releases the overrides the LDO_ANT regulator and releases the HiZ mode. This returns the state of the XCVR to it's
 * default state of neither LDO_ANT overridden nor HiZ mode enabled.
 *
 * @note This routine is used to revert the effects of either of the XCVR_forceLdoAntEnable() or XCVR_setLdoAntHiz() APIs and
 * put the XCVR back to a default state for the override and HiZ settings.
 */
void XCVR_releaseLdoAntAll(void);

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470)
/*!
 * @brief Function to trigger start of a fast DMA upload/download of peripheral registers.
 *
 * This function performs a fast DMA download from a set of peripheral registers, based on a descriptor, to an area of PKT RAM memory.
 * It is intended to support fast switchover between RSM (Chanel Sounding) operations and Bluetooth operations..
 *
 * @param[in] pkt_ram_sel  The bank of PKT RAM to use; Zero means use TX PKT_RAM; 1 means use RX PKT_RAM.
 * @param[in] offset_in_pkt_ram  The 32bit word offset within the chosen PKT_RAM bank where the descriptors start.
 * @param[in] upload  If true, the desired function is upload to registers, if false, then download.
 *
 * @pre The descriptors must be loaded into the desired PKT_RAM bank at the specified offset prior to running this routine.
 *
 * @warning This routine does NOT have the ability to check if the descriptors are configured correctly and can produce 
 * unexpected results if the descriptors overrun the targeted PKT_RAM space.
 */
void XCVR_FastPeriphReg_UpDownload_Go(pkt_ram_bank_sel_t pkt_ram_sel, uint32_t offset_in_pkt_ram, bool upload);

/*!
 * @brief Function to load a compressed list of descriptors for fast DMA download of peripheral registers.
 *
 * This function loads a compressed list of descriptors to an area of PKT RAM memory with the proper spacing to allow
 * for the data storage in between each descriptor (after download is complete). The compressed aspect means that the
 * descriptors are passed in as a list with no data spacing in between them.
 * This routine is intended to be used before XCVR_FastPeriphReg_UpDownload() with the upload parameter set to false.
 *
 * @param[in] descriptor_list  Pointer to the list of descriptors to be placed into PKT_RAM.
 * @param[in] num_descrips  The number of descriptors to be loaded..
 * @param[in] pkt_ram_mem_ptr  Pointer to a location in one of the PKT RAM spaces where descriptors/data will be loaded.
 *
 * @return The status of the descriptor load to PKT_RAM.
 *
 * @post The descriptors will be loaded into the desired PKT_RAM bank at the specified offset with spacing in between them
 * and with a single word of all zeros as the end to indicate the completion of the list.
 *
 * @note The XCVR_ValidateFastPeriphDescrip() or other means must be used to validate the input date before calling this routine because overflow protection
 * is not included. The intent is that static data structures can be checked once and then subsequent checks can be omitted. Dynamic descriptor structures must
 * be checked every time.
 */
xcvrStatus_t XCVR_FastPeriphDescrip_Load(uint32_t * descriptor_list, uint16_t num_descrips, volatile uint32_t * pkt_ram_mem_ptr);

/*!
 * @brief Function to load a list of descriptors and data for fast DMA upload of peripheral registers.
 *
 * This function loads a list of mixed descriptors and data to an area of PKT RAM memory for use with XCVR_FastPeriphReg_UpDownload()
 * when a fresh set of data is being uploaded to peripheral registers.
 *
 * @param[in] descriptor_data_list  Pointer to the list of descriptors and data (mixed) to be placed into PKT_RAM.
 * @param[in] total_words_to_load  The number of words in the above mixed list to be loaded. Does not include the trailing zero required 
 * to end the list.
 * @param[in] pkt_ram_mem_ptr  Pointer to a location in one of the PKT RAM spaces where descriptors/data will be loaded.
 *
 * @return The status of the descriptor and data load to PKT_RAM.
 *
 * @post The descriptors and data will be loaded into the desired PKT_RAM bank at the specified offset 
 * and a single word of all zeros will be added to the end to indicate the completion of the list.
 *
 * @note The XCVR_ValidateFastPeriphDescrip() or other means must be used to validate the input date before calling this routine because overflow protection
 * is not included. The intent is that static data structures can be checked once and then subsequent checks can be omitted. Dynamic descriptor structures must
 * be checked every time.
 */
xcvrStatus_t XCVR_FastPeriphDescripData_Load(uint32_t * descriptor_data_list, uint16_t total_words_to_load, volatile uint32_t * pkt_ram_mem_ptr);

/*!
 * @brief Function to validate a descriptor list for fast DMA upload/download of peripheral registers.
 *
 * This function validates a descriptor list to be used in the fast DMA upload or download functionality. It 
 * requires that the  XCVR_FastPeriphDescrip_WordCount() routine first be used to calculate the word count.
 *
 * @param[in] pkt_ram_sel  The bank of PKT RAM to use; Zero means use TX PKT_RAM; 1 means use RX PKT_RAM.
 * @param[in] offset_in_pkt_ram  The 32bit word offset within the chosen PKT_RAM bank where the descriptors should start.
 * @param[in] descriptor_data_list  Pointer to the list of descriptors to be placed into PKT_RAM.
 * @param[in] num_descrips  The number of descriptors to be loaded..
 * @param[in] compressed_list  If true, the desired list only contains descriptors and not data. If false, list is both descriptors and register data.
 * @param[out] pkt_ram_mem_ptr  Pointer to a location in one of the PKT RAM spaces where subsequent routines will load descriptors/data. Invalid if status return is not success.
 *
 * @return The status of the descriptor validation.
 */
xcvrStatus_t XCVR_ValidateFastPeriphDescrip(pkt_ram_bank_sel_t pkt_ram_sel, uint32_t offset_in_pkt_ram, uint32_t * descriptor_data_list,  uint16_t num_descrips, bool compressed_list, volatile uint32_t ** pkt_ram_mem_ptr);

/*!
 * @brief Function to count the number of words occupied by descriptors and register data.
 *
 * This function takes in a set of descriptors and counts the total number of words occupied by the set of descriptors and register 
 * data. It is used to verify sufficient PKT RAM is available..
 *
 * @param[in] descriptor_list  Pointer to the list of descriptors to be placed into PKT_RAM.
 * @param[in] num_descrips  The number of descriptors to be loaded..
 * @param[in] compressed_list  If true, the desired list only contains descriptors and not data. If false, list is both descriptors and register data.
 *
 * @return Thenumber of words it will occupy in PKT RAM.
 */
 uint32_t  XCVR_FastPeriphDescrip_WordCount(uint32_t * descriptor_list, uint16_t num_descrips , bool compressed_list);

/*!
 * @brief Function towait for completion of a FastPeriph to IPS upload or download.
 *
 * This function waits for completion of a Fast Peripheral to IPS upload or download. It completes if the function is not enabled or 
 * if the function is enabled AND the status bit indicates completion of the activities.
 *
 * @return False if an error is asserted at completion of the transfer; True if the error bit is clear.
 *
 * @post The enable bit for the FastPeriph function is cleared by this routine so it will be ready for subsequent usage. 
 */
bool XCVR_FastPeriph_WaitComplete(void);

#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) */


/*! @}*/

#if defined(__cplusplus)
}
#endif


#endif /* NB2P4_XCVR_H */
