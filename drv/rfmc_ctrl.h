/*
 * Copyright 2018-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */
#ifndef RFMC_CTRL_H
/* clang-format off */
#define RFMC_CTRL_H
/* clang-format on */

#include "fsl_common.h"

/*!
 * @addtogroup rfmc RFMC Routines
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef RFMC_POWERUP_SUPPORTED
#define RFMC_POWERUP_SUPPORTED (true) /* RFMC doesn't support Radio Powerup/down */
#endif                                /* RFMC_POWERUP_SUPPORTED */

/*! @brief Error codes for the RFMC driver. */
typedef enum
{
    gRfmcSuccess_c = 0,
    gRfmcInvalidParameters_c,
    gRfmcUnsupportedOperation_c,
    gRfmcInvalidConfiguration_c,
    gRfmcConfigurationFailure_c,
} rfmcStatus_t;

/*! @brief RFGPO_SRC source definitions for the 8 available outputs. */
/* COEX_3_0 = coext[3:0] = {rf_priority[1:0], rf_status,rf_active} */
/* FEM_3_0 = fem_ctrl[3:0] = {ant_b, ant_b, tx_switch, rx_switch} */
/* LANT_3_0 = lant_lut_gpio[3:0] = values from antenna GPO LUT */

typedef enum
{
    RFMC_GPO_COEX_3_0_FEM_3_0  = 0, /*!< coext[3:0] + fem_ctrl[3:0]  */
    RFMC_GPO_FEM_3_0_COEX_3_0  = 1, /*!< fem_ctrl[3:0] + coext[3:0]   */
    RFMC_GPO_LANT_3_0_FEM_3_0  = 2, /*!< lant_lut_gpio[3:0] + fem_ctrl[3:0]   */
    RFMC_GPO_FEM_3_0_LANT_3_0  = 3, /*!< fem_ctrl[3:0] + lant_lut_gpio[3:0]   */
    RFMC_GPO_LANT_3_0_COEX_3_0 = 4, /*!< lant_lut_gpio[3:0] + coext[3:0]   */
    RFMC_GPO_COEX_3_0_LANT_3_0 = 5, /*!< coext[3:0] + lant_lut_gpio[3:0]  */
    RFMC_GPO_INVALID           = 6  /*!< Invalid selection.  */
} rfmcGpoCoex_t;

/*! @brief Callback indices for the RFMC driver interrupt handler. */
typedef enum
{
    RFMC_XO_RDY_STAT      = 0,
    RFMC_XO_INT_STAT      = 1,
    RFMC_XO_EXT_STAT      = 2,
    RFMC_RF_BLE_WKUP_STAT = 3,
    RFMC_RF_MAN_WKUP_STAT = 4,
    RFMC_RF_WOR_WKUP_STAT = 5,
    RFMC_RF_LP_WKUP_STAT  = 6,
    RFMC_RF_ACTIVE_STAT   = 7,
    RFMC_NUM_CALLBACKS
} rfmcIrqIndex_t;

/*!@brief Unique ID. */
typedef struct rfmc_uid_t
{
    uint32_t MH; /*!< UID Radio version  */
    uint32_t ML; /*!< UID MSB. */
    uint32_t L;  /*!< UID LSB  */
} rfmc_uid_t;

/*!
 * @brief RFMC callback function type
 *
 * The RFMC callback function is provided to allow the RFMC interrupt to call user specified routines
 * to handle interrupts.
 *
 *@note The RFMC IRQ handler will pass the entire RF2P4GHZ_STAT and XO_STAT registers to the
 * user callback to support interpretation of the current state of the RFMC.
 */
typedef void (*rfmc_cb_fptr)(uint32_t rf2p4ghz_stat_reg, uint32_t xo_stat_reg);

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Register a callback for the RFMC interrupts.
 *
 * This function registers a callback to allow handing RFMC interrupts.
 *
 * @param[in] fptr  The function pointer to a RFMC callback. Pass in NULLPTR to de-register a callback.
 * @param[in] cb_index the location in an array of callbacks in which to store the new callback pointer. Allows
 *specifying same or different callback for each interrupt supported by the RFMC.
 *
 *@return True is returned if successful, false otherwise.
 */
bool RFMC_RegisterCb(rfmc_cb_fptr fptr, rfmcIrqIndex_t cb_index); /* allow caller to provide RFMC callback */

/*!
 * @brief Enabled or disable one of the RFMC interrupts.
 *
 * This function allows the caller to enable or disable one of the  RFMC interrupts.
 *
 * @param[in] irq_index the index of the interrupt to be enabled/disabled.
 * @param irq_enabled True if interrupt should be enabled, false if it should be disabled.
 *
 *@return True is returned if successful, false otherwise.
 */
bool RFMC_IrqEnDis(rfmcIrqIndex_t irq_index, bool irq_enabled);

/*!
 * @brief Function to Startup RF OSC.
 *
 * This function enables RF OSC and waits for ready indication.
 *
 * @note
 *  This function may wait an indeterminate amount of time if RF OSC enable doesn't ever indicate ready.
 */
void RFMC_rf_osc_startup(void);

/*!
 * @brief Function to shutdown RF OSC.
 *
 * This function disables RF OSC.
 *
 */
void RFMC_rf_osc_shutdown(void);

/*!
 * @brief Function to return the state of radio warmup (including RF OSC enable).
 *
 * This function returns the state of the RF_OSC warmup, the exit of radio from reset, and
 * the radio exit from low power. It can optionally wait until the status is true before returning.
 *
 * @param wait_for_complete True if the routine should poll on the status until true; False if immediate status
 * should be returned.
 *
 * @return True if the warmup is complete, false otherwise.
 */
bool RFMC_check_radio_warmup_complete(bool wait_for_complete);

/*!
 * @brief Function to reset the radio.
 *
 * This function resets the entire radio internal logic to it's POR state. It does not reset the registers, only the internal state.
 *
 */
void RFMC_radio_reset(void);

/*!
 * @brief Function to read the unique ID.
 *
 * This function reads the unique ID and returns it to the calling routine via a pointer.
 *
 */
void RFMC_GetUniqueId(rfmc_uid_t *uid);

/*!
 * @brief Function to setup the GPO selections for coexistence, FEM, and antenna control.
 *
 * This function configures the 8 GPO pins to support a mix of coexistence, Front End Module,
 * and antenna control usage. The allocation of pins to these functions and the output buffer enables
 * are both configured.
 *
 * @param gpo_setting the selection of how to allocate functions to the available GPO's.
 * @param gpo_output_buf_ena the bit-by-bit enable setting for the output buffers.
 */
rfmcStatus_t RFMC_SetRfGpoConfig(rfmcGpoCoex_t gpo_setting, uint8_t gpo_output_buf_ena);

/*!
 * @brief Function to set XTAL trim value.
 *
 * This function sets a trim for the XTAL.
 *
 */
void RFMC_SetXtalTrim(uint8_t xtalTrim);

/*!
 * @brief Function to get the XTAL trim value.
 *
 * This function gets the trim valuefor the XTAL.
 *
 */
uint8_t RFMC_GetXtalTrim(void);

/*!
 * @brief Function to handle dispatch of interrupts to registered callbacks.
 *
 * This function dispatches interrupts from the RFMC_IRQHandler to registered callbacks, along with the
 * value of the RF and XO status registers at the time of the interrupt.
 *
 * @param temp_register_rf the state of the RF status register upon entering the IRQ handler.
 * @param temp_register_xo the state of the XO status register upon entering the IRQ handler..
 */
#if defined(GCOV_DO_COVERAGE) /* routine is linked externally when testing code coverage */
void RFMC_DispatchIRQ(uint32_t temp_register_rf, uint32_t temp_register_xo);
#endif /* !defined(GCOV_DO_COVERAGE) */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN>=450) && !defined(KW45B41Z82_NBU_SERIES) && !defined(KW45B41Z83_NBU_SERIES)
/*!
 * @brief Function to put the NBU CPU into reset state.
 *
 * This function puts the NBU CPU into reset state from the RFMC.
 *
 */
void RFMC_nbu_enter_reset(void);

/*!
 * @brief Function to release the NBU CPU from reset state.
 *
 * This function releases the NBU CPU from it's reset state from the RFMC.
 *
 * @note After releasing NBU CPU from reset there should be some mechanism for waiting for NBU to
 * complete startup before continuing with RF operations from the CM33 side. Otherwise, CM3 (NBU) and
 * CM33 may issue conflicting RFMC or XCVR operations.
 */
void RFMC_nbu_release_reset(void);

#endif /* defined(RADIO_IS_GEN_4P5) && !defined(KW45B41Z82_NBU_SERIES) && !defined(KW45B41Z83_NBU_SERIES) */

#if defined(__cplusplus)
extern "C" {
#endif

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* RFMC_CTRL_H */
