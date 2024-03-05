/*
 * Copyright 2018-2019,2021-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */
#ifndef RFMC_CTRL_H
/* clang-format off */
#define RFMC_CTRL_H
/* clang-format on */

#include "fsl_common.h"

/*!
 * @addtogroup RFMC Routines
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RFMC_POWERUP_SUPPORTED (false) /* RFMC doesn't support Radio Powerup/down */

/*!@brief Unique ID. */
typedef struct rfmc_uid_t
{
    uint32_t MH; /*!< UID Radio version  */
    uint32_t ML; /*!< UID MSB. */
    uint32_t L;  /*!< UID LSB  */
} rfmc_uid_t;

/*******************************************************************************
 * API
 ******************************************************************************/
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
 * @brief Function to reset the radio.
 *
 * This function resets the entire radio to it's POR state.
 *
 */
void RFMC_radio_reset(void);

/*!
 * @brief Function to reset the radio.
 *
 * This function resets the entire radio to it's POR state.
 *
 */
void RFMC_GetUniqueId(rfmc_uid_t *uid);

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

#if (RFMC_POWERUP_SUPPORTED)
void RFMC_radio_powerup(void);

void RFMC_radio_powerdown(void);
#endif /* RFMC_POWERUP_SUPPORTED */

#if defined(__cplusplus)
extern "C" {
#endif

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* RFMC_CTRL_H */
