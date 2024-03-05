/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2018-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NXP_XCVR_OQPSK_802P15P4_CONFIG_H
/* clang-format off */
#define NXP_XCVR_OQPSK_802P15P4_CONFIG_H
/* clang-format on */

#include "nxp2p4_xcvr.h"
#include "nxp_xcvr_common_config.h" /* make common configs available to the mode config file for pointer struct assembly */

/*!
 * @addtogroup configs Radio Configuration Files
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! @brief Mode OQPSK 802P15P4 250kbps specific configuration */
extern const xcvr_mode_datarate_config_t xcvr_oqpsk_802p15p4_250kbps_config;

/* COMPLETE CONFIG STRUCTURES */
/*! @brief  OQPSK 802.15.4 250Kbps  with 2Mbps undefined rate configuration. */
extern const xcvr_config_t xcvr_oqpsk_802p15p4_250kbps_full_config;

#if defined(__cplusplus)
extern "C" {
#endif

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* NXP_XCVR_OQPSK_802P15P4_CONFIG_H */
