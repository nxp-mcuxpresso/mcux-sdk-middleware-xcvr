/*
 * Copyright 2020-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NXP_XCVR_LCL_CONFIG_H
/* clang-format off */
#define NXP_XCVR_LCL_CONFIG_H
/* clang-format on */

#include "nxp2p4_xcvr.h"

/*!
 * @addtogroup configs Radio Configuration Files
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief TSM register configuration structure containing values to create a T_FC configuration for TSM programming.
 * Requires a T_IP setting to properly finish TSM config and to setup RSM to match. */
typedef struct tag_xcvr_lcl_tsm_config_t
{
    /***********************************************/
    /*********** START OF GENERATED CODE ***********/
    /*********** xcvr_lcl_tsm_config_t **********/
    /***********************************************/

    /* CTRL register value is not modified for RSM operation. */
    /* LPPS_CTRL register value is not modified for RSM operation. */
    uint32_t END_OF_SEQ;
    uint32_t WU_LATENCY;
    uint32_t RECYCLE_COUNT;
    uint32_t FAST_CTRL1;
    uint32_t FAST_CTRL2;
    uint32_t FAST_CTRL3;
    /* TIMING00 register value is not modified for RSM operation. */
    /* TIMING01 register value is not modified for RSM operation. */
    /* TIMING02 register value is not modified for RSM operation. */
    /* TIMING03 register value is not modified for RSM operation. */
    /* TIMING04 register value is not modified for RSM operation. */
    /* TIMING05 register value is not modified for RSM operation. */
    /* TIMING06 register value is not modified for RSM operation. */
    /* TIMING07 register value is not modified for RSM operation. */
    /* TIMING08 register value is not modified for RSM operation. */
    uint32_t TIMING09;
    uint32_t TIMING10;
    uint32_t TIMING11;
    uint32_t TIMING12;
    uint32_t TIMING13;
    uint32_t TIMING14;
    uint32_t TIMING15;
    uint32_t TIMING16;
    uint32_t TIMING17;
    uint32_t TIMING18;
    uint32_t TIMING19;
    uint32_t TIMING20;
    uint32_t TIMING21;
    uint32_t TIMING22;
    uint32_t TIMING23;
    uint32_t TIMING24;
    uint32_t TIMING25;
    uint32_t TIMING26;
    uint32_t TIMING27;
    uint32_t TIMING28;
    uint32_t TIMING29;
    uint32_t TIMING30;
    uint32_t TIMING31;
    uint32_t TIMING32;
    uint32_t TIMING33;
    uint32_t TIMING34;
    uint32_t TIMING35;
    uint32_t TIMING36;
    uint32_t TIMING37;
    uint32_t TIMING38;
    uint32_t TIMING39;
    uint32_t TIMING40;
    uint32_t TIMING41;
    uint32_t TIMING42;
    uint32_t TIMING43;
    uint32_t TIMING44;
    uint32_t TIMING45;
    uint32_t TIMING46;
    uint32_t TIMING47;
    uint32_t TIMING48;
    uint32_t TIMING49;
    uint32_t TIMING50;
    uint32_t TIMING51;
    /* TIMING52 register value is not modified for RSM operation. */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
    uint32_t TIMING53;
    uint32_t TIMING54;
    uint32_t TIMING55;
    uint32_t TIMING56;
    uint32_t TIMING57;
    uint32_t TIMING58;
    uint32_t TIMING59;
    uint32_t TIMING60;
    uint32_t TIMING61;
#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450) */
    /* OVRD0 register value is not modified for RSM operation. */
    /* OVRD1 register value is not modified for RSM operation. */
    /* OVRD2 register value is not modified for RSM operation. */
    /* OVRD3 register value is not modified for RSM operation. */
    /* OVRD4 register value is not modified for RSM operation. */

    /***********************************************/
    /************ END OF GENERATED CODE ************/
    /*********** xcvr_lcl_tsm_config_t **********/
    /***********************************************/
} xcvr_lcl_tsm_config_t;

/*! @brief TSM and RSM bitfield configuration structure containing plug values to create a T_IP configuration based on a
 * T_FC TSM programming. */
typedef struct
{
    uint32_t RSM_CTRL0; /*!<  RSM CTRL0 register configuration for a specific T_IP case. Only intended to cover a subset
                           of the bitfields.  */
    uint32_t RSM_CTRL1; /*!<  RSM CTRL1 register configuration for a specific T_IP case. Intended to cover all of the
                           bitfields.  */
    uint32_t RSM_CTRL2; /*!<  RSM CTRL2 register configuration for a specific T_IP case. Intended to cover all of the
                           bitfields.  */
    uint32_t RSM_CTRL3; /*!<  RSM CTRL3 register configuration for a specific T_IP case. Intended to cover all of the
                           bitfields except RSM_DT_RX_SYNC_DLY & RSM_DMA_RX_EN.  */
    uint32_t RSM_CTRL4; /*!<  RSM CTRL4 register configuration for a specific T_IP case. Intended to cover all of the
                           bitfields.  */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
    uint32_t RSM_CTRL5;
    uint32_t RSM_CTRL6;
    uint32_t RSM_CTRL7;
#endif                   /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450) */
    bool do_rxdig_rccal; /*!<  True indicates PLL will perform RCCAL in this T_IP scenario  */
} xcvr_lcl_rsm_reg_config_t;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* TSM register values, configure these first */
extern const xcvr_lcl_tsm_config_t
    xcvr_lcl_tsm_generic_config; /*!<  TSM register configuration for generic sequence  */
#if (0)
extern const xcvr_lcl_tsm_config_t
    xcvr_lcl_tsm_t_fc_40_config; /*!<  TSM register configuration for every RSM case with T_FC=40  */
extern const xcvr_lcl_tsm_config_t
    xcvr_lcl_tsm_t_fc_80_config; /*!<  TSM register configuration for every RSM case with T_FC=80 */
extern const xcvr_lcl_tsm_config_t
    xcvr_lcl_tsm_t_fc_150_config; /*!<  TSM register configuration for every RSM case with T_FC=150 */
extern const xcvr_lcl_tsm_config_t
    xcvr_lcl_tsm_t_fc_50_config; /*!<  TSM register configuration for every RSM case with T_FC=50 */
#endif
/* TSM & RSM register values, should be configured second, after the above TSM configs */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_generic_config; /*!<  RSM/TSM register configuration for the generic tsm. Meant to overlay after
                                    applying the  ::xcvr_lcl_rsm_generic_config settings */
#if (0)
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_40_t_ip_25_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=40 &
                                            T_IP=25. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_40_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_40_t_ip_40_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=40 &
                                            T_IP=40. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_40_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_80_t_ip_25_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=80 &
                                            T_IP=25. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_80_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_80_t_ip_40_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=80 &
                                            T_IP=40. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_80_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_80_t_ip_80_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=80 &
                                            T_IP=80. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_80_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_150_t_ip_25_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=150 &
                                             T_IP=25. Meant to overlay after applying the
                                             ::xcvr_lcl_tsm_t_fc_150_config settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_150_t_ip_40_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=150 &
                                             T_IP=40. Meant to overlay after applying the
                                             ::xcvr_lcl_tsm_t_fc_150_config settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_150_t_ip_80_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=150 &
                                             T_IP=80. Meant to overlay after applying the
                                             ::xcvr_lcl_tsm_t_fc_150_config settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_150_t_ip_150_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=150 &
                                              T_IP=150. Meant to overlay after applying the
                                              ::xcvr_lcl_tsm_t_fc_150_config settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_50_t_ip_25_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=50 &
                                            T_IP=25. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_40_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_50_t_ip_40_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=50 &
                                            T_IP=40. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_40_config
                                            settings */
extern const xcvr_lcl_rsm_reg_config_t
    xcvr_lcl_rsm_t_fc_50_t_ip_50_config; /*!<  RSM/TSM register configuration for the specific RSM case of T_FC=50 &
                                            T_IP=50. Meant to overlay after applying the  ::xcvr_lcl_tsm_t_fc_40_config
                                            settings */
#endif
/*******************************************************************************
 * Code
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* NXP_XCVR_LCL_CONFIG_H */
