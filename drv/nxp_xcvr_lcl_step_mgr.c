/*
 * Copyright 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "nxp_xcvr_lcl_step_mgr.h"
#include <stdio.h>

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) && \
    (RF_OSC_26MHZ == 0) /* RSM step manager is new on Gen 4.7 radios & with 32MHZ RF_OSC */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
const uint8_t rtt_payload_sizes[7] = {0U, 2U, 6U, 2U, 4U, 6U, 8U};
cs_subevent_info_t subevent_info;

/* Circular buffer related static data - supports setup of buffer state and IRQ operation */
static uint8_t total_step_count;
static uint8_t remaining_configs_to_load; /*!< Running total of the number of config steps remaining to be programmed */
static uint8_t remaining_results_to_read; /*!< Running total of the number of result steps remaining to be read */
static uint8_t step_irq_count;            /*!< Stores the number of steps for each step interrupt */
static uint8_t cfg_step_length_words[4];  /*!< Stores the configuration step lengths for a RTT type and NUM_AP
                                             combination. Used to speed processing. */
static uint8_t res_step_length_words[4]; /*!< Stores the result step lengths for a RTT type and NUM_AP combination. Used
                                            to speed processing. */
static uint32_t *curr_config_in_ptr;  /*!< Pointer to the location in system RAM where the entire subevent configuration
                                         sequence is stored */
static uint32_t *curr_result_out_ptr; /*!< Pointer to the location in system RAM where the entire subevent results are
                                         to be stored */
static uint32_t
    *config_pkt_ram_buffer; /*!< Pointer to the configuration buffer in PKT RAM (either TX or RX); Access it by array
                               accesses. Checks at initialization ensure the buffer is properly sized */
static uint32_t
    *result_pkt_ram_buffer; /*!< Pointer to the result buffer in PKT RAM (either TX or RX); Access it by array accesses.
                               Checks at initialization ensure the buffer is properly sized */
static uint16_t config_rollover_index;
static uint16_t result_rollover_index;
static bool in_sniffer_mode; /*!< track whether in sniffer mode to support proper results handling */

/* Visibility for debugging circular buffer operations */
#define DEBUG_CIRCULAR_BUFF (0)
#if defined(DEBUG_CIRCULAR_BUFF) && (DEBUG_CIRCULAR_BUFF == 1)
#define MAX_IRQ_COUNT (80)
static uint16_t irq_count = 0U;
static uint32_t *cfg_ptr_in_irq[MAX_IRQ_COUNT];
static uint32_t *res_ptr_in_irq[MAX_IRQ_COUNT];
static uint32_t *cfg_ptr_end_irq[MAX_IRQ_COUNT];
static uint32_t *res_ptr_end_irq[MAX_IRQ_COUNT];
static uint32_t cfg_ptr_register_start_val[MAX_IRQ_COUNT];
static uint32_t cfg_ptr_register_end_val[MAX_IRQ_COUNT];
static uint32_t res_ptr_register_start_val[MAX_IRQ_COUNT];
static uint32_t res_ptr_register_end_val[MAX_IRQ_COUNT];
static uint32_t rsm_ptr_register_val[MAX_IRQ_COUNT];
static uint8_t results_remaining_end_val[MAX_IRQ_COUNT];
#endif /* DEBUG_CIRCULAR_BUFF */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(USE_LCL_STEP_MACROS) && (USE_LCL_STEP_MACROS == 1)
/* Macro defined in header file */
#else
/* Function version */
uint16_t XCVR_LCL_MakeStepCfg(XCVR_RSM_FSTEP_TYPE_T step_format,
                              XCVR_RSM_T_PM_FM_SEL_T t_pm_sel,
                              uint8_t ant_permut,
                              uint8_t tone_ext,
                              uint8_t oneway,
                              uint8_t ant_cs_sync)
{
    /* Create the common uint16_t combined step configuration portion of headers */
    return (uint16_t)(
        COM_MODE_013_CFG_HDR_STEP_CFG_MODE(step_format) | COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT(tone_ext) |
        COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT(ant_permut) | COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC(ant_cs_sync) |
        COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY(oneway) | COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL(t_pm_sel));
}
#endif /* defined(USE_LCL_STEP_MACROS) && (USE_LCL_STEP_MACROS==1) */

uint8_t XCVR_LCL_ReadStepCfg(COM_MODE_013_CFG_HDR_Type *header_ptr,
                             XCVR_RSM_FSTEP_TYPE_T *step_format,
                             XCVR_RSM_T_PM_FM_SEL_T *t_pm_sel,
                             uint8_t *ant_permut,
                             uint8_t *tone_ext,
                             uint8_t *oneway,
                             uint8_t *ant_cs_sync)
{
    xcvrLclStatus_t status;
    uint8_t cfg_sz;
    uint8_t res_sz;

    /* Read (from PKT RAM  memory) the common uint16_t combined step configuration portion of headers */
    /* Using COM_MODE_013_CFG_HDR_Type is ok because all headers are identical in the relevant portion of the structure
     */
    uint16_t temp = header_ptr->STEP_CFG;

    /* Update the various pointers with the contents of the header */
    *step_format = (XCVR_RSM_FSTEP_TYPE_T)((temp & COM_MODE_013_CFG_HDR_STEP_CFG_MODE_MASK) >>
                                           COM_MODE_013_CFG_HDR_STEP_CFG_MODE_SHIFT);
    *t_pm_sel    = (XCVR_RSM_T_PM_FM_SEL_T)((temp & COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL_MASK) >>
                                         COM_MODE_013_CFG_HDR_STEP_CFG_T_PM_SEL_SHIFT);
    *ant_permut =
        (temp & COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_ANT_PERMUT_SHIFT;
    *tone_ext = (temp & COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_TONE_EXT_SHIFT;
    *oneway = (temp & COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_EP_ONE_WAY_SHIFT;
    *ant_cs_sync =
        (temp & COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_ANT_CS_SYNC_SHIFT;

    /* Return the length of the header as the function return value */
    XCVR_RSM_RTT_TYPE_T rtt_type = (XCVR_RSM_RTT_TYPE_T)(
        (XCVR_MISC->RSM_CTRL2 & XCVR_MISC_RSM_CTRL2_RSM_RTT_TYPE_MASK) >> XCVR_MISC_RSM_CTRL2_RSM_RTT_TYPE_SHIFT);
    status = XCVR_LCL_CalcConfigResult_Size(*step_format, rtt_type, 1U, &cfg_sz,
                                            &res_sz); /* Don't care about num_ap param, only want cfg_sz result */
    (void)status;
    (void)res_sz;

    return cfg_sz;
}

#if defined(USE_LCL_STEP_MACROS) && (USE_LCL_STEP_MACROS == 1)
/* Macro defined in header file */
#else
xcvrLclStatus_t XCVR_LCL_MakeMode013CfgHeader(xcvr_lcl_mode_0_cfg_t *cfg_entry,
                                              uint16_t channel_num,
                                              uint16_t ctune,
                                              uint16_t step_cfo,
                                              uint16_t hpm_cal,
                                              uint16_t phase_add,
                                              uint16_t step_cfg,
                                              uint32_t *access_address)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (cfg_entry == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        uint32_t *aa_ptr              = access_address;
        cfg_entry->header.STEP_CFG    = (uint16_t)step_cfg; /* This is the output of XCVR_LCL_MakeStepCfg() function */
        cfg_entry->header.CHANNEL_NUM = channel_num;        /* Entry is full 16 bits wide so no need to mask/shift */
        cfg_entry->header.STEP_CFO    = step_cfo;           /* Entry is full 16 bits wide so no need to mask/shift */
        cfg_entry->header.HPM_CAL_FACTOR = COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR(
            hpm_cal >> 1U); /* Field stores the 12MSBs of HPM_CAL_FACTOR:  HPM_CAL_FACTOR[12:1] */
        cfg_entry->header.CTUNE_MANUAL = COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL(ctune);
        cfg_entry->header.PHASE_ADD    = COM_MODE_013_CFG_HDR_PHASE_ADD_PHASE_ADD(phase_add);
        cfg_entry->header.AA_INIT      = *aa_ptr++;
        cfg_entry->header.AA_REFL      = *aa_ptr;
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_MakeMode2CfgHeader(xcvr_lcl_mode_2_cfg_t *cfg_entry,
                                            uint16_t channel_num,
                                            uint16_t ctune,
                                            uint16_t step_cfo,
                                            uint16_t hpm_cal,
                                            uint32_t step_cfg)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (cfg_entry == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        cfg_entry->header.STEP_CFG    = (uint16_t)step_cfg; /* This is the output of XCVR_LCL_MakeStepCfg() function */
        cfg_entry->header.CHANNEL_NUM = channel_num;        /* Entry is full 16 bits wide so no need to mask/shift */
        cfg_entry->header.STEP_CFO    = step_cfo;           /* Entry is full 16 bits wide so no need to mask/shift */
        cfg_entry->header.HPM_CAL_FACTOR = COM_MODE_013_CFG_HDR_HPM_CAL_FACTOR_HPM_CAL_FACTOR(hpm_cal);
        cfg_entry->header.CTUNE_MANUAL   = COM_MODE_013_CFG_HDR_CTUNE_MANUAL_CTUNE_MANUAL(ctune);
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_MakeMode13CfgPayload(xcvr_lcl_mode_1_3_cfg_t *cfg_entry,
                                              XCVR_RSM_RTT_TYPE_T rtt_type,
                                              uint32_t *random_payload1,
                                              uint32_t *random_payload2)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if (cfg_entry == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        const uint32_t sounding_data = 0xAAAAAAAAU; /* Channel sounding data is constant */
        bool increment_ptr = false; /* Controls whether pointers are incremented during the payload copy process */
        uint8_t num_words  = 0U;    /* How many words of payload are to be copied */
        const uint32_t *payload_ptr1 = random_payload1;
        const uint32_t *payload_ptr2 = random_payload2;
        switch (rtt_type)
        {
            case XCVR_RSM_RTT_NO_PAYLOAD:
                num_words = 0U;
                break;
            case XCVR_RSM_RTT_32BIT_SOUNDING:
                num_words     = 1U;
                increment_ptr = false;
                payload_ptr1  = &sounding_data;
                payload_ptr2  = &sounding_data;
                break;
            case XCVR_RSM_RTT_96BIT_SOUNDING:
                num_words     = 3U;
                increment_ptr = false;
                payload_ptr1  = &sounding_data;
                payload_ptr2  = &sounding_data;
                break;
            case XCVR_RSM_RTT_32BIT_RANDOM:
                num_words     = 1U;
                increment_ptr = true;
                break;
            case XCVR_RSM_RTT_64BIT_RANDOM:
                num_words     = 2U;
                increment_ptr = true;
                break;
            case XCVR_RSM_RTT_96BIT_RANDOM:
                num_words     = 3U;
                increment_ptr = true;
                break;
            case XCVR_RSM_RTT_128BIT_RANDOM:
                num_words     = 4U;
                increment_ptr = true;
                break;
            default:
                status = gXcvrLclStatusInvalidArgs;
                break;
        }
        if (increment_ptr) /* this flag aligns with the random payload case so use it to determine when to check for
                              NULLPTR */
        {
            if ((random_payload1 == NULLPTR) || (random_payload2 == NULLPTR))
            {
                status = gXcvrLclStatusInvalidArgs;
            }
        }
        /* Load the channel sounding or random payload words */
        if (status == gXcvrLclStatusSuccess)
        {
            uint8_t i;
            for (i = 0U; i < num_words; i++)
            {
                cfg_entry->payload_body.PAYLOAD[i].INIT = *payload_ptr1;
                cfg_entry->payload_body.PAYLOAD[i].REFL = *payload_ptr2;
                if (increment_ptr)
                {
                    payload_ptr1++;
                    payload_ptr2++;
                }
            }
        }
    }

    return status;
}
#endif /* defined(USE_LCL_STEP_MACROS) && (USE_LCL_STEP_MACROS==1) */

xcvrLclStatus_t XCVR_LCL_CalcConfigResult_Size(XCVR_RSM_FSTEP_TYPE_T step_format,
                                               XCVR_RSM_RTT_TYPE_T rtt_type,
                                               uint8_t num_ap,
                                               uint8_t *config_size_words,
                                               uint8_t *result_size_words)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer */
    if ((config_size_words == NULLPTR) || (result_size_words == NULLPTR))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    /* Support disabling error checking for code which has been tested */
#if defined(CHECK_FSTEP_ERRORS)
    if (num_ap > 4U)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    if (step_format >= XCVR_RSM_STEP_ERROR) /* invalid step format */
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    if (rtt_type >= XCVR_RSM_RTT_ERROR) /* invalid RTT type */
    {
        status = gXcvrLclStatusInvalidArgs;
    }
#endif /* defined(CHECK_FSTEP_ERRORS) */
    if (status == gXcvrLclStatusSuccess)
    {
        /* Calculate configuration and result sizes for combination of step type and RTT selection */
        switch (step_format)
        {
            case XCVR_RSM_STEP_FCS:
                *config_size_words = 5U;
                *result_size_words = 5U;
                break;
            case XCVR_RSM_STEP_PK_PK:
                *config_size_words = 5U + rtt_payload_sizes[rtt_type];
                *result_size_words = 5U;
                break;
            case XCVR_RSM_STEP_TN_TN:
                *config_size_words = 3U;
                *result_size_words = 1U + num_ap + 1U;
                break;
            case XCVR_RSM_STEP_PK_TN_TN_PK:
                *config_size_words = 5U + rtt_payload_sizes[rtt_type];
                *result_size_words = 5U + num_ap + 1U;
                break;
            default:
                *config_size_words = 0U;
                *result_size_words = 0U;
                status             = gXcvrLclStatusInvalidArgs;
                break;
        }
    }
    return status;
}

xcvrLclStatus_t XCVR_LCL_ValidateSubeventInfo(cs_subevent_info_t *subevent_info_ptr,
                                              uint16_t pkt_ram_cfg_buffer_sz_words,
                                              uint16_t pkt_ram_res_buffer_sz_words)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Input pointer is not NULLPTR */
    if (subevent_info_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* None of the pointers in subevent_info is NULLPTR */
        if ((subevent_info_ptr->aa_list == NULLPTR) || (subevent_info_ptr->payload_list_init == NULLPTR) ||
            (subevent_info_ptr->payload_list_refl == NULLPTR) || (subevent_info_ptr->channel_list == NULLPTR) ||
            (subevent_info_ptr->hpm_cal_list == NULLPTR) || (subevent_info_ptr->step_cfg_list == NULLPTR))
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        /* num_steps is > 0 and <= 127 */
        if ((subevent_info_ptr->num_steps == 0U) || (subevent_info_ptr->num_steps > 127U))
        {
            status = gXcvrLclStatusInvalidArgs;
        }

        /* sequence through steps and count the size of all steps */
        if (status == gXcvrLclStatusSuccess)
        {
            uint8_t i;
            XCVR_RSM_FSTEP_TYPE_T *step_type_ptr = subevent_info_ptr->step_type;
            uint16_t total_cfg_sz                = 0U;
            uint16_t total_res_sz                = 0U;
            uint8_t res_mult_factor              = (subevent_info_ptr->is_sniffer_mode ?
                                           2U :
                                           1U); /* Multiplication factor for results size when in sniffer mode */
            for (i = 0; i < (subevent_info_ptr->num_steps); i++)
            {
                uint8_t cfg_sz;
                uint8_t res_sz;
                xcvrLclStatus_t temp_status = XCVR_LCL_CalcConfigResult_Size(
                    *step_type_ptr++, subevent_info_ptr->rtt_type, subevent_info_ptr->num_ap, &cfg_sz, &res_sz);
                if (temp_status == gXcvrLclStatusSuccess)
                {
                    total_cfg_sz += cfg_sz;
                    total_res_sz += (res_sz * res_mult_factor);
                }
                else
                {
                    status = gXcvrLclStatusFail;
                    break;
                }
            }
            if (status == gXcvrLclStatusSuccess)
            {
                if ((total_cfg_sz > pkt_ram_cfg_buffer_sz_words) || (total_res_sz > pkt_ram_res_buffer_sz_words))
                {
                    status = gXcvrLclStatusInvalidLength;
                }
            }
        }
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_ProgramFstepRam(cs_subevent_info_t *subevent_info_ptr,
                                         cs_pkt_ram_config_info_t *pkt_ram_info_ptr,
                                         uint32_t *sys_ram_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Error checking for NULL pointer  */
    if ((subevent_info_ptr == NULLPTR) || ((pkt_ram_info_ptr == NULLPTR) && (sys_ram_ptr == NULLPTR)))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
#if defined(CHECK_FSTEP_ERRORS)
        /* Check that channel index is valid (each step) */
        /* Check HPM CAL value doesn't exceed the bitfield size  (each step) */
#endif /* defined(CHECK_FSTEP_ERRORS) */
        /* Program each step into the PKT RAM or system RAM starting at the pkt_ram_buffer_offset_words location */
        /* Must use pointers rather than arrays for the input streams because they won't track exactly with step number
         */
        bool load_directly_to_pkt_ram =
            (sys_ram_ptr == NULLPTR); /* If sys_ram_ptr is not NULL then load to system RAM instead of PKT RAM */
        uint8_t i;
        uint16_t curr_addr                   = 0U; /* Current address offset into the buffer within the PKT RAM */
        XCVR_RSM_FSTEP_TYPE_T *step_type_ptr = subevent_info_ptr->step_type;
        uint32_t *aa_list_ptr                = subevent_info_ptr->aa_list;
        uint32_t *payload_list_init_ptr      = subevent_info_ptr->payload_list_init;
        uint32_t *payload_list_refl_ptr      = subevent_info_ptr->payload_list_refl;
        uint16_t *channel_list_ptr           = subevent_info_ptr->channel_list;
        uint16_t *hpm_cal_list_ptr           = subevent_info_ptr->hpm_cal_list;
        uint16_t *step_cfg_list_ptr          = subevent_info_ptr->step_cfg_list;
        uint16_t *step_cfo_ptr               = subevent_info_ptr->cfo_list;
        uint16_t *phase_add_ptr              = subevent_info_ptr->phase_add_list;
        static COM_MODE_CFG_HDR_UNION_Type step_header; /* Used to assemble header structures for write to PKT RAM */
        COM_MODE_CFG_HDR_UNION_Type *step_hdr_ptr = &step_header;
        for (i = 0; i < (subevent_info_ptr->num_steps); i++)
        {
            uint8_t cfg_sz = 0U;
            uint8_t res_sz = 0U;
            /* Calculate the size of the config and result (only need the config now) */
            xcvrLclStatus_t temp_status = XCVR_LCL_CalcConfigResult_Size(*step_type_ptr, subevent_info_ptr->rtt_type,
                                                                         subevent_info_ptr->num_ap, &cfg_sz, &res_sz);
            if (temp_status == gXcvrLclStatusSuccess)
            {
                /* Fill out the configuration header & payload based on the step type */
                xcvr_lcl_mode_0_1_3_cfg_t *mode1_3_step_ptr = NULLPTR;
                xcvr_lcl_mode_2_cfg_t *mode2_step_ptr       = NULLPTR;
                switch (*step_type_ptr)
                {
                    case XCVR_RSM_STEP_FCS:
                        /* Cast a structure onto the PKT RAM or system RAM */
                        if (load_directly_to_pkt_ram)
                        {
                            mode1_3_step_ptr = (xcvr_lcl_mode_0_1_3_cfg_t *)GET_PKT_RAM_PTR(
                                pkt_ram_info_ptr->config_pkt_ram_bank,
                                (pkt_ram_info_ptr->config_base_addr_word) + curr_addr);
                        }
                        else
                        {
                            mode1_3_step_ptr = (xcvr_lcl_mode_0_1_3_cfg_t *)sys_ram_ptr;
                            sys_ram_ptr      = sys_ram_ptr + cfg_sz; /* Update the system RAM pointer */
                        }
                        /* Mode 0 steps have no payload */
#if (USE_LCL_STEP_MACROS)
                        /* Generate data in local struct and write */
                        XCVR_LCL_MakeCommonHeader(step_hdr_ptr, *channel_list_ptr, 0U, *step_cfo_ptr, *hpm_cal_list_ptr,
                                                  *phase_add_ptr, *step_cfg_list_ptr);
                        XCVR_LCL_WriteCommonHeader((COM_MODE_CFG_HDR_UNION_Type *)&(mode1_3_step_ptr->header),
                                                   step_hdr_ptr);
                        XCVR_LCL_MakeMode013_Payload(mode1_3_step_ptr, aa_list_ptr[0], aa_list_ptr[1], (cfg_sz - 5U),
                                                     payload_list_init_ptr,
                                                     payload_list_refl_ptr); /* Zero payload words */
#else
                        temp_status |= XCVR_LCL_MakeMode013CfgHeader(
                            mode1_3_step_ptr, *channel_list_ptr, 0U, /* CTUNE not used */
                            *step_cfo_ptr, *hpm_cal_list_ptr, *phase_add_ptr, *step_cfg_list_ptr, aa_list_ptr);
#endif
                        if (!subevent_info_ptr->phy_test_mode) /* In PHY test mode we always use the first pair of AA so
                                                                  never increment */
                        {
                            aa_list_ptr++;
                            aa_list_ptr++;
                        }
                        /* There is never a payload on Mode 0 steps so no need to increment payload pointers */
                        phase_add_ptr++;
                        break;
                    case XCVR_RSM_STEP_PK_PK:
                    case XCVR_RSM_STEP_PK_TN_TN_PK:
                        /* Mode 1 & 3 steps have identical configurations */
                        if (load_directly_to_pkt_ram)
                        {
                            mode1_3_step_ptr = (xcvr_lcl_mode_0_1_3_cfg_t *)GET_PKT_RAM_PTR(
                                pkt_ram_info_ptr->config_pkt_ram_bank,
                                (pkt_ram_info_ptr->config_base_addr_word) + curr_addr);
                        }
                        else
                        {
                            mode1_3_step_ptr = (xcvr_lcl_mode_0_1_3_cfg_t *)sys_ram_ptr;
                            sys_ram_ptr      = sys_ram_ptr + cfg_sz; /* Update the system RAM pointer */
                        }
#if (USE_LCL_STEP_MACROS)
                        /* Generate data in local struct and write to PKT RAM*/
                        XCVR_LCL_MakeCommonHeader(step_hdr_ptr, *channel_list_ptr, 0U, *step_cfo_ptr, *hpm_cal_list_ptr,
                                                  *phase_add_ptr, *step_cfg_list_ptr);
                        XCVR_LCL_WriteCommonHeader((COM_MODE_CFG_HDR_UNION_Type *)&(mode1_3_step_ptr->header),
                                                   step_hdr_ptr);
                        XCVR_LCL_MakeMode013_Payload(
                            mode1_3_step_ptr, aa_list_ptr[0], aa_list_ptr[1], (cfg_sz - 5U) >> 1, payload_list_init_ptr,
                            payload_list_refl_ptr); /* (cfg_sz-5) payload words, both payload list pointers are
                                                       incremented inside the macro  */
#else
                        temp_status |= XCVR_LCL_MakeMode013CfgHeader(
                            (xcvr_lcl_mode_0_cfg_t *)mode1_3_step_ptr, /* Mode 0/1/3 are identical for the header*/
                            *channel_list_ptr, 0U,                     /* CTUNE not used */
                            *step_cfo_ptr, *hpm_cal_list_ptr, *phase_add_ptr, *step_cfg_list_ptr, aa_list_ptr);
                        temp_status |= XCVR_LCL_MakeMode13CfgPayload(mode1_3_step_ptr, subevent_info_ptr->rtt_type,
                                                                     payload_list_init_ptr, payload_list_refl_ptr);

#endif
                        if (!subevent_info_ptr->phy_test_mode) /* In PHY test mode we always use the first pair of AA so
                                                                  never increment */
                        {
                            aa_list_ptr++;
                            aa_list_ptr++;
                        }
                        else
                        {
                            /* Because payload pointers are incremented inside the macro, we need to undo that increment
                             * when in test mode */
                            payload_list_init_ptr -= ((cfg_sz - 5U) >> 1);
                            payload_list_refl_ptr -= ((cfg_sz - 5U) >> 1);
                        }
                        phase_add_ptr++;
                        break;
                    case XCVR_RSM_STEP_TN_TN:
                        /* Cast a structure onto the PKT RAM */
                        if (load_directly_to_pkt_ram)
                        {
                            mode2_step_ptr = (xcvr_lcl_mode_2_cfg_t *)GET_PKT_RAM_PTR(
                                pkt_ram_info_ptr->config_pkt_ram_bank,
                                (pkt_ram_info_ptr->config_base_addr_word) + curr_addr);
                        }
                        else
                        {
                            mode2_step_ptr = (xcvr_lcl_mode_2_cfg_t *)sys_ram_ptr;
                            sys_ram_ptr    = sys_ram_ptr + cfg_sz; /* Update the system RAM pointer */
                        }
#if (USE_LCL_STEP_MACROS)
                        /* Generate data in local struct and write to PKT RAM*/
                        XCVR_LCL_MakeCommonHeader(step_hdr_ptr, *channel_list_ptr, 0U, *step_cfo_ptr, *hpm_cal_list_ptr,
                                                  *phase_add_ptr, *step_cfg_list_ptr);
                        XCVR_LCL_WriteCommonHeader((COM_MODE_CFG_HDR_UNION_Type *)&(mode2_step_ptr->header),
                                                   step_hdr_ptr);
#else
                        temp_status |=
                            XCVR_LCL_MakeMode2CfgHeader(mode2_step_ptr, *channel_list_ptr, 0U, /* CTUNE not used */
                                                        *step_cfo_ptr, *hpm_cal_list_ptr, *step_cfg_list_ptr);

#endif

                        /* Mode 2 steps have no payload */
                        break;
                    default:
                        status = gXcvrLclStatusInvalidArgs;
                        break;
                }
                /* Common pointer updates that apply to all step types (step specific types should be updated above */
                channel_list_ptr++;
                step_cfo_ptr++;
                hpm_cal_list_ptr++;
                step_cfg_list_ptr++;
                step_type_ptr++;
                curr_addr += cfg_sz; /* Update the current offset into the buffer within PKT RAM */
            }
            else
            {
                status = gXcvrLclStatusFail;
                break;
            }
        }
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_InitCfgResPointers(cs_pkt_ram_config_info_t *pkt_ram_info_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    //    static uint32_t * temp;
    //    temp = (uint32_t *)&TX_PACKET_RAM[0];
    //    config_pkt_ram_buffer = &temp[2];
    //    (void)temp;
    //    config_pkt_ram_buffer = (uint32_t *)&TX_PACKET_RAM[1];
    //    config_pkt_ram_buffer = (uint32_t *)&TX_PACKET_RAM[2];
    //    config_pkt_ram_buffer = (uint32_t *)&TX_PACKET_RAM[3];
    /* Error checking for NULL pointer  */
    if (pkt_ram_info_ptr == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
#if defined(XCVR_SKIP_RSM_SETTINGS_CHECK) && (XCVR_SKIP_RSM_SETTINGS_CHECK == 0)
        status = XCVR_LCL_CheckPktRamCfg(pkt_ram_info_ptr);
#endif /* defined(XCVR_SKIP_RSM_SETTINGS_CHECK) && (XCVR_SKIP_RSM_SETTINGS_CHECK == 0) */
        if (status == gXcvrLclStatusSuccess)
        {
            XCVR_MISC->RSM_CONFIG_BUFF =
                (XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR(pkt_ram_info_ptr->config_base_addr_word) |
                 XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BUFF_LOC(pkt_ram_info_ptr->config_pkt_ram_bank) |
                 XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_DEPTH(pkt_ram_info_ptr->config_depth_word) |
                 XCVR_MISC_RSM_CONFIG_BUFF_RSM_INT_NBSTEP(pkt_ram_info_ptr->interrupt_step_count));
            XCVR_MISC->RSM_RESULT_BUFF =
                (XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR(pkt_ram_info_ptr->result_base_addr_word) |
                 XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BUFF_LOC(pkt_ram_info_ptr->result_pkt_ram_bank) |
                 XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_DEPTH(pkt_ram_info_ptr->result_depth_word));
            status                = XCVR_LCL_ResetCfgResPointers();
            step_irq_count        = pkt_ram_info_ptr->interrupt_step_count;
            config_rollover_index = pkt_ram_info_ptr->config_base_addr_word + pkt_ram_info_ptr->config_depth_word;
            result_rollover_index = pkt_ram_info_ptr->result_base_addr_word + pkt_ram_info_ptr->result_depth_word;
            in_sniffer_mode       = pkt_ram_info_ptr->is_sniffer_mode;

            /* Setup buffer pointers to support array access to the config and result buffers in PKT RAM */
            uint32_t *temp_arr_ptr;
            if (pkt_ram_info_ptr->config_pkt_ram_bank == TX_PKT_RAM_SEL)
            {
                temp_arr_ptr = (uint32_t *)(&TX_PACKET_RAM[0U]);
            }
            else
            {
                temp_arr_ptr = (uint32_t *)(&RX_PACKET_RAM[0U]);
            }
            config_pkt_ram_buffer = &temp_arr_ptr[0U]; /* Later processing needs to work with the base of the PKT RAM to
                                                          properly track rollover and index values */
            if (pkt_ram_info_ptr->result_pkt_ram_bank == TX_PKT_RAM_SEL)
            {
                temp_arr_ptr = (uint32_t *)(&TX_PACKET_RAM[0U]);
            }
            else
            {
                temp_arr_ptr = (uint32_t *)(&RX_PACKET_RAM[0U]);
            }
            result_pkt_ram_buffer = &temp_arr_ptr[0U]; /* Later processing needs to work with the base of the PKT RAM to
                                                          properly track rollover and index values */
        }
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_ResetCfgResPointers(void)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Read in the base address and set the config ptr to that value */
    uint32_t temp_start       = ((XCVR_MISC->RSM_CONFIG_BUFF & XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR_MASK) >>
                           XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR_SHIFT);
    uint32_t temp_wrap        = ((XCVR_MISC->RSM_CONFIG_BUFF & XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_DEPTH_MASK) >>
                          XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_DEPTH_SHIFT);
    XCVR_MISC->RSM_CONFIG_PTR = (XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_START_PTR(temp_start) |
                                 XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PTR(temp_start) | 0U);
    temp_start                = ((XCVR_MISC->RSM_RESULT_BUFF & XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR_MASK) >>
                  XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR_SHIFT);
    temp_wrap                 = ((XCVR_MISC->RSM_RESULT_BUFF & XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_DEPTH_MASK) >>
                 XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_DEPTH_SHIFT);
    (void)temp_wrap;
    XCVR_MISC->RSM_RESULT_PTR = (XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_START_PTR(temp_start) |
                                 XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PTR(temp_start) |
#if (1)
                                 XCVR_MISC_RSM_RESULT_PTR_RSM_BUFFER_ABORT_EN_MASK
#else
                                 0U
#endif /* defined(DEBUG_CIRCULAR_BUFF) && (DEBUG_CIRCULAR_BUFF == 1) */
    );
    return status;
}

xcvrLclStatus_t XCVR_LCL_LoadConfigSteps(uint32_t **current_cfg_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    static uint32_t temp_cfg_val;
    /* Copy the step interval count number of steps from the current_cfg pointer into PKT RAM based on the current
     * CONFIG write pointer */
    /* If fewer than step interval steps are remaining then copy that many instead */
    uint8_t i, j;
    uint32_t *temp_cur_cfg_ptr = *current_cfg_ptr; /* Local pointer copy */
    uint8_t count              = step_irq_count;
    if (remaining_configs_to_load < step_irq_count)
    {
        count = remaining_configs_to_load;
    }

    /* Load configs */
    /* Get configuration write pointer */
    uint32_t temp_config_ptr_reg = XCVR_MISC->RSM_CONFIG_PTR;
    uint16_t config_index        = (temp_config_ptr_reg & XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PTR_MASK) >>
                            XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PTR_SHIFT;
    uint8_t config_page = (temp_config_ptr_reg & XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PAGE_MASK) >>
                          XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PAGE_SHIFT;
    assert(config_index < config_rollover_index);

    for (i = 0U; i < count; i++)
    {
        /* Check for rollover - use max result size of *any* step to determine when to rollover (if the biggest step
         * won't fit then rollover) */
        if (cfg_step_length_words[3U] >
            (config_rollover_index - config_index)) // TODO: check rollover logic for off by 1 case
        {
            // reset PKT RAM pointer to base of buffer
            config_index = ((XCVR_MISC->RSM_CONFIG_BUFF & XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR_MASK) >>
                            XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR_SHIFT);
            config_page  = (~config_page) & 0x1U; /* invert only 1 bit */
        }
        /* Read step data (header) and calculate length in words */
        uint32_t step_data = (*temp_cur_cfg_ptr) >> 16U; /* First word always includes STEP_CFG in upper 16 bits */
        uint8_t mode =
            (step_data & COM_MODE_013_CFG_HDR_STEP_CFG_MODE_MASK) >> COM_MODE_013_CFG_HDR_STEP_CFG_MODE_SHIFT;
        uint8_t length = cfg_step_length_words[mode];
        for (j = 0U; j < length; j++)
        {
#if (0)
            config_pkt_ram_buffer[config_index] = *temp_cur_cfg_ptr;
#else
            temp_cfg_val                        = *temp_cur_cfg_ptr;
            config_pkt_ram_buffer[config_index] = temp_cfg_val;
#endif
            config_index++;
            temp_cur_cfg_ptr++;
        }
        remaining_configs_to_load--;
        /* Check for rollover - in the case of the configs just exactly filled the buffer */
        if (config_index == config_rollover_index)
        {
            // reset PKT RAM pointer to base of buffer
            config_index = ((XCVR_MISC->RSM_CONFIG_BUFF & XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR_MASK) >>
                            XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_BASE_ADDR_SHIFT);
            config_page  = (~config_page) & 0x1U; /* invert only 1 bit */
        }
    }
    *current_cfg_ptr = temp_cur_cfg_ptr; /* Update global pointer to the config system RAM storage */
    /* Update the configuration pointer register in RSM */
    temp_config_ptr_reg &=
        ~(XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PTR_MASK | XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PAGE_MASK);
    temp_config_ptr_reg |= XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PTR(config_index) |
                           XCVR_MISC_RSM_CONFIG_PTR_RSM_CONFIG_WR_PAGE(config_page);
    XCVR_MISC->RSM_CONFIG_PTR = temp_config_ptr_reg;

    return status;
}

xcvrLclStatus_t XCVR_LCL_ReadResultSteps(uint32_t **current_res_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Copy the step interval count number of results from PKT RAM to the current_cfg pointer based on the current
     * RESULT read pointer */
    /* If fewer than step interval steps are remaining then copy that many instead */
    uint8_t i, j;
    uint32_t *temp_cur_res_ptr = *current_res_ptr; /* Local pointer copy */
    uint8_t count              = step_irq_count;
    if (remaining_results_to_read < step_irq_count)
    {
        count = remaining_results_to_read;
    }

    /* Read results */
    /* Get result read pointer */
    uint32_t temp_result_ptr_reg = XCVR_MISC->RSM_RESULT_PTR;
    uint16_t result_index        = (temp_result_ptr_reg & XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PTR_MASK) >>
                            XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PTR_SHIFT;
    uint8_t result_page = (temp_result_ptr_reg & XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PAGE_MASK) >>
                          XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PAGE_SHIFT;
    assert(result_index < result_rollover_index);

    for (i = 0U; i < count; i++)
    {
        /* Check for rollover - use max result size of *any* step to determine when to rollover (if the biggest step
         * won't fit then rollover) */
        if (res_step_length_words[3U] >
            (result_rollover_index - result_index)) // TODO: check rollover logic for off by 1 case
        {
            // reset PKT RAM pointer to base of buffer
            result_index = ((XCVR_MISC->RSM_RESULT_BUFF & XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR_MASK) >>
                            XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR_SHIFT);
            result_page  = (result_page == 0U ? 1U : 0U); /* Single bit invert */
        }
        /* Read step data (header) and calculate length in words */
        uint32_t step_data =
            (result_pkt_ram_buffer[result_index]) >> 8U; /* First word always includes RESULT_SIZE in second byte */
        uint8_t length =
            (step_data & COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE_MASK) >> COM_RES_HDR_SIZE_AGC_IDX_RESULT_SIZE_SHIFT;
#if defined(DEBUG_CIRCULAR_BUFF) && (DEBUG_CIRCULAR_BUFF == 1)
        assert(length < 21); /* make sure length is correct */
#endif
        for (j = 0U; j < length; j++)
        {
            *temp_cur_res_ptr = result_pkt_ram_buffer[result_index];
            result_index++;
            temp_cur_res_ptr++;
        }
        /* In sniffer mode there is always another result of the exact same length */
        if (in_sniffer_mode)
        {
            for (j = 0U; j < length; j++)
            {
                *temp_cur_res_ptr = result_pkt_ram_buffer[result_index];
                result_index++;
                temp_cur_res_ptr++;
            }
        }
        remaining_results_to_read--;
        /* If last result chunk just filled the results buffer then must rollover to the start */
        if (result_index == result_rollover_index)
        {
            // reset PKT RAM pointer to base of buffer
            result_index = ((XCVR_MISC->RSM_RESULT_BUFF & XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR_MASK) >>
                            XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_BASE_ADDR_SHIFT);
            result_page  = (result_page == 0U ? 1U : 0U); /* Single bit invert */
        }
    }
    *current_res_ptr = temp_cur_res_ptr; /* Update global pointer to the result system RAM storage */
    /* Update the result pointer register in RSM */
    temp_result_ptr_reg &=
        ~(XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PTR_MASK | XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PAGE_MASK);
    temp_result_ptr_reg |= XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PTR(result_index) |
                           XCVR_MISC_RSM_RESULT_PTR_RSM_RESULT_RD_PAGE(result_page);
    XCVR_MISC->RSM_RESULT_PTR = temp_result_ptr_reg;

    return status;
}

xcvrLclStatus_t XCVR_LCL_SetupInitialConfigs(uint8_t total_num_steps,
                                             uint32_t *config_in_ptr,
                                             uint32_t *results_out_ptr,
                                             XCVR_RSM_RTT_TYPE_T rtt_type,
                                             uint8_t num_ap,
                                             bool sniffer_mode)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Total number of steps in the sequence. One variable keeps the total throughout the sequence, the other 2 count
     * down as the configs */
    /* are loaded and results are read */
    total_step_count          = total_num_steps;
    remaining_configs_to_load = total_num_steps; /* Decrements as configs are loaded */
    remaining_results_to_read = total_num_steps; /* Decrements as results are read */

    /* Prep for routines to quickly calculate addresses based on step sizes */
    for (uint8_t i = 0U; i < 4; i++)
    {
        status |= XCVR_LCL_CalcConfigResult_Size((XCVR_RSM_FSTEP_TYPE_T)i, rtt_type, num_ap, &cfg_step_length_words[i],
                                                 &res_step_length_words[i]);
        if (sniffer_mode)
        {
            res_step_length_words[i] =
                res_step_length_words[i] * 2U; /* Twice as many results in each step in sniffer mode */
        }
    }
#if defined(DEBUG_CIRCULAR_BUFF) && (DEBUG_CIRCULAR_BUFF == 1)
    /* Reset debug visibility data */
    irq_count = 0U;
    memset((void *)&cfg_ptr_in_irq[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&res_ptr_in_irq[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&cfg_ptr_end_irq[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&res_ptr_end_irq[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&cfg_ptr_register_start_val[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&cfg_ptr_register_end_val[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&res_ptr_register_start_val[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&res_ptr_register_end_val[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&rsm_ptr_register_val[0], 0x0, (MAX_IRQ_COUNT)*4);
    memset((void *)&results_remaining_end_val[0], 0x0, (MAX_IRQ_COUNT));
#endif

    /* Load 2 buffers worth of configs (where each is ::step_irq_count in length) */
    curr_config_in_ptr  = config_in_ptr;   /* place the pointer to the input buffer of all configs into static local */
    curr_result_out_ptr = results_out_ptr; /* place the pointer to the output buffer of all results into static local */
    status              = XCVR_LCL_LoadConfigSteps(&curr_config_in_ptr);
    status |= XCVR_LCL_LoadConfigSteps(&curr_config_in_ptr);

    return status;
}

xcvrLclStatus_t XCVR_LCL_FinishFinalResults()
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    (void)total_step_count;
    return status;
}

xcvrLclStatus_t XCVR_LCL_CheckPktRamCfg(cs_pkt_ram_config_info_t *pkt_ram_info_ptr)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    //    uint32_t temp;
    /* Check CONFIG_DEPTH and RESULT_DEPTH < max depth in register */
    uint32_t max_size = XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_DEPTH_MASK >>
                        XCVR_MISC_RSM_CONFIG_BUFF_RSM_CONFIG_DEPTH_SHIFT; /* gets the max value of the register */
    /* Track args check by using all statements that evaluate to false (0 value) if param is ok */
    /* Sum up every status check and compare to zero at the end. If not == zero then return invalid args result */
    uint32_t tmp_logic_sum = 0U;
    /* Zero depth check */
    tmp_logic_sum += pkt_ram_info_ptr->config_depth_word == 0U;
    tmp_logic_sum += pkt_ram_info_ptr->result_depth_word == 0U;
    tmp_logic_sum += pkt_ram_info_ptr->interrupt_step_count == 0U;
    /* Check config buffer depth */
    tmp_logic_sum += pkt_ram_info_ptr->config_depth_word > max_size;
    /* Check reult buffer depth */
    max_size = XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_DEPTH_MASK >>
               XCVR_MISC_RSM_RESULT_BUFF_RSM_RESULT_DEPTH_SHIFT; /* gets the max value of the register */
    tmp_logic_sum += pkt_ram_info_ptr->result_depth_word > max_size;
    uint16_t config_end_index = pkt_ram_info_ptr->config_base_addr_word + pkt_ram_info_ptr->config_depth_word;
    uint16_t result_end_index = pkt_ram_info_ptr->result_base_addr_word + pkt_ram_info_ptr->result_depth_word;
    /* Check that config depth + base address doesn't exceed the selected PKT RAM end  - RX_PACKET_RAM_PACKET_RAM_COUNT
     * or TX_PACKET_RAM_PACKET_RAM_COUNT_DBL_BUFFER */
    max_size = (pkt_ram_info_ptr->config_pkt_ram_bank == TX_PKT_RAM_SEL ? TX_PACKET_RAM_PACKET_RAM_COUNT_DBL_BUFFER :
                                                                          RX_PACKET_RAM_PACKET_RAM_COUNT);
    tmp_logic_sum += config_end_index > max_size;
    /* Check that result depth + base address doesn't exceed the selected PKT RAM end */
    max_size = (pkt_ram_info_ptr->result_pkt_ram_bank == TX_PKT_RAM_SEL ? TX_PACKET_RAM_PACKET_RAM_COUNT_DBL_BUFFER :
                                                                          RX_PACKET_RAM_PACKET_RAM_COUNT);
    tmp_logic_sum += result_end_index > max_size;
    /* If result and config buffers are in the same PKT RAM then check for non-overlap between the two buffers */
    if (pkt_ram_info_ptr->config_pkt_ram_bank == pkt_ram_info_ptr->result_pkt_ram_bank)
    {
        /* already know that both end index are within the PKT RAM block, just check that the buffers don't overlap */
        if (pkt_ram_info_ptr->config_base_addr_word <
            pkt_ram_info_ptr->result_base_addr_word) /* config buffer is lower in memory */
        {
            tmp_logic_sum += config_end_index > pkt_ram_info_ptr->result_base_addr_word;
        }
        else /* result buffer is lower in memory */
        {
            tmp_logic_sum += result_end_index > pkt_ram_info_ptr->config_base_addr_word;
        }
    }
    // TODO: implement all checks on packet ram configuration

    if (tmp_logic_sum != 0U)
    {
        status = gXcvrLclStatusInvalidArgs;
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_HandleIrqStepEos(int32_t status_bits)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
#if defined(DEBUG_CIRCULAR_BUFF) && (DEBUG_CIRCULAR_BUFF == 1)
    assert(irq_count < MAX_IRQ_COUNT);
    cfg_ptr_in_irq[irq_count]             = curr_config_in_ptr;
    res_ptr_in_irq[irq_count]             = curr_result_out_ptr;
    cfg_ptr_register_start_val[irq_count] = XCVR_MISC->RSM_CONFIG_PTR;
    res_ptr_register_start_val[irq_count] = XCVR_MISC->RSM_RESULT_PTR;
    rsm_ptr_register_val[irq_count]       = XCVR_MISC->RSM_PTR;
#endif /* DEBUG_CIRCULAR_BUFF */

    /* Service the configs and results */
    status = XCVR_LCL_LoadConfigSteps(&curr_config_in_ptr);
    status |= XCVR_LCL_ReadResultSteps(&curr_result_out_ptr);

    /* Clear STEP and EOS interrupts */
    //    XCVR_MISC->RSM_INT_STATUS |= XCVR_MISC_RSM_INT_STATUS_RSM_IRQ_STEP_MASK |
    //    XCVR_MISC_RSM_INT_STATUS_RSM_IRQ_STEP_MASK;

#if defined(DEBUG_CIRCULAR_BUFF) && (DEBUG_CIRCULAR_BUFF == 1)
    cfg_ptr_end_irq[irq_count]           = curr_config_in_ptr;
    res_ptr_end_irq[irq_count]           = curr_result_out_ptr;
    cfg_ptr_register_end_val[irq_count]  = XCVR_MISC->RSM_CONFIG_PTR;
    res_ptr_register_end_val[irq_count]  = XCVR_MISC->RSM_RESULT_PTR;
    results_remaining_end_val[irq_count] = remaining_results_to_read;
    irq_count++;
#endif /* DEBUG_CIRCULAR_BUFF */

    return status;
}

#endif /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 470) && (RF_OSC_26MHZ == 0) */
