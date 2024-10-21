/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2018-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_common.h"
#include "dbg_ram_capture.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TX_PACKET_RAM_PACKET_RAM_BYTE_COUNT (TX_PACKET_RAM_PACKET_RAM_COUNT * 4U)
#define RX_PACKET_RAM_PACKET_RAM_BYTE_COUNT (RX_PACKET_RAM_PACKET_RAM_COUNT * 4U)

#ifndef GCOV_DO_COVERAGE /* memory ranges are local except when testing code coverage */
#define TX_RAM_START_ADDR (TX_PACKET_RAM_BASE)
#define TX_RAM_END_ADDR (TX_PACKET_RAM_BASE + TX_PACKET_RAM_PACKET_RAM_BYTE_COUNT - 1U)

#define RX_RAM_START_ADDR (RX_PACKET_RAM_BASE)
#define RX_RAM_END_ADDR (RX_PACKET_RAM_BASE + RX_PACKET_RAM_PACKET_RAM_BYTE_COUNT - 1U)
#endif /* !defined(GCOV_DO_COVERAGE) */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#ifndef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
static dbgRamStatus_t unpack_sequential_data(const int16_t *pkt_ram_location,
                                             uint16_t num_bytes,
                                             uint8_t *result_buffer);
static dbgRamStatus_t dbg_ram_set_trigger(const dbg_ram_trigger_config_t *dbg_ram_trigger_config);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN < 450)
static bool IN_TX_RAM_RANGE(uint32_t start, uint32_t stop);
static bool IN_RX_RAM_RANGE(uint32_t start, uint32_t stop);
#endif /* !defined(RADIO_IS_GEN_4P5) */
#endif /* !defined(GCOV_DO_COVERAGE) */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN < 450)
#ifndef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
static bool IN_TX_RAM_RANGE(uint32_t start, uint32_t stop)
#else
bool IN_TX_RAM_RANGE(uint32_t start, uint32_t stop)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    bool result = true;
    if (stop < start) /* error if stop is before start */
    {
        result = false;
    }
    else
    {
        if (start < TX_RAM_START_ADDR) /* below start */
        {
            result = false;
        }
        if (start > TX_RAM_END_ADDR) /* above end */
        {
            result = false;
        }
        if (stop < TX_RAM_START_ADDR) /* below start */
        {
            result = false;
        }
        if (stop > TX_RAM_END_ADDR) /* above end */
        {
            result = false;
        }
    }

    return result;
}

#ifndef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
static bool IN_RX_RAM_RANGE(uint32_t start, uint32_t stop)
#else
bool IN_RX_RAM_RANGE(uint32_t start, uint32_t stop)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    bool result = true;
    if (stop < start) /* error if stop is before start */
    {
        result = false;
    }
    else
    {
        if (start < RX_RAM_START_ADDR) /* below start */
        {
            result = false;
        }
        if (start > RX_RAM_END_ADDR) /* above end */
        {
            result = false;
        }
        if (stop < RX_RAM_START_ADDR) /* below start */
        {
            result = false;
        }
        if (stop > RX_RAM_END_ADDR) /* above end */
        {
            result = false;
        }
    }

    return result;
}
#endif /* !defined(RADIO_IS_GEN_4P5) */

#ifndef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
static bool IN_PKT_RAM_RANGE(uint32_t start, uint32_t stop)
#else
bool IN_PKT_RAM_RANGE(uint32_t start, uint32_t stop)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    bool result = true;
    if (stop < start) /* error if stop is before start */
    {
        result = false;
    }
    else
    {
        if ((start < TX_RAM_START_ADDR) || (start > RX_RAM_END_ADDR)) /* start address is out of bounds */
        {
            result = false;
        }
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN < 450)
        if ((start > TX_RAM_END_ADDR) && (start < RX_RAM_START_ADDR)) /* this branch can't ever be true for Gen 4.5 */
        {
            result = false;
        }
#endif
        if ((stop < TX_RAM_START_ADDR) || (stop > RX_RAM_END_ADDR)) /* stop address is out of bounds */
        {
            result = false;
        }

        if ((stop - TX_PACKET_RAM_BASE) >=
            (TX_PACKET_RAM_PACKET_RAM_BYTE_COUNT +
             RX_PACKET_RAM_PACKET_RAM_BYTE_COUNT)) /* more bytes are requested than exist in the combined packet RAM */
        {
            result = false;
        }
    }

    return result;
}

dbgRamStatus_t dbg_ram_init(const dbg_ram_capture_config_t *dbg_ram_configuration)
{
    dbgRamStatus_t status = DBG_RAM_SUCCESS;
    if (dbg_ram_configuration == NULLPTR)
    {
        status = DBG_RAM_FAIL_NULL_POINTER;
    }
    else
    {
        /* Start and stop addresses are in system RAM space */
        uint32_t start_sys_ram = (uint32_t)dbg_ram_configuration->dbg_ram_start_addr;
        uint32_t stop_sys_ram  = (uint32_t)(start_sys_ram + dbg_ram_configuration->buffer_sz_bytes -
                                           4U); /* due to unsigned math, stop must be > start, no need to error check */

        /* Verify input parameters in configuration structure */
        /* check for invalid address */
        if (IN_PKT_RAM_RANGE(start_sys_ram, stop_sys_ram) && ((start_sys_ram % 4U) == 0U) &&
            ((stop_sys_ram % 4U) == 0U))
        {
#if defined(RADIO_IS_GEN_3P5)
            /* Turns on clocking to DMA/DBG blocks */
            XCVR_RX_DIG->RX_DIG_CTRL |= XCVR_RX_DIG_RX_DIG_CTRL_RX_DMA_DTEST_EN_MASK;
#endif
            /* Initialize desired starting and ending addresses in the debug RAM internal word-based offsets from zero
             */
            uint32_t start_dbg_ram = 0U; /* Start offset within the combined TX and RX packet RAM space,  starting from zero
                                            and counting words */
            uint32_t stop_dbg_ram = 0U;  /* Stop offset within the combined TX and RX packet RAM space,  starting from zero
                                            and counting words */
            /* TX RAM space is lower in memory and RX RAM is above it. Even if they are separate in the system RAM
             * memory map, they are */
            /* made adjacent in the view from the packet RAM. They are addressed in the packet RAM view by offsets
             * starting with zero */
            /* and going up to the total number of 32 bit words in the two packet RAMs */
            if ((start_sys_ram >= TX_RAM_START_ADDR) &&
                (start_sys_ram <= TX_RAM_END_ADDR)) /* starting in the TX RAM space */
            {
                start_dbg_ram = (start_sys_ram - TX_RAM_START_ADDR);
            }
            else
            {
                if ((start_sys_ram >= RX_RAM_START_ADDR) &&
                    (start_sys_ram <= RX_RAM_END_ADDR)) /* starting in the RX RAM space so must adjust by
                                                           TX_PACKET_RAM_PACKET_RAM_COUNT */
                {
                    start_dbg_ram = ((start_sys_ram - RX_RAM_START_ADDR) + TX_PACKET_RAM_PACKET_RAM_BYTE_COUNT);
                }
            }
            if ((stop_sys_ram >= TX_RAM_START_ADDR) &&
                (stop_sys_ram <= TX_RAM_END_ADDR)) /* stopping in the TX RAM space */
            {
                stop_dbg_ram = (stop_sys_ram - TX_RAM_START_ADDR);
            }
            else
            {
                if ((stop_sys_ram >= RX_RAM_START_ADDR) &&
                    (stop_sys_ram <= RX_RAM_END_ADDR)) /* stopping in the RX RAM space so must adjust by
                                                          TX_PACKET_RAM_PACKET_RAM_COUNT */
                {
                    stop_dbg_ram = (stop_sys_ram - RX_RAM_START_ADDR) + TX_PACKET_RAM_PACKET_RAM_BYTE_COUNT;
                }
            }
            XCVR_MISC->DBG_RAM_ADDR =
                XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_FIRST(start_dbg_ram) | XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_LAST(stop_dbg_ram);
        }
        else
        {
            status = DBG_RAM_INVALID_ADDRESS;
        }
    }
    /* Some external code must perform the RX warmup request */

    return status;
}

void dbg_ram_release(void)
{
    /* Release sample capture to packet RAM */
    XCVR_MISC->DBG_RAM_CTRL &= ~XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE_MASK; /* force to idle */
    XCVR_MISC->DBG_RAM_CTRL &= ~(XCVR_MISC_DBG_RAM_CTRL_DBG_EN_MASK); /* turn off packet RAM debug */
#if defined(RADIO_IS_GEN_3P5)
#endif
    /* Some external code must perform the RX warmdown request */
}

/*! *********************************************************************************
 * \brief  Generic routine to unpack sequentially stored data from PKT_RAM to an output buffer.
 *
 * \param[in] pkt_ram_location - The packet RAM location where the unpack should be started (0-PKT_RAM_SIZE_16B_WORDS,
 *in 16 bit words) \param[in] num_bytes - The number of bytes to be unpacked (in bytes) \param[in] result_buffer - The
 *pointer to the output buffer of a size large enough for the samples.
 *
 * \return Result of the unpack.
 *
 * \details
 * This routine starts at the designated location in the specified packet RAM bank (0/1) and unpacks until it has
 * processed the required number of bytes. It makes no inferences about the validity of the unpacked data, that is
 * assumed to be handled outside this routine. It does NOT handle crossing boundaries, it is intended to be called
 * multiple times to implement wraparound and boundary crossings between the two banks of packet RAM.
 ***********************************************************************************/
#ifndef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
static dbgRamStatus_t unpack_sequential_data(const int16_t *pkt_ram_location,
                                             uint16_t num_bytes,
                                             uint8_t *result_buffer)
#else
dbgRamStatus_t unpack_sequential_data(const int16_t *pkt_ram_location, uint16_t num_bytes, uint8_t *result_buffer)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    dbgRamStatus_t status = DBG_RAM_SUCCESS;

    /* Test pointer parameters for non-null */
    if ((result_buffer == NULLPTR) || (pkt_ram_location == NULLPTR))
    {
        status = DBG_RAM_FAIL_NULL_POINTER;
    }
    else
    {
        uint32_t start;
        uint32_t stop;
        uint16_t i;
        volatile const uint8_t *pkt_ram_ptr0 =
            (volatile const uint8_t *)pkt_ram_location; /* never changing the data in the buffer */
        uint8_t *output_ptr = result_buffer;
        /* Verify that the specified packet RAM location resides in one of the packet RAM blocks */
        start = (uint32_t)pkt_ram_location;
        stop  = start + ((uint32_t)num_bytes);
        if (
            //(IN_TX_RAM_RANGE(start, stop) || IN_RX_RAM_RANGE(start, stop)) &&
            IN_PKT_RAM_RANGE(start, stop) && ((start % 4U) == 0U) && ((stop % 4U) == 0U))
        {
            (void)start; /* Workaround unused variable warnings in GCC */
            (void)stop;  /* Workaround unused variable warnings in GCC */

            /* Clear the page selection to ensure capture is stopped */
            XCVR_MISC->DBG_RAM_CTRL &= ~XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE_MASK;
            /* Copy out the required number of bytes */
            for (i = 0; i < num_bytes; i++)
            {
                *output_ptr =
                    *pkt_ram_ptr0; /* Copying one byte at a time to allow misaligned and non-word-length copies */
                pkt_ram_ptr0++;
                output_ptr++;
            }
        }
        else
        {
            status = DBG_RAM_FAIL_SAMPLE_NUM_LIMIT;
        }
    }
    return status;
}

#ifndef GCOV_DO_COVERAGE /* routine is local except when testing code coverage */
static dbgRamStatus_t dbg_ram_set_trigger(const dbg_ram_trigger_config_t *dbg_ram_trigger_config)
#else
dbgRamStatus_t dbg_ram_set_trigger(const dbg_ram_trigger_config_t *dbg_ram_trigger_config)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    dbgRamStatus_t status = DBG_RAM_SUCCESS;
    if (dbg_ram_trigger_config == NULLPTR)
    {
        status = DBG_RAM_FAIL_NULL_POINTER;
    }
    else
    {
        /* check for invalid trigger */
        if (
#if defined(RADIO_IS_GEN_3P5)
            (dbg_ram_trigger_config->start_trig == START_ON_RESERVED1) ||
            (dbg_ram_trigger_config->start_trig == START_ON_RESERVED2) ||
            (dbg_ram_trigger_config->stop_trig == STOP_ON_RESERVED1) ||
            (dbg_ram_trigger_config->stop_trig == STOP_ON_RESERVED2) ||
#endif
            (dbg_ram_trigger_config->start_trig >= INVALID_START_TRIG) ||
            (dbg_ram_trigger_config->stop_trig >= INVALID_STOP_TRIG))
        {
            status = DBG_RAM_INVALID_TRIG_SETTING;
        }

        /* check for invalid start delay */
        if (dbg_ram_trigger_config->start_delay >= DBG_RAM_START_DELAY_MAX)
        {
            status = DBG_RAM_INVALID_START_DELAY;
        }

        /* check for invalid start delay */
        if ((dbg_ram_trigger_config->start_capture_edge >= DBG_RAM_CAPTURE_EDGE_MAX) ||
            (dbg_ram_trigger_config->stop_capture_edge >= DBG_RAM_CAPTURE_EDGE_MAX))
        {
            status = DBG_RAM_INVALID_EDGE_SETTING;
        }

        /* check for invalid decimation setting */
        if (dbg_ram_trigger_config->decimation >= DBG_RAM_DECIMATE_MAX)
        {
            status = DBG_RAM_INVALID_DECIMATE_SETTING;
        }
        /* check for invalid output selection */
        if ((dbg_ram_trigger_config->out_sel >= DBG_RAM_OUT_INVALID_SEL)
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
            || (dbg_ram_trigger_config->out_sel == DBG_RAM_OUT_DISABLED_SEL)
#endif
        )
        {
            status = DBG_RAM_INVALID_OUT_SEL;
        }

        if (status == DBG_RAM_SUCCESS)
        {
            /* Initialize trigger settings for packet RAM capture */
            XCVR_MISC->DBG_RAM_CTRL =
                XCVR_MISC_DBG_RAM_CTRL_DBG_EN_MASK |
                XCVR_MISC_DBG_RAM_CTRL_DBG_START_TRG(dbg_ram_trigger_config->start_trig) |
                XCVR_MISC_DBG_RAM_CTRL_DBG_STOP_TRG(dbg_ram_trigger_config->stop_trig) |
                XCVR_MISC_DBG_RAM_CTRL_DBG_START_DLY(dbg_ram_trigger_config->start_delay) |
                XCVR_MISC_DBG_RAM_CTRL_DBG_DEC(dbg_ram_trigger_config->decimation) |
                XCVR_MISC_DBG_RAM_CTRL_DBG_START_EDGE(dbg_ram_trigger_config->start_capture_edge) |
                XCVR_MISC_DBG_RAM_CTRL_DBG_STOP_EDGE(dbg_ram_trigger_config->stop_capture_edge);

#if defined(RADIO_IS_GEN_3P5)
            /* Configure DMA/DBG output */
            /* RXDIG DFT offset: 0x1FC */
            *((uint32_t *)(XCVR_RX_DIG_BASE + 0x1FCU)) &= ~XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL_MASK;
            *((uint32_t *)(XCVR_RX_DIG_BASE + 0x1FCU)) |=
                XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL(dbg_ram_trigger_config->out_sel);
#else
            /* Configure DMA/DBG output */
            XCVR_RX_DIG->DFT_CTRL &= ~XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL_MASK;
            XCVR_RX_DIG->DFT_CTRL |= XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL(dbg_ram_trigger_config->out_sel);
#endif
        }
    }

    return status;
}

dbgRamStatus_t dbg_ram_postproc_capture(
    const dbg_ram_capture_config_t *dbg_ram_configuration) /* postprocess a capture to unpack data */
{
    dbgRamStatus_t status = DBG_RAM_SUCCESS;

    /* Process samples after capture to packet RAM completed */
    if (dbg_ram_configuration == NULLPTR)
    {
        status = DBG_RAM_FAIL_NULL_POINTER;
    }
    else
    {
        uint16_t buffer_sz_bytes;
        const int16_t *dbg_ram_start_addr = NULLPTR;
        void *result_buffer               = NULLPTR;

        /* Handle the use of both Rx and Tx RAM banks (cross bondaries) */
        uint16_t dbg_ram_first = (uint16_t)((XCVR_MISC->DBG_RAM_ADDR & XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_FIRST_MASK) >>
                                            XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_FIRST_SHIFT);
        uint16_t dbg_ram_last  = (uint16_t)((XCVR_MISC->DBG_RAM_ADDR & XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_LAST_MASK) >>
                                           XCVR_MISC_DBG_RAM_ADDR_DBG_RAM_LAST_SHIFT);

        result_buffer = dbg_ram_configuration->result_buffer;
        /* If Tx packet RAM used */
        if ((dbg_ram_first / 4U) < TX_PACKET_RAM_PACKET_RAM_COUNT)
        {
            dbg_ram_start_addr = (int16_t *)(TX_PACKET_RAM_BASE + dbg_ram_first);
            buffer_sz_bytes    = ((dbg_ram_last / 4U) < TX_PACKET_RAM_PACKET_RAM_COUNT) ?
                                  dbg_ram_configuration->buffer_sz_bytes :
                                  (uint16_t)((TX_PACKET_RAM_PACKET_RAM_COUNT * 4U) - dbg_ram_first);

            status = unpack_sequential_data(dbg_ram_start_addr, buffer_sz_bytes, result_buffer);

            /* add result buffer offset for Rx RAM Bank */
            result_buffer = (uint8_t *)result_buffer + buffer_sz_bytes;
        }

        if (status == DBG_RAM_SUCCESS)
        {
            /* if Rx packet RAM used */
            if ((dbg_ram_last / 4U) >= TX_PACKET_RAM_PACKET_RAM_COUNT)
            {
                dbg_ram_start_addr = ((dbg_ram_first / 4U) >= TX_PACKET_RAM_PACKET_RAM_COUNT) ?
                                         (int16_t *)(RX_PACKET_RAM_BASE + ((uint32_t)dbg_ram_first -
                                                                           (TX_PACKET_RAM_PACKET_RAM_COUNT * 4U))) :
                                         (int16_t *)RX_PACKET_RAM_BASE;
                buffer_sz_bytes = ((dbg_ram_first / 4U) >= TX_PACKET_RAM_PACKET_RAM_COUNT) ?
                                      dbg_ram_configuration->buffer_sz_bytes :
                                      (uint16_t)(dbg_ram_last - (TX_PACKET_RAM_PACKET_RAM_COUNT * 4U));

                status = unpack_sequential_data(dbg_ram_start_addr, buffer_sz_bytes, result_buffer);
            }
        }
    }

    return status;
}

/* blocking wait for capture completion, no matter what trigger type */
void dbg_ram_wait_for_complete(void)
{
    /* Poll on the capture status, waiting for success */
    while (dbg_ram_poll_capture_status() != DBG_RAM_SUCCESS)
    {
    }
}

/* non-blocking completion check, just reads the current status of the capure */
dbgRamStatus_t dbg_ram_poll_capture_status(void)
{
    dbgRamStatus_t status = DBG_RAM_CAPTURE_NOT_COMPLETE;
    /* read the current status and return, don't block in wait loop */
    uint32_t temp         = XCVR_MISC->DBG_RAM_CTRL;
    uint8_t dbg_ram1_full = (uint8_t)(
        (temp & XCVR_MISC_DBG_RAM_CTRL_DBG_RAM_FULL_MASK) >>
        (XCVR_MISC_DBG_RAM_CTRL_DBG_RAM_FULL_SHIFT)); /* Last location has been written in case of start trig capture */
    if ((temp & XCVR_MISC_DBG_RAM_CTRL_DBG_START_TRG_MASK) != 0U)
    {
        /* Start trigger enabled && (sequential || simultaneous) */
        /* DBG_START_TRIGGERED && DBG_RAM_FULL[1] */
        if (((temp & XCVR_MISC_DBG_RAM_CTRL_DBG_START_TRIGGERED_MASK) ==
             XCVR_MISC_DBG_RAM_CTRL_DBG_START_TRIGGERED_MASK) &&
            (dbg_ram1_full == 1U))
        {
            status = DBG_RAM_SUCCESS;
        }
    }
    else
    {
        if ((temp & XCVR_MISC_DBG_RAM_CTRL_DBG_STOP_TRG_MASK) != 0U)
        {
            /* Stop trigger enabled  && (sequential || simultaneous) */
            /* DBG_STOP_TRIGGERED */
            if (((temp & XCVR_MISC_DBG_RAM_CTRL_DBG_STOP_TRIGGERED_MASK)) ==
                XCVR_MISC_DBG_RAM_CTRL_DBG_STOP_TRIGGERED_MASK)
            {
                status = DBG_RAM_SUCCESS;
            }
        }
        else
        {
            /* no start or stop trigger, software initiated */
            /* DBG_RAM_FULL[1] */
            if (dbg_ram1_full == 1U)
            {
                status = DBG_RAM_SUCCESS;
            }
        }
    }

    return status;
}

dbgRamStatus_t dbg_ram_start_capture(dbgRamPageType_t dbg_ram_page,
                                     const dbg_ram_trigger_config_t *dbg_ram_trigger_config)
{
    dbgRamStatus_t status = DBG_RAM_SUCCESS;

    if (dbg_ram_trigger_config == NULLPTR)
    {
        status = DBG_RAM_FAIL_NULL_POINTER;
    }
    else
    {
        /* check for invalid DBG RAM page */
        if ((dbg_ram_page == DBG_RAM_PAGE_IDLE) || (dbg_ram_page >= DBG_RAM_PAGE_MAX))
        {
            status = DBG_RAM_FAIL_PAGE_ERROR;
        }

        else
        {
            /* Start capture using the appropriate trigger */
            status = dbg_ram_set_trigger(dbg_ram_trigger_config);

            if (status == DBG_RAM_SUCCESS)
            {
                /* Program DBG RAM page to start Data capture */
                XCVR_MISC->DBG_RAM_CTRL &= ~XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE_MASK;
                XCVR_MISC->DBG_RAM_CTRL |= XCVR_MISC_DBG_RAM_CTRL_DBG_PAGE(dbg_ram_page);
            }
        }
    }

    return status;
}
