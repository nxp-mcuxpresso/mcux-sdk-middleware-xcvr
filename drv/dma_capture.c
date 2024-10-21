/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2018-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "dma_capture.h"
#include "math.h"
#include "mathfp.h" /* custom fixed point math for atan2 function */

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/
/* DMA capture configuration structs */
static dsb_handler_t dsb_handler;
static uint32_t *destination_address =
    NULLPTR;                     /* Static storage for dest address to support multiple captures without re-init */
static uint32_t buffer_size = 0; /* Static storage for buffer size to support multiple captures without re-init */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void DSB_IRQHandler(void);

void DSB_IRQHandler(void)
{
    /* CCNT field is cleared at transfer complete so use TCNT for count of bytes transferred */
    uint16_t curr_count = (uint16_t)((DSB->XCR & DSB_XCR_TCNT_MASK) >> DSB_XCR_TCNT_SHIFT);
    bool transfer_done  = ((DSB->INT & DSB_INT_DONE_MASK) == DSB_INT_DONE_MASK);
    /* Only call routine if transfer is complete */
    if (transfer_done)
    {
        /* Clear done flag */
        DSB->INT |= DSB_INT_DONE_MASK;
        dsb_handler.g_Transfer_Done = true;
    }
    else /* transfer is not done, an error happened */
    {
        curr_count = (uint16_t)(DSB->INT & 0xFFFFU); /* Return error interrupt bits in current count field */
        /* Clear DSB errors */
        DSB->INT |= DSB_INT_DBE_MASK | DSB_INT_UNDR_MASK | DSB_INT_OVRF_MASK;
    }

    /* Make sure the function pointer isn't NULLPTR */
    if (dsb_handler.dsb_config.user_callback != NULLPTR)
    {
        dsb_handler.dsb_config.user_callback(dsb_handler.dsb_config.userData, dsb_handler.g_Transfer_Done, curr_count);
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/
dmaStatus_t dma_refresh(const dsb_config_t *dsb_configuration)
{
    dmaStatus_t status = DMA_SUCCESS;

    /* Verify input parameters in configuration structure */
    if (dsb_configuration == NULLPTR)
    {
        status = DMA_FAIL_NULL_POINTER;
    }
    else
    {
        /* check for NULL Pointer to destination buffer */
        if (dsb_configuration->result_buffer == NULLPTR)
        {
            status = DMA_FAIL_NULL_POINTER;
        }

        /* check for 0 buffer size */
        if (dsb_configuration->buffer_sz == 0U)
        {
            status = DMA_FAIL_SAMPLE_NUM_LIMIT;
        }

        if (status == DMA_SUCCESS)
        {
            dsb_handler.g_Transfer_Done = false;

            /* Initialize the DADDR with the destination buffer location in system memory. */
            destination_address =
                (uint32_t *)dsb_configuration->result_buffer; /* Store addr for multiple start capture calls */
            buffer_size = dsb_configuration->buffer_sz;       /* Store size for multiple start capture calls */

            XCVR_MISC->DMA_CTRL &=
                ~(XCVR_MISC_DMA_CTRL_DMA_PAGE_MASK); /*Clear the DMA_PAGE to stop the (still) ongoing radio DMA */
        }
    }

    return status;
}

dmaStatus_t dma_init(const dsb_config_t *dsb_configuration)
{
    dmaStatus_t status = DMA_SUCCESS;

    /* Verify input parameters in configuration structure */
    if (dsb_configuration == NULLPTR)
    {
        status = DMA_FAIL_NULL_POINTER;
    }
    else
    {
        /* check for NULL Pointer to destination buffer */
        if (dsb_configuration->result_buffer == NULLPTR)
        {
            status = DMA_FAIL_NULL_POINTER;
        }

        /* check for zero byte capture request */
        if (dsb_configuration->buffer_sz == 0U)
        {
            status = DMA_FAIL_SAMPLE_NUM_LIMIT;
        }

        /* check if watermark exceeds FIFO size */
        if (dsb_configuration->watermark >= DSB_FIFO_SIZE)
        {
            status = DMA_INVALID_WTRMRK_VALUE;
        }

        /* only continue if prior checks all passed */
        if (status == DMA_SUCCESS)
        {
            /* Configure DMA handler */
            const void *ptr =
                memcpy((void *)&dsb_handler.dsb_config, (const void *)dsb_configuration, sizeof(dsb_config_t));
            /* memcpy returns the first argument which cannot be NULLPTR since it is a local variable in this module  */
            (void)ptr; /* touch the result to "use" it */

            dsb_handler.g_Transfer_Done = false;

            /* Initialize DSB */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
            IPC->IPC_DATA_STREAM_2P4 = IPC_IPC_DATA_STREAM_2P4_RSTB_MASK | IPC_IPC_DATA_STREAM_2P4_CC(1U);
#elif  defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
            MRCC->MRCC_DATA_STREAM_2P4 = MRCC_MRCC_DATA_STREAM_2P4_RSTB_MASK | MRCC_MRCC_DATA_STREAM_2P4_CC(1U);
#else
            SIM->SCGC5 |= SIM_SCGC5_DSB_MASK;
#endif
            /* Put DSB into reset to ensure clean state */
            DSB->CSR |= DSB_CSR_SFTRST_MASK;
            /* Release DSB from reset */
            DSB->CSR &= ~(DSB_CSR_SFTRST_MASK);

            /* Initialize the DADDR with the destination buffer location in system memory (done in dma_start_capture()).
             */
            destination_address =
                (uint32_t *)dsb_configuration->result_buffer; /* Store addr for multiple start capture calls */

            /* Set the desired total transfer count, TCNT, with the total packet length (in 32-bit words). */
            buffer_size = dsb_configuration->buffer_sz; /* Store size for multiple start capture calls */

            /* Set the watermark register, WMRK, with the transfer threshold level. */
            DSB->WMC = DSB_WMC_WMRK(dsb_configuration->watermark);
            /* Set the DMA_EN, INT_EN, and ERR_EN for interrupts on error or packet transfer completion. bit. */
            DSB->CSR |= DSB_CSR_DMA_EN_MASK | DSB_CSR_INT_EN_MASK | DSB_CSR_ERR_EN_MASK;

#if defined(RADIO_IS_GEN_3P5)
            /* Turns on clocking to DMA/DBG blocks */
            XCVR_RX_DIG->RX_DIG_CTRL |= XCVR_RX_DIG_RX_DIG_CTRL_RX_DMA_DTEST_EN_MASK;
#endif
            /* Enable IRQ */
            NVIC_EnableIRQ(DMA_IRQ);

            /* Some external code must perform the RX warmup request */
        }
    }

    return status;
}

/* Uses IRQ */
dmaStatus_t dma_wait_for_complete(void)
{
    dmaStatus_t status = DMA_SUCCESS;

    /* Wait for DSB transfer finish */
    while (dsb_handler.g_Transfer_Done != true)
    {
    }

    /* Release DMA module */
    dma_release();

    return status;
}

/* Uses IRQ */
dmaStatus_t dma_poll_capture_status(void)
{
    dmaStatus_t status = DMA_CAPTURE_NOT_COMPLETE;

    /* polling loop to wait for DMA capture status */
    if (dsb_handler.g_Transfer_Done == true)
    {
        status = DMA_SUCCESS;
    }

    return status;
}

/* Does not use IRQ, the IRQ must be disabled */
dmaStatus_t dma_wait_for_done_bits(uint8_t mask)
{
    dmaStatus_t status = DMA_SUCCESS;

    if ((mask & (DSB_INT_DONE_MASK | DSB_INT_DBE_MASK | DSB_INT_UNDR_MASK | DSB_INT_OVRF_MASK | DSB_INT_DRDY_MASK)) ==
        0U) /* if no interrupt bits are selected for checking it will cause an infinite loop */
    {
        status = DMA_FAIL_NULL_POINTER;
    }
    else
    {
        /* Wait for DSB INT register to show done or error, based on mask input  */
        while ((DSB->INT & mask) == 0U)
        {
        }

        /* Return a NOT_COMPLETE status if an error bit is asserted */
        if ((DSB->INT & (DSB_INT_DBE_MASK | DSB_INT_UNDR_MASK | DSB_INT_OVRF_MASK)) != 0U)
        {
            status = DMA_CAPTURE_NOT_COMPLETE;
        }
    }

    return status;
}

void dma_set_irq_control(bool irq_enable)
{
    if (irq_enable)
    {
        /* Enable interrupts at DSB and NVIC */
        NVIC_EnableIRQ(DMA_IRQ);
        DSB->CSR |= DSB_CSR_INT_EN_MASK | DSB_CSR_ERR_EN_MASK;
    }
    else
    {
        /* Disable interrupts at DSB and NVIC */
        NVIC_DisableIRQ(DMA_IRQ);
        DSB->CSR &= ~(DSB_CSR_INT_EN_MASK | DSB_CSR_ERR_EN_MASK);
    }
}

void dma_abort(void)
{
    /* Reset DSB */
    DSB->CSR |= DSB_CSR_SFTRST_MASK;
}

void dma_release(void)
{
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
    bool dsb_clk_en = ((IPC->IPC_DATA_STREAM_2P4 & IPC_IPC_DATA_STREAM_2P4_CC_MASK) > 0U);
#elif defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    bool dsb_clk_en = ((MRCC->MRCC_DATA_STREAM_2P4 & MRCC_MRCC_DATA_STREAM_2P4_CC_MASK) > 0U);
#else
    bool dsb_clk_en = ((SIM->SCGC5 & SIM_SCGC5_DSB_MASK) == SIM_SCGC5_DSB_MASK);
#endif

    /* Restart flag */
    dsb_handler.g_Transfer_Done = false;
    destination_address         = NULLPTR;
    buffer_size                 = 0U;

    /* Prevent using DSB before initialization */
    if (dsb_clk_en)
    {
        /* Reset DSB */
        DSB->CSR |= DSB_CSR_SFTRST_MASK;
        while ((DSB->CSR & DSB_CSR_SFTRST_MASK) != 0U) /* Wait for self-clearing soft reset to clear */
        {
        }

        /* Disable DMA */
        XCVR_MISC->DMA_CTRL &= ~(XCVR_MISC_DMA_CTRL_DMA_PAGE_MASK | XCVR_MISC_DMA_CTRL_DMA_EN_MASK);

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
        /* Turns off routing DMA/DBG signals */
        XCVR_RX_DIG->DFT_CTRL &= ~XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL_MASK;
#else
        /* Turns off clocking to DMA/DBG blocks */
        XCVR_RX_DIG->RX_DIG_CTRL &= ~XCVR_RX_DIG_RX_DIG_CTRL_RX_DMA_DTEST_EN_MASK;
#endif
    }
}

dmaStatus_t dma_start_capture(const dma_capture_config_t *dma_configuration)
{
    dmaStatus_t status = DMA_SUCCESS;

    /* Verify input parameters in configuration structure and static storage */
    if ((dma_configuration == NULLPTR) || (destination_address == NULLPTR))
    {
        status = DMA_FAIL_NULL_POINTER;
    }
    else
    {
        /* Some external code must perform the RX warmup request after the dma_init() call for SW triggered DMA. */
        /* if hardware triggered DMA is being used then RX warmup should typically be AFTER dma_start_capture() is
         * completed. */

        /* check for invalid DMA page */
        if ((dma_configuration->dma_page == DMA_PAGE_IDLE) || (dma_configuration->dma_page >= DMA_PAGE_MAX))
        {
            status = DMA_FAIL_PAGE_ERROR;
        }

        /* check for invalid start trigger */
        if (
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN < 450)
            (dma_configuration->start_trig == START_DMA_ON_RESERVED1) ||
            (dma_configuration->start_trig == START_DMA_ON_RESERVED2) ||
#endif /* !defined(RADIO_IS_GEN_4P5) */
            (dma_configuration->start_trig >= INVALID_DMA_START_TRIG))
        {
            status = DMA_INVALID_TRIG_SETTING;
        }

        /* check for invalid start delay (too long for the register) */
        if (dma_configuration->start_delay >= DMA_START_DELAY_MAX)
        {
            status = DMA_INVALID_START_DELAY;
        }

        /* check for invalid setting for capture edge */
        if (dma_configuration->capture_edge >= DMA_CAPTURE_EDGE_MAX)
        {
            status = DMA_INVALID_EDGE_SETTING;
        }

        /* check for invalid setting for decimation rate */
        if (dma_configuration->decimation >= DMA_DECIMATE_MAX)
        {
            status = DMA_INVALID_DECIMATE_SETTING;
        }

        /* check for invalid setting for output selection */
        if (dma_configuration->out_sel >= DMA_OUT_INVALID_SEL)
        {
            status = DMA_INVALID_OUTPUT_SETTING;
        }

        /* check for 0 buffer size in stored size value */
        if (buffer_size == 0U)
        {
            status = DMA_FAIL_SAMPLE_NUM_LIMIT;
        }

        if (status == DMA_SUCCESS)
        {
            /* DMA_PAGE must be cleared separately from the write that sets DMA_EN or corrupt samples will be captured
             */
            XCVR_MISC->DMA_CTRL &=
                ~(XCVR_MISC_DMA_CTRL_DMA_PAGE_MASK); /*Clear the DMA_PAGE to stop the (still) ongoing radio DMA */

            /* Set the DSB_EN bit to receive the data stream. */
            DSB->CSR |= DSB_CSR_DSB_EN_MASK;

            /* Configure DMA capture settings but not DMA_PAGE */
            XCVR_MISC->DMA_CTRL = XCVR_MISC_DMA_CTRL_DMA_EN(1) |
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
                                   XCVR_MISC_DMA_CTRL_DMA_SIGNAL_VALID_MASK_EN((dma_configuration->enable_dma_valid_mask == true ? 1U :0U)) |
#endif /*defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)  */
                                  XCVR_MISC_DMA_CTRL_DMA_DEC(dma_configuration->decimation) |
                                  XCVR_MISC_DMA_CTRL_DMA_START_DLY(dma_configuration->start_delay) |
                                  XCVR_MISC_DMA_CTRL_DMA_START_TRG(dma_configuration->start_trig) |
                                  XCVR_MISC_DMA_CTRL_DMA_START_EDGE(dma_configuration->capture_edge);

#if defined(RADIO_IS_GEN_3P5)
            /* Turns on routing DMA/DBG signals */
            /* RXDIG DFT offset: 0x1FC */
            *((uint32_t *)(XCVR_RX_DIG_BASE + 0x1FCU)) &= ~XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL_MASK;
            *((uint32_t *)(XCVR_RX_DIG_BASE + 0x1FCU)) |= XCVR_RX_DIG_RXDIG_DFT_IQ_MUX_SEL(dma_configuration->out_sel);

#elif defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 400)

            /* Turns on routing DMA/DBG signals */
            XCVR_RX_DIG->DFT_CTRL &= ~XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL_MASK;
            XCVR_RX_DIG->DFT_CTRL |= XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL(dma_configuration->out_sel);

#endif
            /* Resets DSB destination address, buffer size, and transfer done flag for the case of multiple */
            /* calls to dma_start_capture() without intervening dma_init() */
            dsb_handler.g_Transfer_Done = false;
            DSB->DADDR                  = DSB_DADDR_DADDR((uint32_t)destination_address);
            DSB->XCR                    = DSB_XCR_TCNT(buffer_size);
            DSB->CSR |= DSB_CSR_DMA_EN_MASK;

            /* Configure DMA page */
            XCVR_MISC->DMA_CTRL |= XCVR_MISC_DMA_CTRL_DMA_PAGE(dma_configuration->dma_page);
        }
    }

    /* This routine will re-use any existing initializations and will only update the destination address and write the
     * DMA page to trigger the capture start */
    /* The intent is to allow init to be called once to setup a standard size of capture and then multiple captures can
     * be triggered by calling this routine */
    return status;
}

void iq_to_phase_float(const int16_t *iq_buffer, float32_t *phase_buffer, uint16_t num_points)
{
    /* Make sure the IQ and PHASE buffer pointers are not NULL */
    if ((iq_buffer != NULLPTR) && (phase_buffer != NULLPTR))
    {
        uint16_t i;
        static int16_t i_sample, q_sample;
        /* Point to input and output buffers in memory */
        const int16_t *in_ptr = iq_buffer;    /* IQ are input */
        float32_t *out_ptr    = phase_buffer; /* Floating point phase is ouput */
        static float32_t result;
        for (i = 0; i < num_points; i++)
        {
            /* I and Q samples are stored in alternating halves of a 32 bit word */
            i_sample = *in_ptr;
            in_ptr++;
            q_sample = *in_ptr;
            in_ptr++;
            /* Perform atan using math library atan2 routine */
            result   = (float32_t)atan2((float64_t)q_sample, (float64_t)i_sample);
            *out_ptr = result;
            out_ptr++;
        }
    }
}

void iq_to_phase_fixedpt(const int16_t *iq_buffer, int16_t *phase_buffer, uint16_t num_points)
{
    /* Make sure the IQ and PHASE buffer pointers are not NULL */
    if ((iq_buffer != NULLPTR) && (phase_buffer != NULLPTR))
    {
        uint16_t i;
        static int16_t i_sample, q_sample;
        /* Point to input and output buffers in memory */
        const int16_t *in_ptr = iq_buffer;    /* IQ are input */
        int16_t *out_ptr      = phase_buffer; /* Fixed point phase (stored in a uint16_t) is ouput */
        static int16_t result = 0;
        for (i = 0; i < num_points; i++)
        {
            /* I and Q samples are stored in alternating halves of a 32 bit word */
            i_sample = *in_ptr;
            in_ptr++;
            q_sample = *in_ptr;
            in_ptr++;
            /* Perform atan using custom fixed point approximation routine */
            result   = atan2fp(q_sample, i_sample);
            *out_ptr = result;
            out_ptr++;
        }
    }
}

#if defined(GCOV_DO_COVERAGE)
void dma_test_config_static_state(uint32_t *addr, uint32_t size)
{
    destination_address = addr; /* configure the destination address with no error checks */
    buffer_size         = size; /* no error checks */
}
#endif
