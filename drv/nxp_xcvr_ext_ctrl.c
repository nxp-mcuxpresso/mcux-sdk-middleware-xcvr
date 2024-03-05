/*
 * Copyright 2018-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "nxp_xcvr_ext_ctrl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* These definitions are intended to allow manipulation of the TSM timing registers independent of which */
/* register is actually being manipulated. The TX and RX WU and WD are always in the same positions */
#define COMMON_TX_HI_MASK (0xFFU)
#define COMMON_TX_HI_SHIFT (0U)
#define COMMON_TX_HI(x) (((uint32_t)(((uint32_t)(x)) << COMMON_TX_HI_SHIFT)) & COMMON_TX_HI_MASK)
#define COMMON_TX_LO_MASK (0xFF00U)
#define COMMON_TX_LO_SHIFT (8U)
#define COMMON_TX_LO(x) (((uint32_t)(((uint32_t)(x)) << COMMON_TX_LO_SHIFT)) & COMMON_TX_LO_MASK)
#define COMMON_RX_HI_MASK (0xFF0000U)
#define COMMON_RX_HI_SHIFT (16U)
#define COMMON_RX_HI(x) (((uint32_t)(((uint32_t)(x)) << COMMON_RX_HI_SHIFT)) & COMMON_RX_HI_MASK)
#define COMMON_RX_LO_MASK (0xFF000000U)
#define COMMON_RX_LO_SHIFT (24U)
#define COMMON_RX_LO(x) (((uint32_t)(((uint32_t)(x)) << COMMON_RX_LO_SHIFT)) & COMMON_RX_LO_MASK)

/* These defines support different radio generations allocating functions to different TSM timing registers. */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
#define RF_ACTIVE_TSM_REG TIMING00
#define RF_STATUS_TSM_REG TIMING01
#define RF_PRIORITY_TSM_REG TIMING02
#define TSM_IRQ0_TSM_REG TIMING03
#define TX_SWITCH_TSM_REG TIMING07
#define RX_SWITCH_TSM_REG TIMING08
#else
#define RF_ACTIVE_TSM_REG TIMING00
#define RF_STATUS_TSM_REG TIMING45
#define RF_PRIORITY_TSM_REG TIMING46
#define TSM_IRQ0_TSM_REG TIMING43
#define TX_SWITCH_TSM_REG TIMING49
#define RX_SWITCH_TSM_REG TIMING50
#endif /* defined(RADIO_IS_GEN_4P0) */

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
/* These defines gather all the parameters for BLECTE API trimming */
#define LUT_TBL_LENGTH 32U
#define BLECTE_TX_TRIG_SEL lclTxTriggerBtleCteEn
#define BLECTE_RX_TRIG_SEL lclRxTriggerCtePresent
#define BLECTE_TX_DELAY 0U
#define BLECTE_TX_DELAY_OFF 0U
#define BLECTE_RX_DELAY 0U
#define BLECTE_RX_DELAY_OFF 0U
#define BLECTE_DMA_MASK_PERIOD 0U
#define BLECTE_DMA_MASK_DELAY 0U
#define BLECTE_DMA_MASK_DELAY_OFF 0U
#define BLECTE_LANT_INV 0U
#define BLECTE_LANT_SW_WIGGLE 0U
#define BLECTE_DEF_ANT_PIOS                                                                                       \
    {                                                                                                             \
        0x01U, 0x02U, 0x04U, 0x08U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, \
            0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U, 0x0U              \
    }
#endif
/*******************************************************************************
 * Variables
 ******************************************************************************/
static lclRxTxMode_t lcl_patt_match_mode = gLclRxTxNone;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
/* LCL LANT SW handler structs */
static lant_sw_handler_t lant_sw_handler;
#endif

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450) /* Only applies for Gen 4.5 */
/* Storage for timings for RF_STATUS and RF_PRIORITY */
static uint8_t rf_stat_tx_wu = 0U; /* Used only when priority and status are mixed together. Otherwise, the original
                                      init value is unmodified in the TSM register */
static uint8_t rf_stat_rx_wu = 0U; /* Used only when priority and status are mixed together. Otherwise, the original
                                      init value is unmodified in the TSM register */
static uint8_t rf_pri_tx_wu =
    0U; /* Storage is required to allow the XCVR_COEX_SetPriority() routine to swap between High and Low priority */
static uint8_t rf_pri_rx_wu = 0U; /* Used only when priority and status are mixed together. Sets the falling edge of a
                                     High Priority & RX combination. */
static bool rf_stat_initialized    = false;
static bool rf_pri_initialized     = false;
static bool rf_pri_muxed_on_status = false; /* Indicates RF_PRIORITY should be muxed on RF_STATUS line. */
static uint8_t tx_wu_time;                  /* temporary tx warmup time storage */
static uint8_t rx_wu_time;                  /* temporary tx warmup time storage */

#if defined(SUPPORT_AOA_AOD_IN_XCVR) && (SUPPORT_AOA_AOD_IN_XCVR == 1)
static uint8_t PatternToLUT[32] = BLECTE_DEF_ANT_PIOS; /* table to convert antenna number to PIOs combination*/
#endif /* defined (SUPPORT_AOA_AOD_IN_XCVR) && (SUPPORT_AOA_AOD_IN_XCVR == 1) */
#endif /* defined(RADIO_IS_GEN_4P5)  */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)/* Only applies for Gen 4.5 */
static bool XCVR_COEX_VerifyTimingAdvance(uint8_t tx_advance,
                                          uint8_t rx_advance,
                                          uint8_t *tx_wu_time_ptr,
                                          uint8_t *rx_wu_time_ptr);
#endif /* defined(RADIO_IS_GEN_4P5)  */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
void RF_LANT_SW_IRQHandler(void);
#endif /* defined(RADIO_IS_GEN_4P0) || defined(RADIO_IS_GEN_4P5) */

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Only applies for Gen 3.5 and Gen 4.0, not Gen 4.5 */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN <= 400)
xcvrStatus_t XCVR_CoexistenceInit(void)
{
    /* Initialize for co-existence feature in the XCVR */
#if (gMWS_UseCoexistence_d || TEST_BUILD_COEX)
    uint32_t temp         = 0x00U;
    uint32_t end_of_tx_wu = 0x00U;
    uint32_t end_of_rx_wu = 0x00U;

    // RF_ACTIVE controls the  TX_SWITCH on (PTC2, gpio2_trig_en)
    uint32_t rf_active_timing = 0x00U;

    // RF_PRIORITY controls the RX_SWITCH (on PTC3, gpio3_trig_en)
    uint32_t rf_priority_timing = 0x00U;

    // TSM_TIMING03 is used to control the tsm_irq0_trig signal
    uint32_t tsm_irq0_timing_rx = 0x00;
    uint32_t tsm_irq0_timing_tx = 0x00;
    /* Select GPIO mode for FAD pins */
    temp = XCVR_MISC->FAD_CTRL;
    temp &= ~(XCVR_MISC_FAD_CTRL_FAD_NOT_GPIO_MASK);
    XCVR_MISC->FAD_CTRL = temp;

    /* Read the END_OF_TX_WU and END_OF_RX_WU for XCVR */
    end_of_tx_wu =
        (XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT;
    end_of_rx_wu =
        (XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT;

    /*****************
     *  TX SEQUENCE  *
     *****************/

    if (end_of_tx_wu < gMWS_CoexRfActiveAssertTime_d)
    {
        temp = end_of_tx_wu;
    }
    else
    {
        temp = gMWS_CoexRfActiveAssertTime_d;
    }

    /* Save the TX RF_ACTIVE start time. */
    tsm_irq0_timing_tx = end_of_tx_wu - temp;

    /* Set RF_ACTIVE pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any TX sequence. */
    rf_active_timing = (((uint32_t)(tsm_irq0_timing_tx) << COMMON_TX_HI_SHIFT) & COMMON_TX_HI_MASK);

    /* Set STATUS pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any TX sequence. */
    rf_priority_timing = (((uint32_t)(tsm_irq0_timing_tx) << COMMON_TX_HI_SHIFT) & COMMON_TX_HI_MASK);
    /*****************
     *  RX SEQUENCE  *
     *****************/

    if (end_of_rx_wu < gMWS_CoexRfActiveAssertTime_d)
    {
        temp = end_of_rx_wu;
    }
    else
    {
        temp = gMWS_CoexRfActiveAssertTime_d;
    }

#if (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d)
    /* Set RF_ACTIVE pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any RX sequence. */
    rf_active_timing |= (((uint32_t)(end_of_rx_wu - temp) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK);

    /* Set STATUS pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any RX sequence and clear it
     * gMWS_CoexPrioSignalTime_d uS before RX start. */
    rf_priority_timing |=
        ((((uint32_t)(end_of_rx_wu - temp) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK) |
         (((uint32_t)(end_of_rx_wu - gMWS_CoexPrioSignalTime_d) << COMMON_RX_LO_SHIFT) & COMMON_RX_LO_MASK));

    temp = XCVR_TSM->RF_ACTIVE_TSM_REG;
    temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK);
    temp |= rf_active_timing;
    XCVR_TSM->RF_ACTIVE_TSM_REG = temp;

    temp = XCVR_TSM->RF_STATUS_TSM_REG;
    temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK | COMMON_RX_LO_MASK);
    temp |= rf_priority_timing;
    XCVR_TSM->RF_STATUS_TSM_REG = temp;
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d) */

#if (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d)
    /* Set RF_ACTIVE pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any RX sequence. */
    rf_active_timing |= (((uint32_t)(end_of_rx_wu - temp) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK);

    /* Set PRIORITY pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any RX sequence and clear it
     * gMWS_CoexPrioSignalTime_d uS before RX start. */
    rf_priority_timing |= (((uint32_t)(end_of_rx_wu - temp) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK);

    /* RF_ACTIVE */
    temp = XCVR_TSM->RF_ACTIVE_TSM_REG;
    temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK);
    temp |= rf_active_timing;
    XCVR_TSM->RF_ACTIVE_TSM_REG = temp;

    /* RF_PRIORITY */
    temp = XCVR_TSM->RF_PRIORITY_TSM_REG;
    temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK);
    temp |= rf_priority_timing;
    XCVR_TSM->RF_PRIORITY_TSM_REG = temp;
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d) */

#if !(defined(CPU_KW45B41Z82AFPA_NBU) || defined(CPU_KW45B41Z82AFTA_NBU) || defined(CPU_KW45B41Z83AFPA_NBU) || \
      defined(CPU_KW45B41Z83AFTA_NBU)) /* NBU core doesn't control GPIOs */
    /* Overwrite pins settings */
    GPIOC->PDDR |= 0x0CU;
    PORTC->PCR[3] = (PORTC->PCR[3] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(2U);
    PORTC->PCR[2] = (PORTC->PCR[2] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(2U);
#endif /* !defined(RADIO_IS_GEN_3P5) */

    /* It should assert 2 ticks prior to END_OF_RX_WU and de-assert 1 tick later at 1 tick prior to END_OF_TX_WU */
    tsm_irq0_timing_tx = end_of_tx_wu - 2U;
    /* It should assert 2 ticks prior to END_OF_RX_WU and de-assert 1 tick later at 1 tick prior to END_OF_RX_WU */
    tsm_irq0_timing_rx = end_of_rx_wu - 2U;

    XCVR_TSM->TSM_IRQ0_TSM_REG = ((((uint32_t)(tsm_irq0_timing_tx) << COMMON_TX_HI_SHIFT) & COMMON_TX_HI_MASK) |
                                  (((uint32_t)(tsm_irq0_timing_tx + 1U) << COMMON_TX_LO_SHIFT) & COMMON_TX_LO_MASK) |
                                  (((uint32_t)(tsm_irq0_timing_rx) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK) |
                                  (((uint32_t)(tsm_irq0_timing_rx + 1U) << COMMON_RX_LO_SHIFT) & COMMON_RX_LO_MASK));

#if defined(RADIO_IS_GEN_3P5)
    BTLE_RF->MISC_CTRL = 0x02;
#else
    // TODO: TSM interrupt enable setting for Gen 4.0 and 4.5?
#endif /* defined(RADIO_IS_GEN_3P5) */

    /* Enable TSM_IRQ0 interrupt */
    XCVR_TSM->CTRL |= XCVR_TSM_CTRL_TSM_IRQ0_EN_MASK;

    /* Save the updated registers values. */
    XCVR_CoexistenceSaveRestoreTimings(1U);

#endif /* gMWS_UseCoexistence_d */

    return gXcvrSuccess_c;
}

xcvrStatus_t XCVR_CoexistenceSetPriority(XCVR_COEX_PRIORITY_T rxPriority, XCVR_COEX_PRIORITY_T txPriority)
{
    /* Setup the priority for co-existence feature in the XCVR */
#if (gMWS_UseCoexistence_d || TEST_BUILD_COEX)
    uint32_t temp         = 0x00U;
    uint32_t end_of_tx_wu = 0x00U;
    uint32_t end_of_rx_wu = 0x00U;

    uint32_t rf_priority_timing = 0x00U;

    /* Read the END_OF_TX_WU and END_OF_RX_WU for XCVR */
    end_of_tx_wu =
        (XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT;
    end_of_rx_wu =
        (XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT;

    /*****************
     *      RX       *
     *****************/

    if (XCVR_COEX_HIGH_PRIO == rxPriority)
    {
        if (end_of_rx_wu < gMWS_CoexRfActiveAssertTime_d)
        {
            temp = end_of_rx_wu;
        }
        else
        {
            temp = gMWS_CoexRfActiveAssertTime_d;
        }

#if (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d)
        /* Set STATUS pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any RX sequence and clear it
         * gMWS_CoexPrioSignalTime_d uS before RX start for high priority RX. */
        rf_priority_timing =
            ((((uint32_t)(end_of_rx_wu - temp) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK) |
             (((uint32_t)(end_of_rx_wu - gMWS_CoexPrioSignalTime_d) << COMMON_RX_LO_SHIFT) & COMMON_RX_LO_MASK));
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d) */

#if (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d)
        /* Set STATUS pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any RX sequence */
        rf_priority_timing = (((uint32_t)(end_of_rx_wu - temp) << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK);
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d) */
    }
    else
    {
        /* Low priority RX */
        rf_priority_timing = (((0xFFUL << COMMON_RX_HI_SHIFT) & COMMON_RX_HI_MASK) |
                              ((0xFFUL << COMMON_RX_LO_SHIFT) & COMMON_RX_LO_MASK));
    }

    /*****************
     *      TX       *
     *****************/
    if (XCVR_COEX_HIGH_PRIO == txPriority)
    {
        if (end_of_tx_wu < gMWS_CoexRfActiveAssertTime_d)
        {
            temp = end_of_tx_wu;
        }
        else
        {
            temp = gMWS_CoexRfActiveAssertTime_d;
        }

        /* Set STATUS pin HIGH gMWS_CoexRfActiveAssertTime_d uS prior to any TX sequence for HIGH priority TX. */
        rf_priority_timing |= (((uint32_t)(end_of_tx_wu - temp) << COMMON_TX_HI_SHIFT) & COMMON_TX_HI_MASK);
    }
    else
    {
#if (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d)
        /* Set STATUS pin HIGH at END_OF_TX_WU prior to any TX sequence for LOW priority TX. */
        rf_priority_timing |= (((uint32_t)(end_of_tx_wu) << COMMON_TX_HI_SHIFT) & COMMON_TX_HI_MASK);
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d) */
#if (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d)
        /* Set STATUS pin LOW at END_OF_TX_WU prior to any TX sequence for LOW priority TX. */
        rf_priority_timing =
            (((0xFFU << COMMON_TX_HI_SHIFT) & COMMON_TX_HI_MASK) | ((0xFFU << COMMON_TX_LO_SHIFT) & COMMON_TX_LO_MASK));
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d) */
    }

#if (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d)
    temp = XCVR_TSM->RF_STATUS_TSM_REG;
    temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK | COMMON_RX_LO_MASK);
    temp |= rf_priority_timing;
    XCVR_TSM->RF_STATUS_TSM_REG = temp;
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Status_Prio_d) */
#if (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d)
    temp = XCVR_TSM->RF_STATUS_TSM_REG;
    temp &= ~(COMMON_TX_HI_MASK | COMMON_TX_LO_MASK | COMMON_RX_HI_MASK | COMMON_RX_LO_MASK);
    temp |= rf_priority_timing;
    XCVR_TSM->RF_STATUS_TSM_REG = temp;
#endif /* (gMWS_Coex_Model_d == gMWS_Coex_Prio_Only_d) */

    /* Save the updated registers values. */
    XCVR_CoexistenceSaveRestoreTimings(1U);
#endif /* gMWS_UseCoexistence_d */

    return gXcvrSuccess_c;
}

void XCVR_CoexistenceSaveRestoreTimings(uint8_t saveTimings)
{
    /* Save and restore XCVR timing registers corrupted by the co-existence feature in the XCVR */
#if (gMWS_UseCoexistence_d || TEST_BUILD_COEX)
    static uint32_t tsm_ovrd0_saved = 0x00;
    static uint32_t tsm_ovrd1_saved = 0x00;
    static uint32_t tsm_ovrd2_saved = 0x00;
    static uint32_t tsm_ovrd3_saved = 0x00;

    static uint32_t rf_active_timing_saved   = 0x00;
    static uint32_t rf_priority_timing_saved = 0x00;

    if (saveTimings == 0U)
    {
        /* Restore registers values. */
        XCVR_TSM->OVRD0 = tsm_ovrd0_saved;
        XCVR_TSM->OVRD1 = tsm_ovrd1_saved;
        XCVR_TSM->OVRD2 = tsm_ovrd2_saved;
        XCVR_TSM->OVRD3 = tsm_ovrd3_saved;

        XCVR_TSM->RF_ACTIVE_TSM_REG   = rf_active_timing_saved;
        XCVR_TSM->RF_PRIORITY_TSM_REG = rf_priority_timing_saved;
    }
    else
    {
        /* Save registers values. */
        tsm_ovrd0_saved          = XCVR_TSM->OVRD0;
        tsm_ovrd1_saved          = XCVR_TSM->OVRD1;
        tsm_ovrd2_saved          = XCVR_TSM->OVRD2;
        tsm_ovrd3_saved          = XCVR_TSM->OVRD3;
        rf_active_timing_saved   = XCVR_TSM->RF_ACTIVE_TSM_REG;
        rf_priority_timing_saved = XCVR_TSM->RF_PRIORITY_TSM_REG;
    }

#endif /* gMWS_UseCoexistence_d */
}

#else /* Gen 4.5 Coexistence interface */

/*!
 * @brief Function to verify that the tming advance values do not exceed the end of warmup values and calculate the new
 warmup times.
 *
 * This function verifies that the input timing advance values do no exceed the end of warmup values AND then also
 calculates the resulting warmup times.
 *
 * @param[in] tx_advance the timing advance value for TX WU to compare to end of tx wu.
 * @param[in] rx_advance the timing advance value for RX WU to compare to end of rx wu.
 * @param[out] tx_wu_time_ptr pointer to where to store the calculated warmup time for the TX signal when the timing
 advance is applied. Invalid if the function return is false.
 * @param[out] rx_wu_time_ptr pointer to where to store the calculated warmup time for the RX signal when the timing
 advance is applied. Invalid if the function return is false.
 *
 @return True if the timing advances do not exceed the end of sequence values. False if either exceeds the end of
 sequence value.
 *
 */
static bool XCVR_COEX_VerifyTimingAdvance(uint8_t tx_advance,
                                          uint8_t rx_advance,
                                          uint8_t *tx_wu_time_ptr,
                                          uint8_t *rx_wu_time_ptr)
{
    uint8_t end_of_tx_wu = (uint8_t)((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_MASK) >>
                                     XCVR_TSM_END_OF_SEQ_END_OF_TX_WU_SHIFT);
    uint8_t end_of_rx_wu = (uint8_t)((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >>
                                     XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT);
    /* Return true if both TX and RX timings are <= their respective end_of_seq values */
    bool retval = ((tx_advance <= end_of_tx_wu) && (rx_advance <= end_of_rx_wu));
    if (retval)
    {
        /* Both advances are valid so calculate new signal warmup times */
        *tx_wu_time_ptr = end_of_tx_wu - tx_advance;
        *rx_wu_time_ptr = end_of_rx_wu - rx_advance;
    }
    else
    {
        /* At least one timing advance is invalid so return zeros for new warmup times. */
        *tx_wu_time_ptr = 0U;
        *rx_wu_time_ptr = 0U;
    }

    return retval;
}

xcvrCoexStatus_t XCVR_COEX_SelectController(bool tsm_controls_coex,
                                            coexRfactSrc_t rf_active_src,
                                            const coexRfSignalInvert_t *config_ptr)
{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    if (rf_active_src >= coexRfactInvalid)
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        uint32_t temp = RADIO_CTRL->COEX_CTRL;
        /* TSM/LL selection is in RADIO_CONTROL module */
        if (tsm_controls_coex)
        {
            temp |= RADIO_CTRL_COEX_CTRL_COEX_SEL_MASK;
        }
        else
        {
            temp &= ~(RADIO_CTRL_COEX_CTRL_COEX_SEL_MASK);
        }
        /* Signal inversion controls */
        if (config_ptr != NULLPTR) /* NULLPTR is an allowed value and indicates no changes from current state of signal
                                      inversion (possibly the reset state) */
        {
            if (config_ptr->rfna_invert)
            {
                temp |= RADIO_CTRL_COEX_CTRL_RF_NALLOWED_INV_MASK;
            }
            else
            {
                temp &= ~(RADIO_CTRL_COEX_CTRL_RF_NALLOWED_INV_MASK);
            }
            /* Control signal inversion selection for RF_ACTIVE */
            if (config_ptr->rfact_invert)
            {
                temp |= RADIO_CTRL_COEX_CTRL_RF_ACTIVE_INV_MASK;
            }
            else
            {
                temp &= ~(RADIO_CTRL_COEX_CTRL_RF_ACTIVE_INV_MASK);
            }
            /* Control signal inversion selection for RF_STATUS */
            if (config_ptr->rfstat_invert)
            {
                temp |= RADIO_CTRL_COEX_CTRL_RF_STATUS_INV_MASK;
            }
            else
            {
                temp &= ~(RADIO_CTRL_COEX_CTRL_RF_STATUS_INV_MASK);
            }
            /* Control signal inversion selection for RF_PRIORITY */
            /* Set lower bit inversion state */
            if (config_ptr->rfpri_invert[0])
            {
                temp |=
                    RADIO_CTRL_COEX_CTRL_RF_PRIORITY_INV(1U); /* Creates partial mask to modify only the lower bit */
            }
            else
            {
                temp &=
                    ~(RADIO_CTRL_COEX_CTRL_RF_PRIORITY_INV(1U)); /* Creates partial mask to modify only the lower bit */
            }
            /* Set upper bit inversion state */
            if (config_ptr->rfpri_invert[1])
            {
                temp |=
                    RADIO_CTRL_COEX_CTRL_RF_PRIORITY_INV(2U); /* Creates partial mask to modify only the upper bit */
            }
            else
            {
                temp &=
                    ~(RADIO_CTRL_COEX_CTRL_RF_PRIORITY_INV(2U)); /* Creates partial mask to modify only the upper bit */
            }
        }
        RADIO_CTRL->COEX_CTRL = temp;

        /* RF_ACTIVE source selection is in RFMC */
        RFMC->RF2P4GHZ_COEXT &= ~(RFMC_RF2P4GHZ_COEXT_RFACT_SRC_MASK);
        RFMC->RF2P4GHZ_COEXT |= RFMC_RF2P4GHZ_COEXT_RFACT_SRC((uint8_t)rf_active_src);
    }

    return coex_status;
}

xcvrCoexStatus_t XCVR_COEX_RfNotAllowedInit(const coexRfNotAllowedConfig_t *config_ptr)
{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    uint32_t temp                = 0U;
    if (config_ptr == NULLPTR)
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        if (config_ptr->rfna_pin_enable >= coexRfNotAllowPinInvalid) /* Fail init if selected pin is invalid */
        {
            coex_status = gXcvrCoexStatusInvalidArgs;
        }
        else
        {
            /* Configure input pin */
            switch (config_ptr->rfna_pin_enable)
            {
                case coexRfNotAllowPinPta16:
                case coexRfNotAllowPinPta17:
                case coexRfNotAllowPinPta22:
                    temp = RFMC_RF2P4GHZ_COEXT_PORTA_PWR_MASK;
                    break;
                default:
                    temp = 0; /* This code only *sets* the PORTA_PWR bit to keep PORTA powered in deep power down. It
                                 will never clear the bit. */
                    break;
            }
            RFMC->RF2P4GHZ_COEXT &= ~(RFMC_RF2P4GHZ_COEXT_RFNA_IBE_MASK);
            RFMC->RF2P4GHZ_COEXT |=
                (temp | RFMC_RF2P4GHZ_COEXT_RFNA_IBE(
                            (uint8_t)(config_ptr->rfna_pin_enable))); /* Input buffer enable & PORTA power mask */

            /* Setup which link layers get the RF_NOT_ALLOWED signal and whether the signal should be inverted */
            temp = RADIO_CTRL->COEX_CTRL;
            temp &= ~(RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_EN_MASK);
            temp |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_EN(
                config_ptr->link_layer_rfna_select); /* Enable the selected pin */
            RADIO_CTRL->COEX_CTRL = temp;
        }
    }
    return coex_status;
}

void XCVR_COEX_RfNotAllowedOvrd(bool override_en, bool override_val)
{
    uint32_t temp = RADIO_CTRL->COEX_CTRL;
    if (override_en)
    {
        /* Assert the override enable bit and update the value bit */
        temp &=
            ~(RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_OVRD_MASK); /* Clear the value bit to setup for OR function next */
        if (override_val)
        {
            temp |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_OVRD_MASK;
        }
        temp |= RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_OVRD_EN_MASK;
    }
    else
    {
        /* Clear the override enable bit */
        temp &= ~(RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_OVRD_EN_MASK);
    }
    RADIO_CTRL->COEX_CTRL = temp; /* update the register in one write to avoid glitches */
}

bool XCVR_COEX_GetRfNotAllowedStat(bool read_latched_status)
{
    bool return_value;
    if (read_latched_status)
    {
        return_value = ((RADIO_CTRL->COEX_CTRL & RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED_MASK) ==
                        RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED_MASK); /* Latched version of the status bit */
        RADIO_CTRL->COEX_CTRL |=
            RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_ASSERTED_MASK; /* Clear the W1C latched bit that was just read */
    }
    else
    {
        return_value = ((RADIO_CTRL->COEX_CTRL & RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_MASK) ==
                        RADIO_CTRL_COEX_CTRL_RF_NOT_ALLOWED_MASK); /* Raw version of the status bit */
    }

    return return_value;
}

xcvrCoexStatus_t XCVR_COEX_RfActiveTsmInit(const coexRfActiveTsmConfig_t *config_ptr)
{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    if (config_ptr == NULLPTR)
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        /* Throw an error if the RF_ACTIVE source is not set to TSM */
        if ((RFMC->RF2P4GHZ_COEXT & RFMC_RF2P4GHZ_COEXT_RFACT_SRC_MASK) !=
            (0x01UL << RFMC_RF2P4GHZ_COEXT_RFACT_SRC_SHIFT))
        {
            coex_status = gXcvrCoexStatusInvalidArgs;
        }

        /* Calculate rx and tx timing advance values and check that the results are legal */
        if (!XCVR_COEX_VerifyTimingAdvance(config_ptr->rf_act_tx_advance, config_ptr->rf_act_rx_advance, &tx_wu_time,
                                           &rx_wu_time))
        {
            coex_status = gXcvrCoexStatusInvalidTime;
        }

        if (coex_status == gXcvrCoexStatusSuccess)
        {
            /* RFACT_IDIS bit must be set in order for RFACT_FLAG to assert (for RF ACTIVE interrupt or polling) */
            RFMC->RF2P4GHZ_COEXT |= RFMC_RF2P4GHZ_COEXT_RFACT_IDIS_MASK;

            /* Setup RF_ACTIVE extend */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
            XCVR_TSM->CTRL &= ~(XCVR_TSM_CTRL_TSM_SPARE1_EXTEND_MASK);
            XCVR_TSM->CTRL |= XCVR_TSM_CTRL_TSM_SPARE1_EXTEND(config_ptr->rf_act_extend);
#elif defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN > 450)
            XCVR_TSM->CTRL &= ~(XCVR_TSM_CTRL_RF_ACTIVE_EXTEND_MASK);
            XCVR_TSM->CTRL |= XCVR_TSM_CTRL_RF_ACTIVE_EXTEND(config_ptr->rf_act_extend);
#endif  /* defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) */
            /* Setup the TSM register for RF_ACTIVE */
            uint32_t temp =
                XCVR_TSM->END_OF_SEQ; /* Start from the END_OF_SEQ register since warmdowns (LO) are from there */
            temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK); /* Zero out the warmup (HI) times */
            temp |=
                COMMON_TX_HI(tx_wu_time) |
                COMMON_RX_HI(rx_wu_time); /* Insert the warmup times calculated during timing advance verification */
            XCVR_TSM->RF_ACTIVE_TSM_REG = temp;
        }
    }

    return coex_status;
}

xcvrCoexStatus_t XCVR_COEX_RfActiveRfmcInit(const coexRfActiveRfmcConfig_t *config_ptr)
{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    if (config_ptr == NULLPTR)
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        if (config_ptr->wakeup_delay > 63U) /* Invalid wakeup delay (too long) */
        {
            coex_status = gXcvrCoexStatusInvalidTime;
        }

        /* Throw an error if the RF_ACTIVE source is not set to RFMC */
        if ((RFMC->RF2P4GHZ_COEXT & RFMC_RF2P4GHZ_COEXT_RFACT_SRC_MASK) !=
            (0x0UL << RFMC_RF2P4GHZ_COEXT_RFACT_SRC_SHIFT))
        {
            coex_status = gXcvrCoexStatusInvalidArgs;
        }

        /* Update RF_ACTIVE settings for RFMC control */
        if (coex_status == gXcvrCoexStatusSuccess)
        {
            uint32_t temp = RFMC->RF2P4GHZ_COEXT;
            temp &= ~(RFMC_RF2P4GHZ_COEXT_RFACT_IDIS_MASK | RFMC_RF2P4GHZ_COEXT_RFACT_WKUP_DLY_MASK);
            temp |= RFMC_RF2P4GHZ_COEXT_RFACT_WKUP_DLY(config_ptr->wakeup_delay); /* Set the wakeup delay */
            /* Control RF_ACTIVE behavior when TSM is in idle state */
            if (config_ptr->deassert_when_tsm_idle)
            {
                temp |= RFMC_RF2P4GHZ_COEXT_RFACT_IDIS_MASK;
            }
            else
            {
                temp &= ~(RFMC_RF2P4GHZ_COEXT_RFACT_IDIS_MASK);
            }
            RFMC->RF2P4GHZ_COEXT = temp;
        }
    }

    return coex_status;
}

void XCVR_COEX_RfActiveOvrd(bool override_en, bool override_val)
{
    if ((RFMC->RF2P4GHZ_COEXT & RFMC_RF2P4GHZ_COEXT_RFACT_SRC_MASK) == (0x1UL << RFMC_RF2P4GHZ_COEXT_RFACT_SRC_SHIFT))
    {
        /* TSM controls RF_ACTIVE so must override there */
        uint32_t temp = XCVR_TSM->OVRD0;
        if (override_en)
        {
            /* Assert the override enable bit and update the value bit */
            temp &= ~(XCVR_TSM_OVRD0_TSM_RF_ACTIVE_OVRD_MASK); /* Clear the value bit to setup for OR function next */
            if (override_val)
            {
                temp |= XCVR_TSM_OVRD0_TSM_RF_ACTIVE_OVRD_MASK;
            }
            temp |= XCVR_TSM_OVRD0_TSM_RF_ACTIVE_OVRD_EN_MASK;
        }
        else
        {
            /* Clear the override enable bit */
            temp &= ~(XCVR_TSM_OVRD0_TSM_RF_ACTIVE_OVRD_EN_MASK);
        }
        XCVR_TSM->OVRD0 = temp; /* update the register in one write to avoid glitches */
    }
    else
    {
        /* RFMC controls RF_ACTIVE so override from the RFMC */
        if (override_en && override_val)
        {
            RFMC->RF2P4GHZ_COEXT |= RFMC_RF2P4GHZ_COEXT_RFACT_EN_MASK; /* Set the RF_ACTIVE signal */
        }
        else
        {
            RFMC->RF2P4GHZ_COEXT &= ~(RFMC_RF2P4GHZ_COEXT_RFACT_EN_MASK); /* Clear the RF_ACTIVE signal */
        }
    }
}

xcvrCoexStatus_t XCVR_COEX_RfStatusTsmInit(const coexRfStatusConfig_t *config_ptr)
{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    if (config_ptr == NULLPTR)
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        /* Throw an error if the RF_STATUS source is not set to TSM */
        if ((RADIO_CTRL->COEX_CTRL & RADIO_CTRL_COEX_CTRL_COEX_SEL_MASK) == 0U)
        {
            coex_status = gXcvrCoexStatusInvalidArgs;
        }

        if (!XCVR_COEX_VerifyTimingAdvance(config_ptr->rf_stat_tx_advance, config_ptr->rf_stat_tx_advance, &tx_wu_time,
                                           &rx_wu_time)) /* Intentionally re-using TX warmup time!! This is used in the
                                                            combined status/priority case as a falling edge timing */
        {
            coex_status = gXcvrCoexStatusInvalidTime;
        }

        if (coex_status == gXcvrCoexStatusSuccess)
        {
            /* Update static local storage for the warmup times */
            rf_stat_tx_wu = tx_wu_time;
            rf_stat_rx_wu = rx_wu_time; /* Only used in muxed priority/status case as the falling edge for a high
                                           priority RX case */

            /* Setup the TSM register for RF_STATUS */
            uint32_t temp =
                XCVR_TSM->END_OF_SEQ;     /* Start from the END_OF_SEQ register since warmdowns (LO) are from there */
            temp &= ~(COMMON_TX_HI_MASK); /* Zero out the TX warmup (HI) time. RX never asserts the signal. */
            temp |= COMMON_TX_HI(tx_wu_time) | COMMON_RX_HI(0xFFU) |
                    COMMON_RX_LO(0xFFU); /* Insert the TX warmup time calculated during timing advance verification */
            XCVR_TSM->RF_STATUS_TSM_REG = temp; /* RF_STATUS TSM register */

            rf_stat_initialized = true; /* Set initialized flag for combined status/priority use case */
        }
    }

    return coex_status;
}

xcvrCoexStatus_t XCVR_COEX_RfPriorityTsmInit(const coexRfPriorityConfig_t *config_ptr)

{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    if (config_ptr == NULLPTR)
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        /* Throw an error if the RF_PRIORITY source is not set to TSM */
        if ((RADIO_CTRL->COEX_CTRL & RADIO_CTRL_COEX_CTRL_COEX_SEL_MASK) == 0U)
        {
            coex_status = gXcvrCoexStatusInvalidArgs;
        }

        if (!XCVR_COEX_VerifyTimingAdvance(config_ptr->rf_pri_tx_advance, config_ptr->rf_pri_rx_advance, &tx_wu_time,
                                           &rx_wu_time))
        {
            coex_status = gXcvrCoexStatusInvalidTime;
        }

        /* status must be initialized if using priority muxed on status */
        if (config_ptr->rf_pri_on_rf_stat && !rf_stat_initialized)
        {
            coex_status = gXcvrCoexStatusIncompleteConfig;
        }

        if (coex_status == gXcvrCoexStatusSuccess)
        {
            /* Update static local storage for the warmup times */
            rf_pri_tx_wu = tx_wu_time;
            rf_pri_rx_wu = rx_wu_time;
            /* These are later used by the XCVR_COEX_SetPriority() routine to control HIGH priority indications */

            rf_pri_initialized     = true; /* Set initialized flag for combined status/priority use case */
            rf_pri_muxed_on_status = config_ptr->rf_pri_on_rf_stat; /* Set flag if priority is muxed on status line */
            if (rf_pri_muxed_on_status)
            {
                /* In this case, the programming is all in the RF_STATUS TSM register and is handled in the
                 * XCVR_COEX_SetPriority() routine */
                if ((rf_pri_tx_wu < rf_stat_tx_wu) && (rf_pri_rx_wu < rf_stat_rx_wu))
                {
                    coex_status = XCVR_COEX_SetPriority(
                        XCVR_COEX_LOW_PRIO, XCVR_COEX_LOW_PRIO); /* Default both TX and RX priority to LOW */
                }
                else
                {
                    coex_status = gXcvrCoexStatusInvalidTime; /* Priority and status timing advances are incorrect for
                                                                 muxed use case */
                }
                /* Signal inversion follow the RF_STATUS setting so no configuration of inversion in this case */
            }
            else
            {
                /* Setup the TSM register for RF_PRIORITY */
                uint32_t temp =
                    XCVR_TSM->END_OF_SEQ; /* Start from the END_OF_SEQ register since warmdowns (LO) are from there */
                temp |= COMMON_TX_HI(0xFFU) |
                        COMMON_RX_HI(0xFFU); /* Force the warmup times disabled, which defaults to LOW priority */
                XCVR_TSM->RF_PRIORITY_TSM_REG = temp;
            }
        }
    }

    return coex_status;
}

xcvrCoexStatus_t XCVR_COEX_SetPriority(XCVR_COEX_PRIORITY_T rxPriority, XCVR_COEX_PRIORITY_T txPriority)
{
    xcvrCoexStatus_t coex_status = gXcvrCoexStatusSuccess;
    if ((rxPriority > XCVR_COEX_HIGH_PRIO) || (txPriority > XCVR_COEX_HIGH_PRIO))
    {
        coex_status = gXcvrCoexStatusInvalidArgs;
    }
    else
    {
        /*  priority must be initialized if using priority set */
        if (!rf_pri_initialized)
        {
            coex_status = gXcvrCoexStatusIncompleteConfig;
        }

        /* Both priority and status must be initialized if using priority muxed on status */
        if (rf_pri_muxed_on_status && (!rf_pri_initialized || !rf_stat_initialized))
        {
            coex_status = gXcvrCoexStatusIncompleteConfig;
        }

        if (coex_status == gXcvrCoexStatusSuccess)
        {
            uint32_t temp =
                XCVR_TSM->END_OF_SEQ; /* Start from the END_OF_SEQ register since warmdowns (LO) are from there */
            tx_wu_time = 0xFFU;
            rx_wu_time = 0xFFU;
            uint8_t rx_wd_time =
                (uint8_t)((temp & XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_MASK) >>
                          XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_SHIFT); /* default RX WD time, may be overridden below */
            if (rf_pri_muxed_on_status)
            {
                /* Reprogram RF_STATUS TSM register to include the muxed RF_PRIORITY signal as well */
                if (rxPriority == XCVR_COEX_HIGH_PRIO)
                {
                    /* Warmup on the priority timing advance and warmdown on the status timing advance (which is
                     * normally used for warmup!) */
                    rx_wu_time = rf_pri_rx_wu;
                    rx_wd_time = rf_stat_rx_wu;
                }
                /* when rxPriority is low, defaults from above apply */

                if (txPriority == XCVR_COEX_HIGH_PRIO)
                {
                    tx_wu_time = rf_pri_tx_wu; /* Use the warmup time calculated during RF_PRIORITY initialization */
                }
                else
                {
                    tx_wu_time = rf_stat_tx_wu; /* Use the warmup time calculated during RF_STATUS initialization */
                }

                /* apply these new warmup times to the temp register and then into the RF_STATUS register */
                temp &= ~(
                    COMMON_TX_HI_MASK | COMMON_RX_HI_MASK |
                    COMMON_RX_LO_MASK); /* Zero out the warmup (HI) times and RX WD time, leaving TX to flow through */
                temp |= COMMON_TX_HI(tx_wu_time) | COMMON_RX_HI(rx_wu_time) |
                        COMMON_RX_LO(rx_wd_time); /* Insert both the warmup and warmdown times  (except TX WD) */
                XCVR_TSM->RF_STATUS_TSM_REG = temp;
            }
            else
            {
                /* Calculate  the TSM register settings for RF_PRIORITY */
                if (rxPriority == XCVR_COEX_HIGH_PRIO)
                {
                    rx_wu_time = rf_pri_rx_wu; /* Use the warmup time calculated during RF_PRIORITY initialization */
                }
                if (txPriority == XCVR_COEX_HIGH_PRIO)
                {
                    tx_wu_time = rf_pri_tx_wu; /* Use the warmup time calculated during RF_PRIORITY initialization */
                }

                temp &= ~(COMMON_TX_HI_MASK | COMMON_RX_HI_MASK);            /* Zero out the warmup (HI) times */
                temp |= COMMON_TX_HI(tx_wu_time) | COMMON_RX_HI(rx_wu_time); /* Insert the warmup times  */
                XCVR_TSM->RF_PRIORITY_TSM_REG = temp;
            }
        }
    }

    return coex_status;
}

void XCVR_COEX_RfStatPrioOvrd(bool override_en_stat,
                              bool override_val_stat,
                              bool override_en_pri,
                              bool override_val_pri)
{
    uint32_t temp = XCVR_TSM->OVRD0;
    /* RF_STATUS overrides */
    if (override_en_stat)
    {
        /* Assert the override enable bit and update the value bit */
        temp &= ~(XCVR_TSM_OVRD0_TSM_RF_STATUS_OVRD_MASK); /* Clear the value bit to setup for OR function next */
        if (override_val_stat)
        {
            temp |= XCVR_TSM_OVRD0_TSM_RF_STATUS_OVRD_MASK;
        }
        temp |= XCVR_TSM_OVRD0_TSM_RF_STATUS_OVRD_EN_MASK;
    }
    else
    {
        /* Clear the override enable bit */
        temp &= ~(XCVR_TSM_OVRD0_TSM_RF_STATUS_OVRD_EN_MASK);
    }
    /* RF_PRIORITY overrides */
    if (override_en_pri)
    {
        /* Assert the override enable bit and update the value bit */
        temp &= ~(XCVR_TSM_OVRD0_TSM_RF_PRIORITY_OVRD_MASK); /* Clear the value bit to setup for OR function next */
        if (override_val_pri)
        {
            temp |= XCVR_TSM_OVRD0_TSM_RF_PRIORITY_OVRD_MASK;
        }
        temp |= XCVR_TSM_OVRD0_TSM_RF_PRIORITY_OVRD_EN_MASK;
    }
    else
    {
        /* Clear the override enable bit */
        temp &= ~(XCVR_TSM_OVRD0_TSM_RF_PRIORITY_OVRD_EN_MASK);
    }
    XCVR_TSM->OVRD0 = temp; /* update the register in one write to avoid glitches */
}

void XCVR_COEX_ClearSavedState(void)
{
    /*Clear local state for timings for RF_STATUS and RF_PRIORITY */
    rf_stat_tx_wu = 0U;
    rf_stat_rx_wu = 0U;
    rf_pri_tx_wu  = 0U;
    rf_pri_rx_wu  = 0U;
    /* Set all initialization flags to false */
    rf_stat_initialized    = false;
    rf_pri_initialized     = false;
    rf_pri_muxed_on_status = false;
}

#endif /* defined(RADIO_IS_GEN_3P5)  || defined(RADIO_IS_GEN_4P0) */

/* ******************************************************************* */
/* ******************** PA/FEM control routines *************************** */
/* ******************************************************************* */
xcvrStatus_t XCVR_ExternalFadPaFemInit(xcvr_pa_fem_config_t *pa_fem_settings_ptr)
{
    xcvrStatus_t status = gXcvrSuccess_c;
    uint32_t temp_reg   = 0;
    uint8_t subfield;
    uint8_t wu_time;
    uint8_t wd_time;
    /* Initialize the external PA and FEM control feature */

    /* Perform any input verification checks needed and update status appropriately */
    if (pa_fem_settings_ptr == NULLPTR)
    {
        status = gXcvrInvalidParameters_c;
    }
    else
    {
        /* Check contents of the initialization structure */

        /* when FAD state machine is in use, ANT_A and ANT_B must be under FAD state machine control */
        if ((pa_fem_settings_ptr->use_fad_state_machine == 1U) &&
            ((pa_fem_settings_ptr->ant_a_pad_control == XCVR_FAD_TSM_GPIO) ||
             (pa_fem_settings_ptr->ant_b_pad_control == XCVR_FAD_TSM_GPIO)))
        {
            status = gXcvrInvalidConfiguration_c;
        }

        /* TX and RX warmdown times are restricted to be 0 in the current TSM design */
        if ((pa_fem_settings_ptr->pa_tx_wd != 0U) || (pa_fem_settings_ptr->lna_rx_wd != 0U))
        {
            status = gXcvrInvalidParameters_c;
        }

        /* All of the below parameters must be either 0 or 1 and are treated as booleans so OR them together and check
         * for > 1 to validate them */
        if (((uint32_t)pa_fem_settings_ptr->op_mode | (uint32_t)(pa_fem_settings_ptr->high_z_enable) |
             (uint32_t)pa_fem_settings_ptr->ant_sel_pins_enable |
             (uint32_t)pa_fem_settings_ptr->tx_rx_switch_pins_enable |
             (uint32_t)pa_fem_settings_ptr->use_fad_state_machine | (uint32_t)pa_fem_settings_ptr->ant_a_pad_control |
             (uint32_t)pa_fem_settings_ptr->ant_b_pad_control | (uint32_t)pa_fem_settings_ptr->tx_switch_pad_control |
             (uint32_t)pa_fem_settings_ptr->rx_switch_pad_control |
             (uint32_t)pa_fem_settings_ptr->tx_switch_pol_control |
             (uint32_t)pa_fem_settings_ptr->rx_switch_pol_control) > 1U)
        {
            status = gXcvrInvalidParameters_c;
        }
    }

    /* If input verification checks have passed then perform the PA/FEM initialization */
    if (status == gXcvrSuccess_c)
    {
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
        XCVR_ANALOG->LDO_1 &= ~(XCVR_ANALOG_LDO_1_LDO_ANT_HIZ_MASK);
        XCVR_ANALOG->LDO_1 |=
            XCVR_ANALOG_LDO_1_LDO_ANT_HIZ(pa_fem_settings_ptr->high_z_enable); /* Set High Z if enabled */
#endif

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
        temp_reg =
            XCVR_MISC_FAD_CTRL_FAD_LANT_SEL(1U); /* New bit for Gen 4.5 to select between FAD and LANT_LUT outputs */
#elif defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
        temp_reg = 0U;
#else
        temp_reg = XCVR_MISC_FAD_CTRL_ANTX_HZ(pa_fem_settings_ptr->high_z_enable); /* Set High Z if enabled */
#endif
        temp_reg |= XCVR_MISC_FAD_CTRL_ANTX_CTRLMODE(pa_fem_settings_ptr->op_mode); /* Set control mode */
        subfield =
            (uint8_t)(((uint32_t)pa_fem_settings_ptr->rx_switch_pol_control << 3) |
                      ((uint32_t)pa_fem_settings_ptr->tx_switch_pol_control << 2)); /* Build field value for ANTX_POL */
        temp_reg |= XCVR_MISC_FAD_CTRL_ANTX_POL(subfield);
        subfield = (uint8_t)(
            ((uint32_t)pa_fem_settings_ptr->rx_switch_pad_control << 3) | /* Build field value for FAD_NOT_GPIO */
            ((uint32_t)pa_fem_settings_ptr->tx_switch_pad_control << 2) |
            ((uint32_t)pa_fem_settings_ptr->ant_b_pad_control << 1) |
            ((uint32_t)pa_fem_settings_ptr->ant_a_pad_control << 0));
        temp_reg |= XCVR_MISC_FAD_CTRL_FAD_NOT_GPIO(subfield);
        subfield = (uint8_t)(((uint32_t)pa_fem_settings_ptr->ant_sel_pins_enable << 1) |
                             ((uint32_t)pa_fem_settings_ptr->tx_rx_switch_pins_enable << 0));
        temp_reg |= XCVR_MISC_FAD_CTRL_ANTX_EN(subfield);
        XCVR_MISC->FAD_CTRL = temp_reg;

        /* Program TSM Timing registers for TSM_GPIO_2 (tx switch) and TSM_GPIO_3 (rx switch) */

        /* External PA needs to start before intnernal PA, which starts at TX_DIG_EN so apply advance relative to
         * TX_DIG_EN */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
        // TODO: update this equation for Gen4.x because PA doesn't actually start a TX_DIG_EN but starts after a delay
        // (potentially).
        wu_time = (uint8_t)(
            ((XCVR_TSM->TIMING14 & XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI_MASK) >> XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI_SHIFT) -
            pa_fem_settings_ptr->pa_tx_wu);
#else
        wu_time  = (uint8_t)(
            ((XCVR_TSM->TIMING35 & XCVR_TSM_TIMING35_TX_DIG_EN_TX_HI_MASK) >> XCVR_TSM_TIMING35_TX_DIG_EN_TX_HI_SHIFT) -
            pa_fem_settings_ptr->pa_tx_wu);
#endif
        wd_time = (uint8_t)(
            ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_SHIFT) +
            pa_fem_settings_ptr->pa_tx_wd);
        /* The TX timing advance value must be strictly less than the difference between wd_time and wu_time due to TSM
         * counter restrictions. */
        if (pa_fem_settings_ptr->pa_tx_wd < (wd_time - wu_time))
        {
            XCVR_TSM->TX_SWITCH_TSM_REG =
                B3(0xFFU) | B2(0xFFU) | B1(wd_time) | B0(wu_time); /* TSM_GPIO_2 (controls the tx switch) */
        }
        else
        {
            status = gXcvrInvalidConfiguration_c;
        }
        /* RX timing calculations */
        wu_time = (uint8_t)(
            ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT) -
            pa_fem_settings_ptr->lna_rx_wu);
        wd_time = (uint8_t)(
            ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_SHIFT) +
            pa_fem_settings_ptr->lna_rx_wd);
        /* The RX timing advance value must be strictly less than the difference between wd_time and wu_time due to TSM
         * counter restrictions. */
        if (pa_fem_settings_ptr->lna_rx_wd < (wd_time - wu_time))
        {
            XCVR_TSM->RX_SWITCH_TSM_REG =
                B3(wd_time) | B2(wu_time) | B1(0xFFU) | B0(0xFFU); /* TSM_GPIO_3 (controls the rx switch) */
        }
        else
        {
            status = gXcvrInvalidConfiguration_c;
        }
    }

    return status;
}

xcvrStatus_t XCVR_ExternalFadPaFemDeInit(void)
{
    /* De-iInitialize the external PA and FEM control feature */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)
    XCVR_TSM->TX_SWITCH_TSM_REG = 0xFFFFFFFFUL;
    XCVR_TSM->RX_SWITCH_TSM_REG = 0xFFFFFFFFUL;
#else
    XCVR_TSM->TIMING49 = 0xFFFFFFFFUL;
    XCVR_TSM->TIMING50 = 0xFFFFFFFFUL;
#endif /* defined(RADIO_IS_GEN_4P0) */
    XCVR_MISC->FAD_CTRL = 0x0UL;
    return gXcvrSuccess_c;
}

void XCVR_ExtAntOvrd(XCVR_FAD_OVRD_ANT_A_B_SEL_MODE_T antenna_sel)
{
    /* Force an external antenna selection in the FAD feature */
    uint32_t temp_reg = XCVR_MISC->FAD_CTRL;
    if (antenna_sel == XCVR_FAD_OVRD_SEL_ANT_A)
    {
        temp_reg &= ~XCVR_MISC_FAD_CTRL_ANTX_OVRD_MASK; /* Select Antenna A */
    }
    else
    {
        temp_reg |= XCVR_MISC_FAD_CTRL_ANTX_OVRD_MASK; /* Select Antenna B */
    }
    XCVR_MISC->FAD_CTRL = temp_reg | XCVR_MISC_FAD_CTRL_ANTX_OVRD_EN_MASK;
}

void XCVR_ExtAntRelease(void)
{
    /* Release an external antenna selection in the FAD feature */
    XCVR_MISC->FAD_CTRL &= ~XCVR_MISC_FAD_CTRL_ANTX_OVRD_EN_MASK;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN <= 400)/* Only applies for Gen 3.5 and Gen 4.0, not Gen 4.5 */
xcvrStatus_t XCVR_FadPaFemOnCoexInit(xcvr_pa_fem_config_t *test_settings,
                                     tx_rx_coex_pin_func_t rf_status_func,
                                     tx_rx_coex_pin_func_t rf_priority_func)
{
    uint8_t tx_wu_time;
    uint8_t tx_wd_time;
    uint8_t rx_wu_time;
    uint8_t rx_wd_time;
    uint32_t temp;
    xcvrStatus_t status = gXcvrSuccess_c;

    /* Perform any input verification checks needed and update status appropriately */
    if (test_settings == NULLPTR)
    {
        status = gXcvrInvalidParameters_c;
    }
    else
    {
        /* TX and RX warmdown times are restricted to be 0 in the current TSM design */
        if ((test_settings->pa_tx_wd != 0U) || (test_settings->lna_rx_wd != 0U))
        {
            status = gXcvrInvalidParameters_c;
        }

        /* All of the below parameters must be either 0 or 1 and are treated as booleans so OR them together and check
         * for > 1 to validate them */
        if (((uint32_t)test_settings->op_mode | (uint32_t)(test_settings->high_z_enable) |
             (uint32_t)test_settings->ant_sel_pins_enable | (uint32_t)test_settings->tx_rx_switch_pins_enable |
             (uint32_t)test_settings->use_fad_state_machine | (uint32_t)test_settings->ant_a_pad_control |
             (uint32_t)test_settings->ant_b_pad_control | (uint32_t)test_settings->tx_switch_pad_control |
             (uint32_t)test_settings->rx_switch_pad_control | (uint32_t)test_settings->tx_switch_pol_control |
             (uint32_t)test_settings->rx_switch_pol_control) > 1U)
        {
            status = gXcvrInvalidParameters_c;
        }
    }

    /* Input checks have all passed */
    if (status == gXcvrSuccess_c)
    {
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN== 400)
        RFMC->RF2P4GHZ_COEXT &=
            ~(RFMC_RF2P4GHZ_COEXT_RFPRI_OBE_MASK |
              RFMC_RF2P4GHZ_COEXT_RFSTAT_OBE_MASK); /* Ensure both enable bits are cleared to start out. */
        tx_wu_time = (uint8_t)(
            ((XCVR_TSM->TIMING14 & XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI_MASK) >> XCVR_TSM_TIMING14_TX_DIG_EN_TX_HI_SHIFT) -
            test_settings->pa_tx_wu);
#else
        RSIM->SW_CONFIG &=
            ~(RSIM_SW_CONFIG_IPP_OBE_RF_PRIORITY_MASK |
              RSIM_SW_CONFIG_IPP_OBE_RF_STATUS_MASK); /* Ensure both enable bits are cleared to start out. */
        /* External PA should start up before the internal PA */
        tx_wu_time = (uint8_t)(
            ((XCVR_TSM->TIMING35 & XCVR_TSM_TIMING35_TX_DIG_EN_TX_HI_MASK) >> XCVR_TSM_TIMING35_TX_DIG_EN_TX_HI_SHIFT) -
            test_settings->pa_tx_wu);
#endif /* defined(RADIO_IS_GEN_4P0) */
        tx_wd_time = (uint8_t)(
            ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_TX_WD_SHIFT) +
            test_settings->pa_tx_wd);
        rx_wu_time = (uint8_t)(
            ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WU_SHIFT) -
            test_settings->lna_rx_wu);
        rx_wd_time = (uint8_t)(
            ((XCVR_TSM->END_OF_SEQ & XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_MASK) >> XCVR_TSM_END_OF_SEQ_END_OF_RX_WD_SHIFT) +
            test_settings->lna_rx_wd);
        /* The TX & RX timing advance values must be strictly less than the difference between wd_time and wu_time due
         * to TSM counter restrictions. */
        if ((test_settings->pa_tx_wd < (tx_wd_time - tx_wu_time)) &&
            (test_settings->lna_rx_wd < (rx_wd_time - rx_wu_time)))
        {
            status = gXcvrSuccess_c;
            if (rf_status_func != NO_FUNC)
            {
                switch (rf_status_func)
                {
                    case TX_FUNC:
                        temp = B3(0xFFU) | B2(0xFFU) | B1(tx_wd_time) | B0(tx_wu_time); /*  Setup tx switch */
                        break;
                    case RX_FUNC:
                        temp = B3(rx_wd_time) | B2(rx_wu_time) | B1(0xFFU) | B0(0xFFU); /*  Setup rx switch */
                        break;
                    case BOTH_TX_RX_FUNC:
                        temp = B3(rx_wd_time) | B2(rx_wu_time) | B1(tx_wd_time) |
                               B0(tx_wu_time); /*  Both tx switch and rx switch together */
                        break;
                    default:
                        temp =
                            B3(0xFFU) | B2(0xFFU) | B1(0xFFU) | B0(0xFFU); /* default to no function in case of error */
                        status = gXcvrInvalidConfiguration_c;
                        break;
                }

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
                /* setup RF_STATUS pin for TSM output */
                XCVR_TSM->RF_STATUS_TSM_REG = temp; /* set the timing */
                /* Enable the output drivers for RF_STATUS */
                RFMC->RF2P4GHZ_COEXT |= RFMC_RF2P4GHZ_COEXT_RFSTAT_OBE_MASK;
#else
                /* setup RF_STATUS pin for TSM_SPARE2 output */
                RSIM->DSM_CONTROL &= ~(RSIM_SW_CONFIG_WIFI_COEXIST_2_MASK); /* Clear WIFI_COEXIST_2 bit to use
                                                                               TSM_SPARE2 on RF_STATUS pin */
                XCVR_TSM->TIMING45 = temp;                                  /* set the timing */
                /* Enable the output drivers for RF_STATUS and RF_PRIORITY */
                RSIM->SW_CONFIG |= RSIM_SW_CONFIG_IPP_OBE_RF_STATUS_MASK;
#endif /* defined(RADIO_IS_GEN_4P0) */
            }
            if ((rf_priority_func != NO_FUNC) && (status == gXcvrSuccess_c))
            {
                switch (rf_priority_func)
                {
                    case TX_FUNC:
                        temp = B3(0xFFU) | B2(0xFFU) | B1(tx_wd_time) | B0(tx_wu_time); /*  setup tx switch */
                        break;
                    case RX_FUNC:
                        temp = B3(rx_wd_time) | B2(rx_wu_time) | B1(0xFFU) | B0(0xFFU); /*  setup rx switch */
                        break;
                    case BOTH_TX_RX_FUNC:
                        temp = B3(rx_wd_time) | B2(rx_wu_time) | B1(tx_wd_time) |
                               B0(tx_wu_time); /*  both tx switch and rx switch together */
                        break;
                    default:
                        temp =
                            B3(0xFFU) | B2(0xFFU) | B1(0xFFU) | B0(0xFFU); /* default to no function in case of error */
                        status = gXcvrInvalidConfiguration_c;
                        break;
                }

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
                /* setup RF_PRIORITY pin for TSM */
                XCVR_TSM->RF_PRIORITY_TSM_REG = temp; /* set the timing */
                /* Enable the output drivers for RF_PRIORITY */
                RFMC->RF2P4GHZ_COEXT |= RFMC_RF2P4GHZ_COEXT_RFPRI_OBE_MASK;
#else
                /* setup RF_STATUS pin for TSM_SPARE3 output */
                RSIM->DSM_CONTROL |= RSIM_SW_CONFIG_WIFI_COEXIST_3_MASK; /* Set WIFI_COEXIST_3 bit to use TSM_SPARE3 on
                                                                            RF_PRIORITY pin */
                XCVR_TSM->TIMING46 = temp;                               /* set the timing */
                /* Enable the output drivers for RF_STATUS and RF_PRIORITY */
                RSIM->SW_CONFIG |= RSIM_SW_CONFIG_IPP_OBE_RF_PRIORITY_MASK;
#endif /* defined(RADIO_IS_GEN_4P0) */
            }
        }
        else
        {
            status = gXcvrInvalidConfiguration_c;
        }
    }

    return status;
}

void XCVR_FadPaFemOnCoexDeInit(void)
{
    /* Disable the PA/FEM over coexistence feature */
    XCVR_TSM->RF_STATUS_TSM_REG   = 0xFFFFFFFFUL;
    XCVR_TSM->RF_PRIORITY_TSM_REG = 0xFFFFFFFFUL;

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
    RFMC->RF2P4GHZ_COEXT &=
        ~(RFMC_RF2P4GHZ_COEXT_RFPRI_OBE_MASK |
          RFMC_RF2P4GHZ_COEXT_RFSTAT_OBE_MASK); /* Ensure both enable bits are cleared to start out. */
#else
    RSIM->SW_CONFIG &=
        ~(RSIM_SW_CONFIG_IPP_OBE_RF_PRIORITY_MASK |
          RSIM_SW_CONFIG_IPP_OBE_RF_STATUS_MASK); /* Ensure both enable bits are cleared to prevent usage. */
#endif /* defined(RADIO_IS_GEN_4P0) */
}
#endif /* defined(RADIO_IS_GEN_3P5)  || defined(RADIO_IS_GEN_4P0) */

/* ************************************************************************** */
/* **************** Localization and antenna control routines *************** */
/* ************************************************************************** */

/*! *********************************************************************************
 * \brief  Handles LCL module programming for AoA/AoD antenna switching
 ********************************************************************************** */
static inline bool is_ant_duration_invalid(uint8_t duration)
{
    bool invalid = false;
    if ((duration == 0U) || (duration >= XCVR_LCL_SAMPLES_MAX_LEN)) /* duration of 0 or max length is invalid */
    {
        invalid = true;
    }
    return invalid;
}

static xcvrLclStatus_t XCVR_LCL_ValidateAntennaSwitchSettings(lclRxTxMode_t mode,
                                                              const lclAntennaSwitchingPatternConfig_t *pConfig)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    /* Not checking NULLPTR because this routine is local and the calling routine is already checking NULLPTR inputs */

    if (is_ant_duration_invalid(pConfig->samplesPerInterval)) /* Error check for max length */
    {
        status = gXcvrLclStatusInvalidLength;
    }

    /* Check sample duration (> 31 doesn't fit in the register and 0 doesn't make any sense and may cause unexpected
     * behavior) */
    if (is_ant_duration_invalid(pConfig->lowPeriodDuration[0]) ||
        is_ant_duration_invalid(pConfig->highPeriodDuration[0])) /* Error check for  duration 0 */
    {
        status = gXcvrLclStatusInvalidDuration;
    }

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    if (is_ant_duration_invalid(pConfig->lowPeriodDuration[1]) ||
        is_ant_duration_invalid(pConfig->highPeriodDuration[1]) ||
        is_ant_duration_invalid(pConfig->lowPeriodDuration[2]) ||
        is_ant_duration_invalid(pConfig->highPeriodDuration[2]) ||
        is_ant_duration_invalid(pConfig->lowPeriodDuration[3]) ||
        is_ant_duration_invalid(pConfig->highPeriodDuration[3])) /* Error check for duration 1 */
    {
        status = gXcvrLclStatusInvalidDuration;
    }
#endif /* #if defined(RADIO_IS_GEN_4P5) */
    /* Check trigger configuration */
    if ((pConfig->txTrig >= lclTxTriggerInvalid) || (pConfig->rxTrig >= lclRxTriggerInvalid))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    /* Check mode */
    if ((mode != gLclTxMode) && (mode != gLclRxMode))
    {
        status = gXcvrLclStatusInvalidArgs;
    }

    return status;
}
xcvrLclStatus_t XCVR_LCL_AntennaSwitchInit(lclRxTxMode_t mode, const lclAntennaSwitchingPatternConfig_t *pConfig)
{
    uint32_t temp          = 0U;
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    /* Verify input parameters in configuration structure */
    if (pConfig == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        uint8_t sample_per_interval;
        status = XCVR_LCL_ValidateAntennaSwitchSettings(
            mode, pConfig); /* Use helper function to validation settings to reduce CCM */

        /* End of error checks, if still successful, implement the change */
        if (status == gXcvrLclStatusSuccess)
        {
            sample_per_interval =
                pConfig->samplesPerInterval -
                1U; /* Gen 4.0 & 4.5 registers store the interval value less 1. API value is the interval */

            /* ************* COMMON MODE CONFIGURATION ************* */
            temp = XCVR_MISC->LCL_CFG0;
            temp &= ~(XCVR_MISC_LCL_CFG0_CTE_DUR_MASK | XCVR_MISC_LCL_CFG0_LANT_INV_MASK |
                      XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE_MASK | XCVR_MISC_LCL_CFG0_LANT_BLOCK_RX_MASK |
                      XCVR_MISC_LCL_CFG0_LANT_BLOCK_TX_MASK);
            temp |= XCVR_MISC_LCL_CFG0_CTE_DUR(pConfig->numberOfSwitchesInPattern); /* set pattern duration */
            if (pConfig->lantSwInvert)
            {
                temp |= XCVR_MISC_LCL_CFG0_LANT_INV_MASK; /* Set bit only, it is cleared above */
            }
            if (pConfig->lantSwWiggle)
            {
                temp |= XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE_MASK; /* Set bit only, it is cleared above */
            }
            XCVR_MISC->LCL_CFG0 = temp; /* Update the config register */

            /* ************* RX MODE CONFIGURATION ************* */
            /* Configure switching delay, switching trigger and switching pattern for RX mode*/
            if (mode == gLclRxMode)
            {
                /* Configure trigger delay and offset */
                XCVR_MISC->LCL_RX_CFG0 = XCVR_MISC_LCL_RX_CFG0_RX_DELAY(pConfig->triggerDelay) |
                                         XCVR_MISC_LCL_RX_CFG0_RX_DELAY_OFF(pConfig->triggerOffset);

                /* Configure trigger RX settings */
                XCVR_MISC->LCL_RX_CFG1 = XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL((uint8_t)pConfig->rxTrig) |
                                         XCVR_MISC_LCL_RX_CFG1_RX_SPINT(sample_per_interval) |
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) /* RX_LO_PER1 and RX_HI_PER1 only exist on KW45 */
                                         XCVR_MISC_LCL_RX_CFG1_RX_LO_PER_1(pConfig->lowPeriodDuration[1]) |
                                         XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_1(pConfig->highPeriodDuration[1]) |
#endif /* #if defined(RADIO_IS_GEN_4P5) */
                                         XCVR_MISC_LCL_RX_CFG1_RX_HI_PER(pConfig->highPeriodDuration[0]) |
                                         XCVR_MISC_LCL_RX_CFG1_RX_LO_PER(pConfig->lowPeriodDuration[0]);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) /* LCL_RX_CFG2 only exists on KW45 */
                XCVR_MISC->LCL_RX_CFG2 = XCVR_MISC_LCL_RX_CFG2_RX_LO_PER_2(pConfig->lowPeriodDuration[2]) |
                                         XCVR_MISC_LCL_RX_CFG2_RX_HI_PER_2(pConfig->highPeriodDuration[2]) |
                                         XCVR_MISC_LCL_RX_CFG2_RX_LO_PER_3(pConfig->lowPeriodDuration[3]) |
                                         XCVR_MISC_LCL_RX_CFG2_RX_HI_PER_3(pConfig->highPeriodDuration[3]);
#endif /* #if defined(RADIO_IS_GEN_4P5) */
            }

            /* ************* TX MODE CONFIGURATION ************* */
            /* Configure switching delay, switching trigger and switching pattern for TX mode*/
            if (mode == gLclTxMode)
            {
                /* Configure trigger delay and offset */
                XCVR_MISC->LCL_TX_CFG0 = XCVR_MISC_LCL_TX_CFG0_TX_DELAY(pConfig->triggerDelay) |
                                         XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF(pConfig->triggerOffset);

                /* Configure trigger TX settings */
                XCVR_MISC->LCL_TX_CFG1 = XCVR_MISC_LCL_TX_CFG1_TX_ANT_TRIG_SEL((uint8_t)pConfig->txTrig) |
                                         XCVR_MISC_LCL_TX_CFG1_TX_SPINT(sample_per_interval) |
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) /* TX_LO_PER1 and TX_HI_PER1 only exist on KW45 */
                                         XCVR_MISC_LCL_TX_CFG1_TX_LO_PER_1(pConfig->lowPeriodDuration[1]) |
                                         XCVR_MISC_LCL_TX_CFG1_TX_HI_PER_1(pConfig->highPeriodDuration[1]) |
#endif /* #if defined(RADIO_IS_GEN_4P5) */
                                         XCVR_MISC_LCL_TX_CFG1_TX_HI_PER(pConfig->highPeriodDuration[0]) |
                                         XCVR_MISC_LCL_TX_CFG1_TX_LO_PER(pConfig->lowPeriodDuration[0]);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450) /* LCL_TX_CFG2 only exists on KW45 */
                XCVR_MISC->LCL_TX_CFG2 = XCVR_MISC_LCL_TX_CFG2_TX_LO_PER_2(pConfig->lowPeriodDuration[2]) |
                                         XCVR_MISC_LCL_TX_CFG2_TX_HI_PER_2(pConfig->highPeriodDuration[2]) |
                                         XCVR_MISC_LCL_TX_CFG2_TX_LO_PER_3(pConfig->lowPeriodDuration[3]) |
                                         XCVR_MISC_LCL_TX_CFG2_TX_HI_PER_3(pConfig->highPeriodDuration[3]);
#endif /* #if defined(RADIO_IS_GEN_4P5) */
            }
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
            /* Use only mask capability from RX DIG since it properly supports averaging */
            XCVR_RX_DIG->CTRL1 |= XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK;
#endif /* #if defined(RADIO_IS_GEN_4P5) */
        }
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_AntennaSwitchRead(lclRxTxMode_t mode, lclAntennaSwitchingPatternConfig_t *pConfig)
{
    xcvrLclStatus_t status;
    uint8_t temp = 0U;

    /* Verify input parameters in configuration structure */
    if (pConfig == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Read pattern duration */
        pConfig->numberOfSwitchesInPattern =
            (uint16_t)((XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_CTE_DUR_MASK) >> XCVR_MISC_LCL_CFG0_CTE_DUR_SHIFT);
        /* Read invert configuration */
        pConfig->lantSwInvert = (bool)(XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_LANT_INV_MASK);
        /* Read wiggle configuration */
        pConfig->lantSwWiggle = (bool)(XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE_MASK);

        /* Read switching settings for RX mode */
        if (mode == gLclRxMode)
        {
            /* Read RX trigger delay */
            pConfig->triggerDelay = (uint16_t)((XCVR_MISC->LCL_RX_CFG0 & XCVR_MISC_LCL_RX_CFG0_RX_DELAY_MASK) >>
                                               XCVR_MISC_LCL_RX_CFG0_RX_DELAY_SHIFT);
            /* Read RX trigger offset */
            pConfig->triggerOffset = (uint8_t)((XCVR_MISC->LCL_RX_CFG0 & XCVR_MISC_LCL_RX_CFG0_RX_DELAY_OFF_MASK) >>
                                               XCVR_MISC_LCL_RX_CFG0_RX_DELAY_OFF_SHIFT);

            /* Read trigger settings */
            temp = (uint8_t)((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL_MASK) >>
                             XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL_SHIFT);
            switch (temp)
            {
                case 0U:
                    pConfig->rxTrig = lclRxTriggerSoftware;
                    break;
                case 1U:
                    pConfig->rxTrig = lclRxTriggerPatternFound;
                    break;
                case 2U:
                    pConfig->rxTrig = lclRxTriggerCrcComplete;
                    break;
                case 3U:
                    pConfig->rxTrig = lclRxTriggerCrcPass;
                    break;
                case 4U:
                    pConfig->rxTrig = lclRxTriggerCtePresent;
                    break;
                case 5U:
                    pConfig->rxTrig = lclRxTriggerAccessAddressFound;
                    break;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
                case 6U:
                    pConfig->rxTrig = lclTxTriggerRsmLclRxTrig;
                    break;
#endif /* #if defined(RADIO_IS_GEN_4P5) */
                default:
                    pConfig->rxTrig = lclRxTriggerInvalid;
                    break;
            }

            /* Read number of samples per interval */
            pConfig->samplesPerInterval = (uint8_t)((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_SPINT_MASK) >>
                                                    XCVR_MISC_LCL_RX_CFG1_RX_SPINT_SHIFT);
            /* Read high and low period durations */
            pConfig->highPeriodDuration[0] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_MASK) >>
                          XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_SHIFT);
            pConfig->lowPeriodDuration[0] = (uint8_t)((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_LO_PER_MASK) >>
                                                      XCVR_MISC_LCL_RX_CFG1_RX_LO_PER_SHIFT);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
            pConfig->highPeriodDuration[1] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_1_MASK) >>
                          XCVR_MISC_LCL_RX_CFG1_RX_HI_PER_1_SHIFT);
            pConfig->lowPeriodDuration[1] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG1 & XCVR_MISC_LCL_RX_CFG1_RX_LO_PER_1_MASK) >>
                          XCVR_MISC_LCL_RX_CFG1_RX_LO_PER_1_SHIFT);
            pConfig->highPeriodDuration[2] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG2 & XCVR_MISC_LCL_RX_CFG2_RX_HI_PER_2_MASK) >>
                          XCVR_MISC_LCL_RX_CFG2_RX_HI_PER_2_SHIFT);
            pConfig->lowPeriodDuration[2] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG2 & XCVR_MISC_LCL_RX_CFG2_RX_LO_PER_2_MASK) >>
                          XCVR_MISC_LCL_RX_CFG2_RX_LO_PER_2_SHIFT);
            pConfig->highPeriodDuration[3] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG2 & XCVR_MISC_LCL_RX_CFG2_RX_HI_PER_3_MASK) >>
                          XCVR_MISC_LCL_RX_CFG2_RX_HI_PER_3_SHIFT);
            pConfig->lowPeriodDuration[3] =
                (uint8_t)((XCVR_MISC->LCL_RX_CFG2 & XCVR_MISC_LCL_RX_CFG2_RX_LO_PER_3_MASK) >>
                          XCVR_MISC_LCL_RX_CFG2_RX_LO_PER_3_SHIFT);
#endif /* #if defined(RADIO_IS_GEN_4P5) */
        }

        /* Read switching settings for TX mode */
        if (mode == gLclTxMode)
        {
            /* Read TX trigger delay */
            pConfig->triggerDelay = (uint16_t)((XCVR_MISC->LCL_TX_CFG0 & XCVR_MISC_LCL_TX_CFG0_TX_DELAY_MASK) >>
                                               XCVR_MISC_LCL_TX_CFG0_TX_DELAY_SHIFT);
            /* Read TX trigger offset */
            pConfig->triggerOffset = (uint8_t)((XCVR_MISC->LCL_TX_CFG0 & XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF_MASK) >>
                                               XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF_SHIFT);

            /* Read trigger settings */
            temp = (uint8_t)(((uint32_t)XCVR_MISC->LCL_TX_CFG1 & XCVR_MISC_LCL_TX_CFG1_TX_ANT_TRIG_SEL_MASK) >>
                             XCVR_MISC_LCL_TX_CFG1_TX_ANT_TRIG_SEL_SHIFT);
            switch (temp)
            {
                case 0U:
                    pConfig->txTrig = lclTxTriggerSoftware;
                    break;
                case 1U:
                    pConfig->txTrig = lclTxTriggerPatternFound;
                    break;
                case 2U:
                    pConfig->txTrig = lclTxTriggerCrcComplete;
                    break;
                case 3U:
                    pConfig->txTrig = lclTxTriggerPaWuComplete;
                    break;
                case 4U:
                    pConfig->txTrig = lclTxTriggerRbmeTxDonePre;
                    break;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
                case 5U:
                    pConfig->txTrig = lclTxTriggerBtleCteEn;
                    break;
                case 6U:
                    pConfig->txTrig = lclTxTriggerRsmLclTxTrig;
                    break;
#endif /* #if defined(RADIO_IS_GEN_4P5) */
                default:
                    pConfig->txTrig = lclTxTriggerInvalid;
                    break;
            }

            /* Read number of samples per interval */
            pConfig->samplesPerInterval = (uint8_t)((XCVR_MISC->LCL_TX_CFG1 & XCVR_MISC_LCL_TX_CFG1_TX_SPINT_MASK) >>
                                                    XCVR_MISC_LCL_TX_CFG1_TX_SPINT_SHIFT);
            /* Read high and low period durations */
            pConfig->highPeriodDuration[0] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG1 & XCVR_MISC_LCL_TX_CFG1_TX_HI_PER_MASK) >>
                          XCVR_MISC_LCL_TX_CFG1_TX_HI_PER_SHIFT);
            pConfig->lowPeriodDuration[0] = (uint8_t)((XCVR_MISC->LCL_TX_CFG1 & XCVR_MISC_LCL_TX_CFG1_TX_LO_PER_MASK) >>
                                                      XCVR_MISC_LCL_TX_CFG1_TX_LO_PER_SHIFT);
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
            pConfig->highPeriodDuration[1] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG1 & XCVR_MISC_LCL_TX_CFG1_TX_HI_PER_1_MASK) >>
                          XCVR_MISC_LCL_TX_CFG1_TX_HI_PER_1_SHIFT);
            pConfig->lowPeriodDuration[1] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG1 & XCVR_MISC_LCL_TX_CFG1_TX_LO_PER_1_MASK) >>
                          XCVR_MISC_LCL_TX_CFG1_TX_LO_PER_1_SHIFT);
            pConfig->highPeriodDuration[2] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG2 & XCVR_MISC_LCL_TX_CFG2_TX_HI_PER_2_MASK) >>
                          XCVR_MISC_LCL_TX_CFG2_TX_HI_PER_2_SHIFT);
            pConfig->lowPeriodDuration[2] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG2 & XCVR_MISC_LCL_TX_CFG2_TX_LO_PER_2_MASK) >>
                          XCVR_MISC_LCL_TX_CFG2_TX_LO_PER_2_SHIFT);
            pConfig->highPeriodDuration[3] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG2 & XCVR_MISC_LCL_TX_CFG2_TX_HI_PER_3_MASK) >>
                          XCVR_MISC_LCL_TX_CFG2_TX_HI_PER_3_SHIFT);
            pConfig->lowPeriodDuration[3] =
                (uint8_t)((XCVR_MISC->LCL_TX_CFG2 & XCVR_MISC_LCL_TX_CFG2_TX_LO_PER_3_MASK) >>
                          XCVR_MISC_LCL_TX_CFG2_TX_LO_PER_3_SHIFT);
#endif /* #if defined(RADIO_IS_GEN_4P5) */
        }

        /* RX_SPINT and TX_SPINT register stores the interval-1, API defines the actual interval */
        pConfig->samplesPerInterval += 1U;

        /* Check mode */
        if ((mode != gLclTxMode) &&
            (mode != gLclRxMode)) /* if neither of these is set then above code was skipped and result is invalid */
        {
            status = gXcvrLclStatusInvalidArgs;
        }
        else
        {
            status = gXcvrLclStatusSuccess;
        }
    }

    return status;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)

xcvrLclStatus_t XCVR_LCL_AntennaSwitchDmaMaskSet(uint16_t MaskDelay, uint8_t MaskDelayOff, uint8_t MaskRefPeriod)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Check delay, delay offset, and reference period */
    if ((MaskDelay > ((uint32_t)XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_MASK >>
                      XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_SHIFT)) ||
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 450)
        /* Comparison only makes sense on KW45 when DMA_MASK_DELAY_OFF field is < 8 bits wide */
        (MaskDelayOff > ((uint8_t)XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF_MASK >>
                         XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF_SHIFT)) ||
#endif
        (MaskRefPeriod > ((uint8_t)XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER_MASK >>
                          XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER_SHIFT)))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Set the mask delay and offset */
        XCVR_MISC->LCL_DMA_MASK_DELAY = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(MaskDelay) |
                                        XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(MaskDelayOff);
        /* Set the mask period */
        XCVR_MISC->LCL_DMA_MASK_PERIOD = XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER(MaskRefPeriod);
    }
    return (status);
}

xcvrLclStatus_t XCVR_LCL_AntennaSwitchDmaMaskGet(uint16_t *pMaskDelay, uint8_t *pMaskDelayOff, uint8_t *pMaskRefPeriod)
{
    xcvrLclStatus_t status = gXcvrLclStatusInvalidArgs;
    /* Read mask delay */
    if (pMaskDelay != NULLPTR)
    {
        *pMaskDelay = (uint16_t)((XCVR_MISC->LCL_DMA_MASK_DELAY & XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_MASK) >>
                                 XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_SHIFT);
        status      = gXcvrLclStatusSuccess;
    }
    /* Read mask delay offset */
    if (pMaskDelayOff != NULLPTR)
    {
        *pMaskDelayOff =
            (uint8_t)((XCVR_MISC->LCL_DMA_MASK_DELAY & XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF_MASK) >>
                      XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF_SHIFT);
        status = gXcvrLclStatusSuccess;
    }
    /* Read mask reference period */
    if (pMaskRefPeriod != NULLPTR)
    {
        *pMaskRefPeriod =
            (uint8_t)((XCVR_MISC->LCL_DMA_MASK_PERIOD & XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER_MASK) >>
                      XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER_SHIFT);
        status = gXcvrLclStatusSuccess;
    }
    return (status);
}

static uint32_t XCVR_LCL_GroupNibbles(const lclAntennaSwitchingLutConfig_t *pConfig, uint8_t startindex)
{
    /* generate a 32 value by group 8 chunks of 4 bits stored in a table (first 4bit value is read from startindex) */
    uint32_t temp_value = 0U;

    for (uint8_t j = startindex; j < (startindex + 8U); j++)
    {
        temp_value |= (((uint32_t)(pConfig->antennaLut[j]) & 0xFUL)
                       << (4U * (j - startindex))); /* assemble 8 chunks of 4 bits into a 32bit word*/
    }
    return (temp_value);
}

static void XCVR_LCL_UngroupNibbles(lclAntennaSwitchingLutConfig_t *pConfig,
                                    uint8_t startindex,
                                    uint32_t sourceregister)
{
    /* split a 32bit value into 8 chunks of 4 bits and store them in the table from startindex position*/
    uint32_t temp_register = sourceregister;

    for (uint8_t j = startindex; j < (startindex + 8U); j++)
    {
        pConfig->antennaLut[j] = (uint8_t)(temp_register & 0xFU); /* break the register value in 8 chunks of 4 bits */
        temp_register          = temp_register >> 4U;
    }
}

xcvrLclStatus_t XCVR_LCL_AntennaSwitchLUTConfigure(const lclAntennaSwitchingLutConfig_t *pConfig)
{
    /* Antenna LUT configuration */
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    /* Verify input parameters in configuration structure */
    if (pConfig == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else if (pConfig->lutWrapPoint > 31U)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        XCVR_MISC->LCL_GPIO_CTRL0 = XCVR_LCL_GroupNibbles(
            pConfig, 0); /* assemble the LCL_GPIO_CTRL0 register in 8 chunks of 4 bits from antennaLut[0..7]*/
        XCVR_MISC->LCL_GPIO_CTRL1 = XCVR_LCL_GroupNibbles(
            pConfig, 8); /* assemble the LCL_GPIO_CTRL1 register in 8 chunks of 4 bits from antennaLut[8..15]*/
        XCVR_MISC->LCL_GPIO_CTRL2 = XCVR_LCL_GroupNibbles(
            pConfig, 16); /* assemble the LCL_GPIO_CTRL2 register in 8 chunks of 4 bits from antennaLut[16..23]*/
        XCVR_MISC->LCL_GPIO_CTRL3 = XCVR_LCL_GroupNibbles(
            pConfig, 24); /* assemble the LCL_GPIO_CTRL3 register in 8 chunks of 4 bits from antennaLut[24..31]*/
        XCVR_MISC->LCL_GPIO_CTRL4 =
            (uint32_t)pConfig->lutWrapPoint; /* update the LCL_GPIO_CTRL4 register with just the LUT_WRAP_PTR */
    }

    return status;
}

xcvrLclStatus_t XCVR_LCL_AntennaSwitchLUTRead(lclAntennaSwitchingLutConfig_t *pConfig)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    /* Verify input parameters in configuration structure */
    if (pConfig == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        XCVR_LCL_UngroupNibbles(
            pConfig, 0,
            XCVR_MISC->LCL_GPIO_CTRL0); /* break the LCL_GPIO_CTRL0 register in 8 chunks of 4 bits to antennaLut[0..7]*/
        XCVR_LCL_UngroupNibbles(pConfig, 8, XCVR_MISC->LCL_GPIO_CTRL1); /* break the LCL_GPIO_CTRL1 register in 8 chunks
                                                                           of 4 bits to antennaLut[8..15]*/
        XCVR_LCL_UngroupNibbles(pConfig, 16, XCVR_MISC->LCL_GPIO_CTRL2); /* break the LCL_GPIO_CTRL2 register in 8
                                                                            chunks of 4 bits to antennaLut[16..23]*/
        XCVR_LCL_UngroupNibbles(pConfig, 24, XCVR_MISC->LCL_GPIO_CTRL3); /* break the LCL_GPIO_CTRL3 register in 8
                                                                            chunks of 4 bits to antennaLut[24..31]*/
        pConfig->lutWrapPoint =
            (uint8_t)(XCVR_MISC->LCL_GPIO_CTRL4 & 0xFFU); /* update lutWrapPoint with LCL_GPIO_CTRL4 register value */
    }

    return status;
}
#endif /* #if defined(RADIO_IS_GEN_4P5) */

/*! *********************************************************************************
 * \brief  Enables LCL module for AoA/AoD antenna switching
 ********************************************************************************** */

xcvrLclStatus_t XCVR_LCL_AntennaSwitchEn(bool enTXmode, bool enRXmode)
{
    uint32_t temp = XCVR_MISC->LCL_CFG0;

    /* Disable TX/RX module */
    temp &= ~(XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_LCL_EN_MASK);

    if (enTXmode)
    {
        /* Enable TX module */
        temp |= XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_LCL_EN(1U);
    }
    if (enRXmode)
    {
        /* Enable TX module */
        temp |= XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_LCL_EN(1U);
    }

    /* Write TX/RX configuration */
    XCVR_MISC->LCL_CFG0 = temp;

    return gXcvrLclStatusSuccess;
}

/*! *********************************************************************************
 * \brief  Blocks Tx/RX in LCL module for AoA/AoD antenna switching
 ********************************************************************************** */

xcvrLclStatus_t XCVR_LCL_AntennaSwitchBlock(bool blockTXmode, bool blockRXmode)
{
    uint32_t temp = XCVR_MISC->LCL_CFG0;

    /* Disable blocking on TX/RX module */
    temp &= ~(XCVR_MISC_LCL_CFG0_LANT_BLOCK_RX_MASK | XCVR_MISC_LCL_CFG0_LANT_BLOCK_TX_MASK);

    if (blockTXmode)
    {
        /* Block TX module */
        temp |= XCVR_MISC_LCL_CFG0_LANT_BLOCK_TX_MASK;
    }
    if (blockRXmode)
    {
        /* Block RX module */
        temp |= XCVR_MISC_LCL_CFG0_LANT_BLOCK_RX_MASK;
    }

    /* Write TX/RX configuration */
    XCVR_MISC->LCL_CFG0 = temp;

    return gXcvrLclStatusSuccess;
}

/*! *********************************************************************************
 * \brief  Reads current LCL antenna switching mode, RX TX or both
 ********************************************************************************** */

xcvrLclStatus_t XCVR_LCL_AntennaSwitchModeRead(lclRxTxMode_t *mode)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    /* Verify input parameters in configuration structure */
    if (mode == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Read block TX configuration */
        bool rxEn = ((XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK) != 0U);
        /* Read block RX configuration */
        bool txEn = ((XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK) != 0U);

        /* Return TX RX status*/
        if (rxEn && txEn)
        {
            *mode = gLclRxTxMode;
        }
        else
        {
            if (!rxEn && !txEn)
            {
                *mode = gLclRxTxNone;
            }
            else
            {
                if (rxEn)
                {
                    *mode = gLclRxMode;
                }
                else
                {
                    *mode = gLclTxMode;
                }
            }
        }
    }

    return status;
}

/*! *********************************************************************************
 * \brief  Handles LCL module programming for pattern matching
 ********************************************************************************** */
xcvrLclStatus_t XCVR_LCL_PatternMatchInit(lclRxTxMode_t mode, const lclPatternMatchConfig_t *pConfig)
{
    uint32_t temp = 0U, temp_lsb = 0U, temp_msb = 0U;
    xcvrLclStatus_t status;
    /* Verify input parameters in configuration structure */
    if (pConfig == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Verify number of bytes is valid */
        assert(pConfig->numBytes < lclPatternMatchNumBytesInvalid);
        /* Check mode */
        assert((mode > gLclRxTxNone) && (mode < gLclRxTxMode));

        lcl_patt_match_mode = gLclRxTxNone;

        /* Configure Pattern match */
        temp = XCVR_MISC->LCL_CFG0;
        temp &= ~XCVR_MISC_LCL_CFG0_PM_NUM_BYTES_MASK;
        temp |= XCVR_MISC_LCL_CFG0_PM_NUM_BYTES(pConfig->numBytes);
        XCVR_MISC->LCL_CFG0 = temp;

        /* Fill LSB and MSB pattern registers */
        for (uint8_t i = 0U; i < 4U; i++)
        {
            temp_lsb <<= 8U;
            temp_lsb |= (uint32_t)(pConfig->pattern[3U - i]);
            temp_msb <<= 8U;
            temp_msb |= (uint32_t)(pConfig->pattern[7U - i]);
        }
        XCVR_MISC->LCL_PM_LSB = temp_lsb;
        XCVR_MISC->LCL_PM_MSB = temp_msb;

        status = gXcvrLclStatusSuccess;
    }

    return status;
}

/*! *********************************************************************************
 * \brief  Enables LCL module programming for pattern matching
 ********************************************************************************** */
xcvrLclStatus_t XCVR_LCL_PatternMatchEn(lclRxTxMode_t mode)
{
    uint32_t temp = 0U;

    /* Check mode */
    assert((mode > gLclRxTxNone) && (mode < gLclRxTxInvalid));

    /* Configure Pattern match */
    temp = XCVR_MISC->LCL_CFG0;

    /* Ensure everything is clear to start */
    temp &= ~(XCVR_MISC_LCL_CFG0_COMP_TX_EN_MASK | XCVR_MISC_LCL_CFG0_COMP_EN_MASK);

    switch (mode)
    {
        case gLclRxTxMode: {
            lcl_patt_match_mode = mode;
            temp |= XCVR_MISC_LCL_CFG0_COMP_TX_EN_MASK | XCVR_MISC_LCL_CFG0_COMP_EN_MASK;
        }
        break;
        case gLclTxMode: {
            if (lcl_patt_match_mode == gLclRxMode)
            {
                lcl_patt_match_mode = gLclRxTxMode;
            }
            else
            {
                lcl_patt_match_mode = gLclTxMode;
            }
            temp |= XCVR_MISC_LCL_CFG0_COMP_TX_EN_MASK | XCVR_MISC_LCL_CFG0_COMP_EN_MASK;
        }
        break;
        case gLclRxMode: {
            if (lcl_patt_match_mode == gLclTxMode)
            {
                lcl_patt_match_mode = gLclRxTxMode;
            }
            else
            {
                lcl_patt_match_mode = gLclRxMode;
            }
            temp |= XCVR_MISC_LCL_CFG0_COMP_EN_MASK;
        }
        break;
        default:
            temp |= 0U; /* need default clause to be non-empty */
            break;
    }

    // Enable block
    temp |= XCVR_MISC_LCL_CFG0_LCL_EN_MASK;
    XCVR_MISC->LCL_CFG0 = temp;

    return gXcvrLclStatusSuccess;
}

/*! *********************************************************************************
 * \brief  Handles LCL module programming for pattern matching
 ********************************************************************************** */
xcvrLclStatus_t XCVR_LCL_PatternMatchModeRead(lclRxTxMode_t *mode)
{
    xcvrLclStatus_t status;
    /* Check pointer */
    if (mode == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Must use a static global because the mode can't be derived from COMP_EN + COMP_TX_EN */
        *mode  = lcl_patt_match_mode;
        status = gXcvrLclStatusSuccess;
    }

    return status;
}

/*! *********************************************************************************
 * \brief  Handles LCL module programming for pattern matching
 ********************************************************************************** */
xcvrLclStatus_t XCVR_LCL_PatternMatchRead(lclPatternMatchConfig_t *pConfig)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    uint32_t temp_pattern;

    /* Verify input parameters in configuration structure */
    if (pConfig == NULLPTR)
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Get Number of Bytes*/
        uint32_t numBytesLcl =
            (XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_PM_NUM_BYTES_MASK) >> XCVR_MISC_LCL_CFG0_PM_NUM_BYTES_SHIFT;

        pConfig->numBytes = (lclPatternMatchNumBytes_t)numBytesLcl;

        /* Get pattern LSB */
        temp_pattern        = XCVR_MISC->LCL_PM_LSB;
        pConfig->pattern[0] = (uint8_t)(temp_pattern & 0xFFU);
        pConfig->pattern[1] = (uint8_t)((temp_pattern >> 8U) & 0xFFU);
        pConfig->pattern[2] = (uint8_t)((temp_pattern >> 16U) & 0xFFU);
        pConfig->pattern[3] = (uint8_t)((temp_pattern >> 24U) & 0xFFU);

        /* Get pattern MSB */
        temp_pattern        = XCVR_MISC->LCL_PM_MSB;
        pConfig->pattern[4] = (uint8_t)(temp_pattern & 0xFFU);
        pConfig->pattern[5] = (uint8_t)((temp_pattern >> 8U) & 0xFFU);
        pConfig->pattern[6] = (uint8_t)((temp_pattern >> 16U) & 0xFFU);
        pConfig->pattern[7] = (uint8_t)((temp_pattern >> 24U) & 0xFFU);
    }

    return status;
}

/*! *********************************************************************************
 * \brief  Disable pattern matching in LCL
 ********************************************************************************** */
xcvrLclStatus_t XCVR_LCL_PatternMatchDisable(lclRxTxMode_t mode)
{
    uint32_t temp = 0U;

    /* Check mode */
    assert((mode > gLclRxTxNone) && (mode < gLclRxTxInvalid));

    /* Configure Pattern match */
    temp = XCVR_MISC->LCL_CFG0;

    switch (mode)
    {
        case gLclRxTxMode: {
            temp &= ~(XCVR_MISC_LCL_CFG0_COMP_TX_EN_MASK | XCVR_MISC_LCL_CFG0_COMP_EN_MASK);
            lcl_patt_match_mode = gLclRxTxNone;
        }
        break;
        case gLclTxMode: {
            temp &= ~XCVR_MISC_LCL_CFG0_COMP_TX_EN_MASK;
            /* Only disable the specified part of the pattern match mode */
            if (lcl_patt_match_mode == gLclRxTxMode)
            {
                lcl_patt_match_mode = gLclRxMode;
            }
            else
            {
                if (lcl_patt_match_mode == gLclTxMode)
                {
                    lcl_patt_match_mode = gLclRxTxNone;
                }
            }
        }
        break;
        case gLclRxMode: {
            temp &= ~XCVR_MISC_LCL_CFG0_COMP_EN_MASK;
            /* Only disable the specified part of the pattern match mode */
            if (lcl_patt_match_mode == gLclRxTxMode)
            {
                lcl_patt_match_mode = gLclTxMode;
            }
            else
            {
                if (lcl_patt_match_mode == gLclRxMode)
                {
                    lcl_patt_match_mode = gLclRxTxNone;
                }
            }
        }
        break;
        default:
            temp |= 0U; /* need default clause to be non-empty */
            break;
    }
    // Write new config
    XCVR_MISC->LCL_CFG0 = temp;

    return gXcvrLclStatusSuccess;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
#if defined(SUPPORT_AOA_AOD_IN_XCVR) && (SUPPORT_AOA_AOD_IN_XCVR == 1)
/*!
 * @brief Handle LCL module programming for BLE AoA/AoD use cases.
 *
 * This function configures the required LCL HW registers that are not controlled by the BLE LL in case of AoA/AOD .
 *
 * @param[in] BLELocalizationMethod Localization method to handle (AoA or AOD).
 * @param[in] BLELocalizationRole Role of the device in the method (Emitter or receiver).
 * @param[in] Switching_Pattern_Length Length of the pattern.
 * @param[in] Antenna_IDs Switching pattern to apply (table where antennae 0 is specified as 0, antenna 1 as 1 and so
 on...).
 *
 @return None.
 *
 */

static void XCVR_LCL_BLEAoDAoALutConfig(uint8_t Switching_Pattern_Length, const uint8_t *Antenna_IDs)
{
    uint32_t lcl_gpio_ctrl[4] = {0, 0, 0, 0};
    uint32_t temp_register;
    uint8_t i, j;
    uint8_t nb_reg_to_program;
    uint8_t nb_chunk_to_program;

    /* evaluate the number of LCL registers to program based on the pattern length */
    nb_reg_to_program = (uint8_t)(((Switching_Pattern_Length - 1U) / 8U) + 1U);

    /* Antenna LUT configuration */
    for (i = 0U; i < nb_reg_to_program; i++) /* loop over all 4 registers:  LCL_GPIO_CTRL0..LCL_GPIO_CTRL3*/
    {
        nb_chunk_to_program = 8U;
        if (Switching_Pattern_Length < ((i + 1U) * 8U))
        {
            nb_chunk_to_program = (uint8_t)(Switching_Pattern_Length - (8U * i));
        }
        temp_register = 0;
        for (j = 0U; j < nb_chunk_to_program; j++) /* build register chunks*/
        {
            temp_register = temp_register | ((uint32_t)(PatternToLUT[Antenna_IDs[(8U * i) + j]]) << (j * 4U));
        }
        lcl_gpio_ctrl[i] = temp_register;
    }

    /* write generated values in LCL registers */
    XCVR_MISC->LCL_GPIO_CTRL0 = lcl_gpio_ctrl[0];
    XCVR_MISC->LCL_GPIO_CTRL1 = lcl_gpio_ctrl[1];
    XCVR_MISC->LCL_GPIO_CTRL2 = lcl_gpio_ctrl[2];
    XCVR_MISC->LCL_GPIO_CTRL3 = lcl_gpio_ctrl[3];

    /* Programm wrap pointeur */
    XCVR_MISC->LCL_GPIO_CTRL4 = XCVR_MISC_LCL_GPIO_CTRL4_LUT_WRAP_PTR((uint8_t)(Switching_Pattern_Length - 1U));
}

xcvrLclStatus_t XCVR_LCL_BLEAoDAoAInit(lclBleCteLocRate_t BLEDataRate,
                                       lclBleCteLocMethod_t BLELocalizationMethod,
                                       lclBleCteLocRole_t BLELocalizationRole,
                                       uint8_t Switching_Pattern_Length,
                                       const uint8_t *Antenna_IDs
#ifdef DMACONFIG
                                       ,
                                       uint32_t DMA_Buffer_Address,
                                       uint32_t DMA_Buffer_Size
#endif
)
{
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;
    uint32_t temp_register;
    uint8_t TxSamplePerInterval, RxSamplePerInterval;

    /* Verify input parameters in configuration structure */
    if ((Antenna_IDs == NULLPTR) || (Switching_Pattern_Length < 1U))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Program LUT related LCL registers */
        XCVR_LCL_BLEAoDAoALutConfig(Switching_Pattern_Length, Antenna_IDs);

        /* Program other registers not controlled by BLE LL*/

        /* Evaluate number of sample per 1us interval */
        if (BLEDataRate == lclBleCteRate2M)
        {
            RxSamplePerInterval = 8U; /* Rx sampling frequency is 8MHz for 2Mbps */
        }
        else
        {
            RxSamplePerInterval = 4U; /* Rx sampling frequency is 4MHz for 1Mbps */
        }
        TxSamplePerInterval = 8U; /* Tx sampling frequency is 8MHz for both 1Mbps and 2Mbps */

        /* Antenna switch*/
        /* Configure TX Trigger Source and number of samples per interval*/
        temp_register = XCVR_MISC->LCL_TX_CFG1;
        temp_register &= ~(XCVR_MISC_LCL_TX_CFG1_TX_ANT_TRIG_SEL_MASK | XCVR_MISC_LCL_TX_CFG1_TX_SPINT_MASK);
        temp_register |= XCVR_MISC_LCL_TX_CFG1_TX_ANT_TRIG_SEL(BLECTE_TX_TRIG_SEL) |
                         XCVR_MISC_LCL_TX_CFG1_TX_SPINT(TxSamplePerInterval);
        XCVR_MISC->LCL_TX_CFG1 = temp_register;

        /* Configure RX Trigger Source  and number of samples per interval*/
        temp_register = XCVR_MISC->LCL_RX_CFG1;
        temp_register &= ~(XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL_MASK | XCVR_MISC_LCL_RX_CFG1_RX_SPINT_MASK);
        temp_register |= XCVR_MISC_LCL_RX_CFG1_RX_ANT_TRIG_SEL(BLECTE_RX_TRIG_SEL) |
                         XCVR_MISC_LCL_RX_CFG1_RX_SPINT(RxSamplePerInterval);
        XCVR_MISC->LCL_RX_CFG1 = temp_register;

        /* Configure TX trigger delay and offset */
        XCVR_MISC->LCL_TX_CFG0 =
            XCVR_MISC_LCL_TX_CFG0_TX_DELAY(BLECTE_TX_DELAY) | XCVR_MISC_LCL_TX_CFG0_TX_DELAY_OFF(BLECTE_TX_DELAY_OFF);
        /* Configure trigger delay and offset */
        XCVR_MISC->LCL_RX_CFG0 =
            XCVR_MISC_LCL_RX_CFG0_RX_DELAY(BLECTE_RX_DELAY) | XCVR_MISC_LCL_RX_CFG0_RX_DELAY_OFF(BLECTE_RX_DELAY_OFF);

        /* DMA */
        XCVR_MISC->LCL_DMA_MASK_PERIOD = XCVR_MISC_LCL_DMA_MASK_PERIOD_DMA_MASK_REF_PER(BLECTE_DMA_MASK_PERIOD);
        XCVR_MISC->LCL_DMA_MASK_DELAY  = XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY_OFF(BLECTE_DMA_MASK_DELAY_OFF) |
                                        XCVR_MISC_LCL_DMA_MASK_DELAY_DMA_MASK_DELAY(BLECTE_DMA_MASK_DELAY);

        if (BLELocalizationMethod == lclBleCteMethodAoA)
        {
            if (BLELocalizationRole == lclBleCteRoleTransmitter)
            {
                /* AoA Transmitter has only one antenna. It emits CTE and receives without antenna switching, disable
                 * antenna switching */
                temp_register = XCVR_MISC->LCL_CFG0;
                temp_register &= ~(XCVR_MISC_LCL_CFG0_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK |
                                   XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK);
                XCVR_MISC->LCL_CFG0 = temp_register;
            }
            else /* BLELocalizationRole == lclBleCteRoleReceiver */
            {
                /* AoA Receiver receives with antenna switching and DMA I/Q recording */
                /* Configure LCL_EN, RX_LCL_EN, INV and WIGGLE bits */
                temp_register = XCVR_MISC->LCL_CFG0;
                temp_register &= ~(XCVR_MISC_LCL_CFG0_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK |
                                   XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_LANT_INV_MASK |
                                   XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE_MASK);
                temp_register |= XCVR_MISC_LCL_CFG0_LCL_EN(1U) | XCVR_MISC_LCL_CFG0_TX_LCL_EN(0U) |
                                 XCVR_MISC_LCL_CFG0_RX_LCL_EN(1U) |
                                 XCVR_MISC_LCL_CFG0_LANT_INV(BLECTE_LANT_INV) | /* Polarity of lant_sw signal */
                                 XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE(
                                     BLECTE_LANT_SW_WIGGLE); /* Enables SW wiggle output on lant_sw signal */
                XCVR_MISC->LCL_CFG0 = temp_register;
            }
        }
        else /* lclBleCteMethodAoD */
        {
            if (BLELocalizationRole == lclBleCteRoleTransmitter)
            {
                /* Transmitter emits CTE with antenna switching */
                temp_register = XCVR_MISC->LCL_CFG0;
                temp_register &= ~(XCVR_MISC_LCL_CFG0_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK |
                                   XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_LANT_INV_MASK |
                                   XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE_MASK);
                temp_register |= XCVR_MISC_LCL_CFG0_LCL_EN(1U) | XCVR_MISC_LCL_CFG0_TX_LCL_EN(1U) |
                                 XCVR_MISC_LCL_CFG0_RX_LCL_EN(0U) |
                                 XCVR_MISC_LCL_CFG0_LANT_INV(BLECTE_LANT_INV) | /* Polarity of lant_sw signal */
                                 XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE(
                                     BLECTE_LANT_SW_WIGGLE); /* Enables SW wiggle output on lant_sw signal */
                XCVR_MISC->LCL_CFG0 = temp_register;
            }
            else /* lclBleCteRoleReceiver */
            {
                /* Receiver receives without antenna switching but with DMA I/Q recording */
                temp_register = XCVR_MISC->LCL_CFG0;
                temp_register &= ~(XCVR_MISC_LCL_CFG0_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK |
                                   XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK | XCVR_MISC_LCL_CFG0_LANT_INV_MASK |
                                   XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE_MASK);
                temp_register |= XCVR_MISC_LCL_CFG0_LCL_EN(1U) | XCVR_MISC_LCL_CFG0_TX_LCL_EN(0U) |
                                 XCVR_MISC_LCL_CFG0_RX_LCL_EN(0U) |
                                 XCVR_MISC_LCL_CFG0_LANT_INV(BLECTE_LANT_INV) | /* Polarity of lant_sw signal */
                                 XCVR_MISC_LCL_CFG0_LANT_SW_WIGGLE(
                                     BLECTE_LANT_SW_WIGGLE); /* Enables SW wiggle output on lant_sw signal */
                XCVR_MISC->LCL_CFG0 = temp_register;
            }
        }
    }

#ifdef DMACONFIG

    /* This section is not finalized and need to be reviewed. Not even clear if this is meaningful... */

    if (BLELocalizationRole == lclBleCteRoleReceiver)
    {
        /* XCVR DMA configuration */

        /* Select SRC (100b) in DFT_CTRL RX_IQ_OUT_SEL field*/
        temp_register = XCVR_RX_DIG->DFT_CTRL;
        temp_register &= ~(XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL_MASK);
        temp_register |= XCVR_RX_DIG_DFT_CTRL_DFT_RX_IQ_OUT_SEL(4U); /* Select SRC */
        XCVR_RX_DIG->DFT_CTRL = temp_register;
        /* Select RXDIG_IQ as DMA_PAGE to capture data coming from RXDIG based on DFT_CTRL_RX_IQ_OUT_SEL */
        XCVR_MISC->DMA_CTRL = XCVR_MISC_DMA_CTRL_DMA_PAGE(1U) | XCVR_MISC_DMA_CTRL_DMA_START_TRG(11U) |
                              XCVR_MISC_DMA_CTRL_DMA_START_EDGE(0U) | XCVR_MISC_DMA_CTRL_DMA_DEC(0U) |
                              XCVR_MISC_DMA_CTRL_DMA_START_DLY(0U) | XCVR_MISC_DMA_CTRL_DMA_EN(1U) |
                              XCVR_MISC_DMA_CTRL_DMA_SIGNAL_VALID_MASK_EN(1U);

        /* DSB (Data Stream Buffer) DMA configuration */

        DSB0->DADDR = DSB_DADDR_DADDR(DMA_Buffer_Address);
        DSB0->XCR   = DSB_XCR_TCNT(DMA_Buffer_Size);
        DSB0->WMC =
            DSB_WMC_WMRK((((DSB0->WMC) & DSB_WMC_SIZE_MASK) >> DSB_WMC_SIZE_SHIFT) / 2U); /*half of the FIFO size, TBC*/
        DSB0->CSR = DSB_CSR_CBT_EN(1U) | DSB_CSR_DMA_EN(1U) | DSB_CSR_DSB_EN(1U);
    }

    /* TO DO:   - CREATE API TO START/STOP DMA ? -> DEINIT API*/
#endif

    return status;
}

void XCVR_LCL_BLEAoDAoADeInit(void)
{
    DSB0->CSR           = 0U; /* Disable DSB */
    XCVR_MISC->DMA_CTRL = 0U; /* Disable DMA */
    XCVR_MISC->LCL_CFG0 = 0U; /* Disable all antenna switching */
}

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
xcvrLclStatus_t XCVR_LCL_SetLUTAntennaTable(uint8_t TableLength, const uint8_t *Table)
{
    uint8_t i;
    const uint8_t *tbl_ptr = Table;
    xcvrLclStatus_t status = gXcvrLclStatusSuccess;

    // TODO: add NULLPTR input check
    if ((Table == NULLPTR) || (TableLength > LUT_TBL_LENGTH))
    {
        status = gXcvrLclStatusInvalidArgs;
    }
    else
    {
        /* Populate the table */
        for (i = 0U; i < TableLength; i++)
        {
            PatternToLUT[i] = *tbl_ptr;
            tbl_ptr++;
        }
        /* Ensure remainder of the table is populated with zeros */
        if (TableLength < LUT_TBL_LENGTH)
        {
            for (i = TableLength; i < LUT_TBL_LENGTH; i++)
            {
                PatternToLUT[i] = 0U;
            }
        }
    }
    return (status);
}

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
void XCVR_LCL_GetLUTAntennaTable(uint8_t *Table)
{
    uint8_t *tbl_ptr = Table;
    // TODO: add NULLPTR input check
    for (uint8_t i = 0U; i < LUT_TBL_LENGTH; i++)
    {
        *tbl_ptr = PatternToLUT[i];
        tbl_ptr++;
    }
}
#endif /* defined (SUPPORT_AOA_AOD_IN_XCVR) && (SUPPORT_AOA_AOD_IN_XCVR == 1) */
#endif

/*! *********************************************************************************
 * \brief  Force a manual trigger to LCL
 ********************************************************************************** */
void XCVR_LCL_SwTriggerAssert(void)
{
    lclRxTxMode_t mode     = gLclRxTxNone;
    xcvrLclStatus_t status = gXcvrLclStatusFail;

    /* Read current mode */
    status = XCVR_LCL_AntennaSwitchModeRead(&mode);
    (void)status;

    /* Clear the SW_TRIG bit in LCL_CFG0 */
    XCVR_MISC->LCL_CFG0 &= ~XCVR_MISC_LCL_CFG0_SW_TRIG_MASK;

    /* Rearm module for multiple triggers - TX */
    if ((mode == gLclTxMode) || (mode == gLclRxTxMode))
    {
        /* Disable TX */
        XCVR_MISC->LCL_CFG0 &= ~XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK;
        /* Enable TX */
        XCVR_MISC->LCL_CFG0 |= XCVR_MISC_LCL_CFG0_TX_LCL_EN_MASK;
    }
    /* Rearm module for multiple triggers - RX */
    if ((mode == gLclRxMode) || (mode == gLclRxTxMode))
    {
        /* Disable RX */
        XCVR_MISC->LCL_CFG0 &= ~XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK;
        /* Enable RX */
        XCVR_MISC->LCL_CFG0 |= XCVR_MISC_LCL_CFG0_RX_LCL_EN_MASK;
    }

    /* Assert the SW_TRIG bit in LCL_CFG0 */
    XCVR_MISC->LCL_CFG0 |= XCVR_MISC_LCL_CFG0_SW_TRIG_MASK;
}

/*! *********************************************************************************
 * \brief  Disable LCL module, both antenna switch and pattern match
 ********************************************************************************** */
void XCVR_LCL_DeInit(void)
{
    /* Clear all individual LCL_EN enable bit */
    XCVR_MISC->LCL_CFG0 &= ~XCVR_MISC_LCL_CFG0_LCL_EN_MASK;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
    /* Disable mask capability from RX DIG to allow independent IQ capture  */
    XCVR_RX_DIG->CTRL1 &= ~(XCVR_RX_DIG_CTRL1_RX_IQ_PH_OUTPUT_COND_MASK);
#endif /* #if defined(RADIO_IS_GEN_4P5) */
}

/*! *********************************************************************************
 * \brief  Gets LCL module enable status
 ********************************************************************************** */
bool XCVR_LCL_GetEn(void)
{
    /* Returns whether LCL is enabled or not  */
    return ((XCVR_MISC->LCL_CFG0 & XCVR_MISC_LCL_CFG0_LCL_EN_MASK) != 0U);
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 400)

void RF_LANT_SW_IRQHandler(void)
{
    /* Clear the Localization Antenna Switch Flag */
    XCVR_MISC->LCL_CFG1 |= XCVR_MISC_LCL_CFG1_LANT_SW_FLAG_MASK;

#if (0)
    lclRxTxMode_t mode = gLclRxTxNone;
    lclAntennaSwitchingPatternConfig_t ant_sw_cfg;
    xcvrLclStatus_t status = glclStatusFail;

    /* Read current LCL mode (RX/AoA or TX/AoD) for Antenna Switching*/
    status = XCVR_LCL_AntennaSwitchModeRead(&mode);
    assert(status == gXcvrLclStatusSuccess);

    /* Read current Antenna Switch config */
    XCVR_LCL_AntennaSwitchRead(mode, &ant_sw_cfg);
#endif

    if (lant_sw_handler.user_callback != NULLPTR)
    {
        lant_sw_handler.user_callback(lant_sw_handler.userData);
    }
}
#endif /* defined(RADIO_IS_GEN_4P0) || defined(RADIO_IS_GEN_4P5) */
