/*
 * Copyright 2018-2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */
#include "rfmc_ctrl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RF_OSCILLATOR_STAYS_ON (false) /* Control whether RF_OSC can be left on all the time. */
#define RF_OSCILLATOR_READY ((RFMC->XO_STAT & RFMC_XO_STAT_XTAL_RDY_MASK) != 0x0UL)
#define NULLPTR ((void *)(0x0)) /* MISRA safe definition for NULL */

#ifndef FPGA_TARGET /* make sure the FPGA_TARGET flag is defined to support XCVR compilation */
#define FPGA_TARGET (0)
#endif /* FPGA_TARGET */

/*******************************************************************************
 * Variables
 ******************************************************************************/
static rfmc_cb_fptr callback_list[RFMC_NUM_CALLBACKS] = {NULLPTR, NULLPTR, NULLPTR, NULLPTR,
                                                         NULLPTR, NULLPTR, NULLPTR, NULLPTR};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void RFMC_IRQHandler(void); /* Replaces weakly defined handler in startup_***.s file */

#if !defined(GCOV_DO_COVERAGE) /* routine is local when not testing code coverage */
static void RFMC_DispatchIRQ(uint32_t temp_register_rf, uint32_t temp_register_xo);
#endif /* !defined(GCOV_DO_COVERAGE) */

/*******************************************************************************
 * Code
 ******************************************************************************/
bool RFMC_RegisterCb(rfmc_cb_fptr fptr, rfmcIrqIndex_t cb_index)
{
    bool retval = true;
    if (cb_index >= RFMC_NUM_CALLBACKS)
    {
        retval = false;
    }
    else
    {
        callback_list[(uint8_t)cb_index] =
            fptr; /* update the callbacks list with the passed in pointer (including NULLPTR) */
    }

    return retval;
}

bool RFMC_IrqEnDis(rfmcIrqIndex_t irq_index, bool irq_enabled)
{
    bool retval             = true;
    volatile uint32_t *addr = &(RFMC->XO_STAT);
    uint32_t mask           = 0U;
    /* Determine the register address and mask for the bit to be set or cleared */
    switch (irq_index)
    {
        case RFMC_XO_RDY_STAT:
            addr = &(RFMC->XO_CTRL);
            mask = RFMC_XO_CTRL_RDY_IE_MASK;
            break;
        case RFMC_XO_INT_STAT:
            addr = &(RFMC->XO_CTRL);
            mask = RFMC_XO_CTRL_INT_IE_MASK;
            break;
        case RFMC_XO_EXT_STAT:
            addr = &(RFMC->XO_CTRL);
            mask = RFMC_XO_CTRL_EXT_IE_MASK;
            break;
        case RFMC_RF_BLE_WKUP_STAT:
            addr = &(RFMC->RF2P4GHZ_CTRL);
            mask = RFMC_RF2P4GHZ_CTRL_BLE_WKUP_IE_MASK;
            break;
        case RFMC_RF_MAN_WKUP_STAT:
            addr = &(RFMC->RF2P4GHZ_CTRL);
            mask = RFMC_RF2P4GHZ_CTRL_MAN_WKUP_IE_MASK;
            break;
        case RFMC_RF_WOR_WKUP_STAT:
            addr = &(RFMC->RF2P4GHZ_CTRL);
            mask = RFMC_RF2P4GHZ_CTRL_WOR_WKUP_IE_MASK;
            break;
        case RFMC_RF_LP_WKUP_STAT:
            addr = &(RFMC->RF2P4GHZ_CTRL);
            mask = RFMC_RF2P4GHZ_CTRL_LP_WKUP_IE_MASK;
            break;
        case RFMC_RF_ACTIVE_STAT:
            addr = &(RFMC->RF2P4GHZ_CTRL);
            mask = RFMC_RF2P4GHZ_CTRL_RFACT_IE_MASK;
            break;
        default:
            retval = false;
            break;
    }

    if (retval) /* no failure in the switch statement above */
    {
        if (irq_enabled)
        {
            *addr |= mask; /* set the bit at the desired address */
        }
        else
        {
            *addr &= ~(mask); /* clear the bit at the desired address */
        }
    }

    return retval;
}

void RFMC_rf_osc_startup(void)
{
    /* Bring RFMC out of reset */
    RFMC->CTRL &= ~RFMC_CTRL_RFMC_RST_MASK;

    /* Enable XO for 2.4GHz radio */
    RFMC->RF2P4GHZ_CTRL |= RFMC_RF2P4GHZ_CTRL_XO_EN_MASK;

    /* Release radio reset(s) and power up radio (but NOT the NBU CPU) */
    RFMC->RF2P4GHZ_CTRL &= ~(RFMC_RF2P4GHZ_CTRL_LP_ENTER_MASK | /* make sure Low Power is not requested by SW */
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
                             RFMC_RF2P4GHZ_CTRL_RF_POR_MASK | /* ensure that RF POR is not requested */
#endif
                             RFMC_RF2P4GHZ_CTRL_RST_MASK); /* ensure that reset of the radio is not requested */
}

bool RFMC_check_radio_warmup_complete(bool wait_for_complete)
{
    bool warmup_complete = false;
    do
    {
        warmup_complete = (
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
            ((RFMC->RF2P4GHZ_STAT & RFMC_RF2P4GHZ_STAT_RST_STAT_MASK) == 0U) && /* RFMC is not in reset */
#endif
#if (defined(FPGA_TARGET) && (FPGA_TARGET == 0UL)) /* Don't check RF_OSC ready on FPGA, it never asserts */
            (RF_OSCILLATOR_READY) &&               /* RF_OSC is ready */
#endif                                             /* (defined(FPGA_TARGET) && (FPGA_TARGET == 0UL)) */
            ((RFMC->RF2P4GHZ_STAT & RFMC_RF2P4GHZ_STAT_LP_ACK_STAT_MASK) == 0x0UL)); /* RFMC is not in low power */
    } while (
        !warmup_complete &&
        wait_for_complete); /* Only continues to wait if the wait_for_complete flag == true and the status is false */

    return warmup_complete; /* Return the state of warmup complete */
}

void RFMC_rf_osc_shutdown(void)
{
    /* RF_OSCILLATOR_STAYS_ON flag controls whether XCVR can shut off RF OSC or not */
#if (!RF_OSCILLATOR_STAYS_ON)
    RFMC->CTRL &= ~RFMC_XO_CTRL_XTAL_OUT_EN_MASK;
#endif
}

void RFMC_radio_reset(void)
{
    /* Assert and then clear XCVR soft reset to reset only the XCVR hardware (no CPU reset) */
    /* Does not reset registers, XCVR_Init() must be used for reconfiguring registers */
    XCVR_MISC->XCVR_CTRL |= XCVR_MISC_XCVR_CTRL_XCVR_SOFT_RESET_MASK;
    XCVR_MISC->XCVR_CTRL &= ~(XCVR_MISC_XCVR_CTRL_XCVR_SOFT_RESET_MASK);
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450) && !defined(KW45B41Z82_NBU_SERIES) && !defined(KW45B41Z83_NBU_SERIES)
void RFMC_nbu_enter_reset(void)
{
    /* Hold NBU CPU in reset */
    RFMC->RF2P4GHZ_CTRL |= RFMC_RF2P4GHZ_CTRL_CPU_RST_MASK;
}

void RFMC_nbu_release_reset(void)
{
    /* Release NBU CPU reset  */
    RFMC->RF2P4GHZ_CTRL &= ~(RFMC_RF2P4GHZ_CTRL_CPU_RST_MASK); /* Release NBU CPU reset */
    CIU2->CIU2_CPU_CPU2_CTRL = CIU2_CIU2_CPU_CPU2_CTRL_VINITHI_MASK;
}

#endif /* defined(RADIO_IS_GEN_4P5) && !defined(KW45B41Z82_NBU_SERIES) && !defined(KW45B41Z83_NBU_SERIES) */

void RFMC_GetUniqueId(rfmc_uid_t *uid)
{
    uid->MH = RFMC->VERID;
#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN == 400)
    uid->ML = RFMC->UID_MSB;
    uid->L  = RFMC->UID_LSB;
#elif defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)

    uid->ML = RADIO_CTRL->UID_MSB;
    uid->L  = RADIO_CTRL->UID_LSB;
#else
#error "No valid radio generation defined for RFMC."
#endif
}

void RFMC_SetXtalTrim(uint8_t xtalTrim)
{
    uint32_t temp;

    /* Apply a trim value to the crystal oscillator */
    temp = RFMC->XO_TEST;
    temp &= ~(RFMC_XO_TEST_CDAC_MASK);
    RFMC->XO_TEST = temp | RFMC_XO_TEST_CDAC(xtalTrim);
}

uint8_t RFMC_GetXtalTrim(void)
{
    uint8_t xtalVal;

    /* Fetch a trim value from the crystal oscillator trim bitfield */
    xtalVal = (uint8_t)((RFMC->XO_TEST & RFMC_XO_TEST_CDAC_MASK) >> RFMC_XO_TEST_CDAC_SHIFT);

    return xtalVal;
}

#if defined(NXP_RADIO_GEN) && (NXP_RADIO_GEN >= 450)
rfmcStatus_t RFMC_SetRfGpoConfig(rfmcGpoCoex_t gpo_setting, uint8_t gpo_output_buf_ena)
{
    rfmcStatus_t status = gRfmcSuccess_c;
    if (gpo_setting >= RFMC_GPO_INVALID)
    {
        status = gRfmcInvalidParameters_c;
    }
    else
    {
        uint8_t temp_field = (uint8_t)gpo_setting; /* map the enumeration type to uint8 */
        RFMC->RF2P4GHZ_COEXT &= ~(RFMC_RF2P4GHZ_COEXT_RFGPO_SRC_MASK | RFMC_RF2P4GHZ_COEXT_RFGPO_OBE_MASK);
        /* Set the GPO function selection and output buffer enable */
        RFMC->RF2P4GHZ_COEXT |=
            RFMC_RF2P4GHZ_COEXT_RFGPO_SRC(temp_field) | RFMC_RF2P4GHZ_COEXT_RFGPO_OBE(gpo_output_buf_ena);
    }

    return status;
}
#endif

void RFMC_IRQHandler(void)
{
    /* Use temp status variables to enforce the order of volatile accesses */
    uint32_t temp_rf_stat = RFMC->RF2P4GHZ_STAT;
    uint32_t temp_xo_stat = RFMC->XO_STAT;
    /* Handle the interrupts according to the asserted bits. */
    RFMC_DispatchIRQ(temp_rf_stat, temp_xo_stat);
}

#if !defined(GCOV_DO_COVERAGE) /* routine is local except when testing code coverage */
static void RFMC_DispatchIRQ(uint32_t temp_register_rf, uint32_t temp_register_xo)
#else
void RFMC_DispatchIRQ(uint32_t temp_register_rf, uint32_t temp_register_xo)
#endif /* !defined(GCOV_DO_COVERAGE) */
{
    rfmc_cb_fptr temp_fptr;
    // Interrupts from radio crystal oscillator (XO)
    bool xo_rdy_stat = ((temp_register_xo & RFMC_XO_STAT_RDY_FLAG_MASK) == RFMC_XO_STAT_RDY_FLAG_MASK);
    bool xo_int_stat = ((temp_register_xo & RFMC_XO_STAT_INT_FLAG_MASK) == RFMC_XO_STAT_INT_FLAG_MASK);
    bool xo_ext_stat = ((temp_register_xo & RFMC_XO_STAT_EXT_FLAG_MASK) == RFMC_XO_STAT_EXT_FLAG_MASK);

    // Interrupts from low power radio wakup events
    bool rf_ble_wkup_stat =
        ((temp_register_rf & RFMC_RF2P4GHZ_STAT_BLE_WKUP_FLAG_MASK) == RFMC_RF2P4GHZ_STAT_BLE_WKUP_FLAG_MASK);
    bool rf_man_wkup_stat =
        ((temp_register_rf & RFMC_RF2P4GHZ_STAT_MAN_WKUP_FLAG_MASK) == RFMC_RF2P4GHZ_STAT_MAN_WKUP_FLAG_MASK);
    bool rf_wor_wkup_stat =
        ((temp_register_rf & RFMC_RF2P4GHZ_STAT_WOR_WKUP_FLAG_MASK) == RFMC_RF2P4GHZ_STAT_WOR_WKUP_FLAG_MASK);
    bool rf_lp_wkup_stat =
        ((temp_register_rf & RFMC_RF2P4GHZ_STAT_LP_WKUP_FLAG_MASK) == RFMC_RF2P4GHZ_STAT_LP_WKUP_FLAG_MASK);
    bool rf_active_stat =
        ((temp_register_rf & RFMC_RF2P4GHZ_STAT_RFACT_FLAG_MASK) == RFMC_RF2P4GHZ_STAT_RFACT_FLAG_MASK);

    if (xo_rdy_stat)
    {
        // XTAL ready output from XO analog asserts
        temp_fptr = callback_list[RFMC_XO_RDY_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_fptr(temp_register_rf,
                      temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers */
        }
        RFMC->XO_STAT |= RFMC_XO_STAT_RDY_FLAG_MASK;
    }

    if (xo_int_stat)
    {
        // internal source (e.g. SCG, or 2.4GHz radios) has requested the XO to be enabled
        temp_fptr = callback_list[RFMC_XO_INT_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->XO_STAT |= RFMC_XO_STAT_INT_FLAG_MASK;
    }

    if (xo_ext_stat)
    {
        // external source (XTAL_OUT_EN bit) has requested the XO to be enabled
        temp_fptr = callback_list[RFMC_XO_EXT_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->XO_STAT |= RFMC_XO_STAT_EXT_FLAG_MASK;
    }

    if (rf_lp_wkup_stat)
    {
        // Radio low power acknowledge from the SPC, indicating radio power domain wakeup is complete
        // Note, this flag will assert for all sources of radio wakeup e.g. MAN/WOR/BluetoothLE controller
        //       or software clear of LP_ENTER bit
        temp_fptr = callback_list[RFMC_RF_LP_WKUP_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->RF2P4GHZ_STAT |= RFMC_RF2P4GHZ_STAT_LP_WKUP_FLAG_MASK;
    }

    if (rf_ble_wkup_stat)
    {
        // BluetoothLE wakeup event
        temp_fptr = callback_list[RFMC_RF_BLE_WKUP_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->RF2P4GHZ_STAT |= RFMC_RF2P4GHZ_STAT_BLE_WKUP_FLAG_MASK;
    }

    if (rf_man_wkup_stat)
    {
        // MAN wakeup event
        temp_fptr = callback_list[RFMC_RF_MAN_WKUP_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->RF2P4GHZ_STAT |= RFMC_RF2P4GHZ_STAT_MAN_WKUP_FLAG_MASK;
    }

    if (rf_wor_wkup_stat)
    {
        // WOR wakeup event
        temp_fptr = callback_list[RFMC_RF_WOR_WKUP_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->RF2P4GHZ_STAT |= RFMC_RF2P4GHZ_STAT_WOR_WKUP_FLAG_MASK;
    }

    if (rf_active_stat)
    {
        // Interrupt from the assertion of the RF_ACTIVE pin
        temp_fptr = callback_list[RFMC_RF_ACTIVE_STAT];
        if (temp_fptr != NULLPTR)
        {
            temp_register_xo = RFMC->XO_STAT;
            temp_register_rf = RFMC->RF2P4GHZ_STAT;
            temp_fptr(
                temp_register_rf,
                temp_register_xo); /* Call the specified callback pointer with the rf and xo status registers updated*/
        }
        RFMC->RF2P4GHZ_STAT |= RFMC_RF2P4GHZ_STAT_RFACT_FLAG_MASK;
    }
}
