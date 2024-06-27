/***********************************************************************************************************************
 * Copyright [2020-2021] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics Corporation and/or its affiliates and may only
 * be used with products of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.
 * Renesas products are sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for
 * the selection and use of Renesas products and Renesas assumes no liability.  No license, express or implied, to any
 * intellectual property right is granted by Renesas.  This software is protected under all applicable laws, including
 * copyright laws. Renesas reserves the right to change or discontinue this software and/or this documentation.
 * THE SOFTWARE AND DOCUMENTATION IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND
 * TO THE FULLEST EXTENT PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY,
 * INCLUDING WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE
 * SOFTWARE OR DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.
 * TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR
 * DOCUMENTATION (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER,
 * INCLUDING, WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY
 * LOST PROFITS, OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "hal_data.h"
#include <unistd.h>
#include <stdio.h>
#include "da7212.h"
#include "da7212_data.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
//#define DEBUG_A3M_DEV_BOARD
#define AUDIO_EVENT_FLAG_I2C_ABRT   (0x00000001UL)
#define AUDIO_EVENT_FLAG_I2C_RXCP   (0x00000002UL)
#define AUDIO_EVENT_FLAG_I2C_TXCP   (0x00000004UL)
#define AUDIO_EVENT_FLAG_I2C_ERR    (0x00000008UL)
#define AUDIO_EVENT_FLAG_I2C_MASK   (AUDIO_EVENT_FLAG_I2C_ABRT|AUDIO_EVENT_FLAG_I2C_RXCP|AUDIO_EVENT_FLAG_I2C_TXCP|AUDIO_EVENT_FLAG_I2C_ERR)
#define DA7212_I2C_WAIT             (500)

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
static TX_EVENT_FLAGS_GROUP event_flag;

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void da7212_send_data(i2c_master_instance_t *p_i2c, uint8_t addr, uint8_t data);
#ifdef DEBUG_A3M_DEV_BOARD
static void da7212_read_data(i2c_master_instance_t *p_i2c, uint8_t addr);
#endif
static void da7212_setup_data( i2c_master_instance_t *p_i2c );
static void da7212_i2c_callback( i2c_master_callback_args_t *p_arg );
static void da7212_i2c_open(i2c_master_instance_t *p_i2c);
static void da7212_i2c_close(i2c_master_instance_t *p_i2c);
static UINT da7212_i2c_wait_cb(i2c_master_event_t *p_event);


/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static uint8_t da7212_reg_data;
static uint8_t calboffset_data = 0x01;

/***********************************************************************************************************************
 * Global Variables
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * da7212_init
 **********************************************************************************************************************/
void da7212_init( i2c_master_instance_t *p_i2c )
{
    UINT status;

    /* event flag create */
    status = tx_event_flags_create( &event_flag, (CHAR*)"da7212 event flag");
    if (TX_SUCCESS != status)
    {
        printf("[DA7212] tx_event_flags_create is failed(%d).\r\n", status);
        return;
    }

    /* open i2c */
    da7212_i2c_open(p_i2c);

    /* set initialize data */
    da7212_setup_data(p_i2c);

    /* close i2c */
    da7212_i2c_close(p_i2c);
}

static void da7212_setup_data( i2c_master_instance_t *p_i2c )
{
    /* Soft reset */
    da7212_reg_data = 0x00;
    da7212_reg_data = DA7212_SOFT_RESET_VAL;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_CIF_CTRL], da7212_reg_data);

    /* set DIG_ROUTING_DAI */
    da7212_reg_data = DA7212_DIG_ROUTING_DAI_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_DIG_ROUTING_DAI_L_SRC_BIT | DA7212_DIG_ROUTING_DAI_R_SRC_BIT));
    //da7212_reg_data |= (DA7212_DIG_ROUTING_DAI_L_SRC_DAI | DA7212_DIG_ROUTING_DAI_R_SRC_DAI);
    da7212_reg_data |= (DA7212_DIG_ROUTING_DAI_R_SRC_ADC | DA7212_DIG_ROUTING_DAI_L_SRC_ADC);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DIG_ROUTING_DAI], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DIG_ROUTING_DAI]);
#endif

    /* set sample rate */
    da7212_reg_data = DA7212_SR_INI_VAL;
    da7212_reg_data &= (uint8_t)(~DA7212_SR_BIT);
    da7212_reg_data |= DA7212_SR_48kHz;
    //da7212_reg_data |= DA7212_SR_44_1kHz;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_SR], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_SR]);
#endif

    /* set PLL_CTRL */
    da7212_reg_data = DA7212_PLL_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_PLL_CTRL_INDIV_BIT));
    da7212_reg_data |= DA7212_PLL_CTRL_INDIV_20_40;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_PLL_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_PLL_CTRL]);
#endif

    /* set DAI_CLK_MODE */
    da7212_reg_data = DA7212_DAI_CLK_MODE_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_DAI_CLK_MODE_CLK_EN_BIT | DA7212_DAI_CLK_MODE_BCLKS_PER_WCLK_BIT));
    da7212_reg_data |= (DA7212_DAI_CLK_MODE_CLK_EN_ON | DA7212_DAI_CLK_MODE_BCLKS_PER_WCLK_32);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DAI_CLK_MODE], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DAI_CLK_MODE]);
#endif

    /* set DAI_CTRL */
    da7212_reg_data = DA7212_DAI_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_DAI_CTRL_EN_BIT | DA7212_DAI_CTRL_OE_BIT | DA7212_DAI_CTRL_WL_LEN_BIT));
    da7212_reg_data |= (DA7212_DAI_CTRL_EN_ON | DA7212_DAI_CTRL_OE_ON | DA7212_DAI_CTRL_WL_LEN_16);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DAI_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DAI_CTRL]);
#endif

    /* set DIG_ROUTING_DAC */
    da7212_reg_data = DA7212_DIG_ROUTING_DAC_INI_VAL;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DIG_ROUTING_DAC], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DIG_ROUTING_DAC]);
#endif

    /* set DAC_L_GAIN */
    da7212_reg_data = DA7212_DAC_L_GAIN_INI_VAL;
    //da7212_reg_data &= (uint8_t)(~(DA7212_DAC_L_GAIN_BIT));
    //da7212_reg_data |= DA7212_DAC_L_GAIN_P6DB;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_L_GAIN], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_L_GAIN]);
#endif

    /* set DAC_R_GAIN */
    da7212_reg_data = DA7212_DAC_R_GAIN_INI_VAL;
    //da7212_reg_data &= (uint8_t)(~(DA7212_DAC_R_GAIN_BIT));
    //da7212_reg_data |= DA7212_DAC_R_GAIN_P6DB;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_R_GAIN], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_R_GAIN]);
#endif

    /* set HP_L_GAIN */
    da7212_reg_data = DA7212_HP_L_GAIN_INI_VAL;
    //da7212_reg_data &= (uint8_t)(~(DA7212_HP_L_GAIN_BIT));
    //da7212_reg_data |= DA7212_HP_L_GAIN_P6DB;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_L_GAIN], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_L_GAIN]);
#endif

    /* set HP_R_GAIN */
    da7212_reg_data = DA7212_HP_R_GAIN_INI_VAL;
    //da7212_reg_data &= (uint8_t)(~(DA7212_HP_R_GAIN_BIT));
    //da7212_reg_data |= DA7212_HP_R_GAIN_P6DB;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_R_GAIN], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_R_GAIN]);
#endif

    /* set MIXOUT_L_SELECT */
    da7212_reg_data = DA7212_MIXOUT_L_SELECT_INI_VAL;
    da7212_reg_data |= DA7212_MIXOUT_L_SELECT_DAC_L_ON;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXOUT_L_SELECT], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXOUT_L_SELECT]);
#endif

    /* set MIXOUT_R_SELECT */
    da7212_reg_data = DA7212_MIXOUT_R_SELECT_INI_VAL;
    da7212_reg_data |= DA7212_MIXOUT_R_SELECT_DAC_R_ON;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXOUT_R_SELECT], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXOUT_R_SELECT]);
#endif

    /* set SYSTEM_MODES_OUTPUT */
    da7212_reg_data = DA7212_SYSTEM_MODES_OUTPUT_INI_VAL;
    da7212_reg_data |= (DA7212_SYSTEM_MODES_OUTPUT_DAC_R_ON | DA7212_SYSTEM_MODES_OUTPUT_DAC_L_ON | DA7212_SYSTEM_MODES_OUTPUT_HP_R_ON
                      | DA7212_SYSTEM_MODES_OUTPUT_HP_L_ON | DA7212_SYSTEM_MODES_OUTPUT_LINE_ON | DA7212_SYSTEM_MODES_OUTPUT_MD_SUB_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_SYSTEM_MODES_OUTPUT], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_SYSTEM_MODES_OUTPUT]);
#endif

    /* set DAC_L_CTRL */
    da7212_reg_data = DA7212_DAC_L_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_DAC_L_CTRL_EN_BIT | DA7212_DAC_L_CTRL_MUTE_BIT | DA7212_DAC_L_CTRL_RMP_EN_BIT));
    da7212_reg_data |= (DA7212_DAC_L_CTRL_EN_ON | DA7212_DAC_L_CTRL_MUTE_OFF | DA7212_DAC_L_CTRL_RMP_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_L_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_L_CTRL]);
#endif

    /* set DAC_R_CTRL */
    da7212_reg_data = DA7212_DAC_R_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_DAC_R_CTRL_EN_BIT | DA7212_DAC_R_CTRL_MUTE_BIT | DA7212_DAC_R_CTRL_RMP_EN_BIT));
    da7212_reg_data |= (DA7212_DAC_R_CTRL_EN_ON | DA7212_DAC_R_CTRL_MUTE_OFF | DA7212_DAC_R_CTRL_RMP_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_R_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_DAC_R_CTRL]);
#endif

    /* set CP_CTRL */
    da7212_reg_data = DA7212_CP_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_CP_CTRL_EN_BIT));
    da7212_reg_data |= (DA7212_CP_CTRL_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_CP_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_CP_CTRL]);
#endif

    /* set HP_L_CTRL */
    da7212_reg_data = DA7212_HP_L_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_HP_L_CTRL_AMP_EN_BIT | DA7212_HP_L_CTRL_AMP_MUTE_BIT | DA7212_HP_L_CTRL_AMP_RMP_EN_BIT
                                 | DA7212_HP_L_CTRL_AMP_AMP_OE_BIT ));
    da7212_reg_data |= (DA7212_HP_L_CTRL_AMP_EN_ON | DA7212_HP_L_CTRL_AMP_MUTE_OFF | DA7212_HP_L_CTRL_AMP_AMP_OE_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_L_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_L_CTRL]);
#endif

    /* set HP_R_CTRL */
    da7212_reg_data = DA7212_HP_R_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_HP_R_CTRL_AMP_EN_BIT | DA7212_HP_R_CTRL_AMP_MUTE_BIT | DA7212_HP_R_CTRL_AMP_RMP_EN_BIT
                                 | DA7212_HP_R_CTRL_AMP_AMP_OE_BIT ));
    da7212_reg_data |= (DA7212_HP_R_CTRL_AMP_EN_ON | DA7212_HP_R_CTRL_AMP_MUTE_OFF | DA7212_HP_R_CTRL_AMP_AMP_OE_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_R_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_R_CTRL]);
#endif

    /* set LINE_CTRL */
    da7212_reg_data = DA7212_LINE_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_LINE_CTRL_AMP_EN_BIT | DA7212_LINE_CTRL_AMP_MUTE_BIT | DA7212_LINE_CTRL_AMP_RMP_EN_BIT
                                 | DA7212_LINE_CTRL_AMP_AMP_OE_BIT | DA7212_LINE_CTRL_AMP_MIN_GAIN_EN_BIT ));
    da7212_reg_data |= (DA7212_LINE_CTRL_AMP_EN_ON | DA7212_LINE_CTRL_AMP_MUTE_OFF | DA7212_LINE_CTRL_AMP_AMP_OE_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_LINE_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_HP_R_CTRL]);
#endif

    /* set IO_CTRL */
    da7212_reg_data = DA7212_TONE_GEN_CYCLES_INI_VAL;
    da7212_reg_data |= DA7212_IO_CTRL_IO_VOL_LVL_HIGH;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_IO_CTRL], da7212_reg_data);

#ifdef DEBUG_A3M_DEV_BOARD
    /* set TONE_GEN_CFG1 */
    da7212_reg_data = DA7212_TONE_GEN_CFG1_INI_VAL;
    da7212_reg_data |= (DA7212_TONE_GEN_CFG1_STRT_STP_ON | DA7212_TONE_GEN_CFG1_DMTF_EN_ON | DA7212_TONE_GEN_CFG1_DMTF_REG_VAL);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_TONE_GEN_CFG1], da7212_reg_data);
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_TONE_GEN_CFG1]);

    /* set TONE_GEN_CYCLES */
    da7212_reg_data = DA7212_TONE_GEN_CYCLES_INI_VAL;
    da7212_reg_data |= DA7212_TONE_GEN_CYCLES_BEEP_CYCLES_VAL;
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_TONE_GEN_CYCLES], da7212_reg_data);
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_TONE_GEN_CYCLES]);
#endif

#if 1
    //Input set up
    // set REFERENCES
    da7212_reg_data = DA7212_REFERENCES_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_REFERENCES_BIAS_EN_BIT));
    da7212_reg_data |= (DA7212_REFERENCES_BIAS_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_REFERENCES], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_REFERENCES]);
#endif

    // set MICBIAS_CTRL
    da7212_reg_data = DA7212_MICBIAS_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_MICBIAS_CTRL_MICBIAS1_EN_BIT));
    da7212_reg_data |= (DA7212_MICBIAS_CTRL_MICBIAS1_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MICBIAS_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MICBIAS_CTRL]);
#endif

    // set MIC_1_CTRL
    da7212_reg_data = DA7212_MIC_1_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_MIC_1_CTRL_AMP_EN_BIT | DA7212_MIC_1_CTRL_AMP_MUTE_EN_BIT | DA7212_MIC_1_CTRL_AMP_IN_SEL_BIT));
    da7212_reg_data |= (DA7212_MIC_1_CTRL_AMP_EN_ON | DA7212_MIC_1_CTRL_AMP_MUTE_EN_OFF | DA7212_MIC_1_CTRL_AMP_IN_SEL_PEND);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIC_1_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
        da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIC_1_CTRL]);
#endif
    // set MIC_1_CTRL
    da7212_reg_data = DA7212_MIC_1_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_MIC_1_CTRL_AMP_EN_BIT | DA7212_MIC_1_CTRL_AMP_MUTE_EN_BIT | DA7212_MIC_1_CTRL_AMP_IN_SEL_BIT));
    da7212_reg_data |= (DA7212_MIC_1_CTRL_AMP_EN_OFF | DA7212_MIC_1_CTRL_AMP_MUTE_EN_ON | DA7212_MIC_1_CTRL_AMP_IN_SEL_PEND);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIC_1_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIC_1_CTRL]);
#endif
    // set MIXIN_L_CTRL
    da7212_reg_data = DA7212_MIXIN_L_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_MIXIN_L_CTRL_AMP_EN_BIT | DA7212_MIXIN_L_CTRL_AMP_MUTE_EN_BIT | DA7212_MIXIN_L_CTRL_AMP_RMP_EN_BIT | DA7212_MIXIN_L_CTRL_MIX_EN_BIT));
    da7212_reg_data |= (DA7212_MIXIN_L_CTRL_AMP_EN_ON | DA7212_MIXIN_L_CTRL_AMP_MUTE_EN_OFF | DA7212_MIXIN_L_CTRL_AMP_RMP_EN_ON | DA7212_MIXIN_L_CTRL_MIX_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXIN_L_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXIN_L_CTRL]);
#endif
    // set MIXIN_L_SELECT
    da7212_reg_data = DA7212_MIXIN_L_SELECT_INI_VAL;
    da7212_reg_data |= (DA7212_MIXIN_L_SELECT_MIC1_SEL_MIX);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXIN_L_SELECT], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIXIN_L_SELECT]);
#endif
    // set ADC_L_CTRL
    da7212_reg_data = DA7212_ADC_L_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_ADC_L_CTRL_EN_BIT | DA7212_ADC_L_CTRL_MUTE_BIT | DA7212_ADC_L_CTRL_RMP_EN_BIT));
    da7212_reg_data |= (DA7212_ADC_L_CTRL_EN_ON | DA7212_ADC_L_CTRL_MUTE_ON | DA7212_ADC_L_CTRL_RMP_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_ADC_L_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_ADC_L_CTRL]);
#endif
    // set ALC_CTRL1
    da7212_reg_data = DA7212_ALC_CTRL1_INI_VAL;
    da7212_reg_data |= (DA7212_ALC_CTRL1_AUTO_CALIB_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_ALC_CTRL1], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_ALC_CTRL1]);
#endif
    while(calboffset_data)
    {
        uint8_t send_data[1];
        //uint8_t read_data[1];
        fsp_err_t err;
        UINT status;
        i2c_master_event_t cb_event;

        send_data[0] = da7212_reg_addr[DA7212_REG_ALC_CTRL1];

        // Access register
        err = p_i2c->p_api->write(p_i2c->p_ctrl, send_data, sizeof(send_data), true);
        if(FSP_SUCCESS != err)
        {
            printf("[DA7212] I2C write failed.(%d)\r\n", err);
        }

        status = da7212_i2c_wait_cb( &cb_event );
        if(status != TX_SUCCESS)
        {
            printf("da7212_i2c_wait_cb failed (%d)\r\n", status);
        }
        else
        {
            switch(cb_event)
            {
            case I2C_MASTER_EVENT_RX_COMPLETE:
            case I2C_MASTER_EVENT_TX_COMPLETE:
                break;
            case I2C_MASTER_EVENT_ABORTED:
            default:
            printf("[W]I2C_MASTER_EVENT_ABORTED or UNKNOWN (%d)\r\n", cb_event);
                return;
                break;
            }
        }

        // Read data
        err = p_i2c->p_api->read(p_i2c->p_ctrl, &calboffset_data, sizeof(calboffset_data), false);
        if(FSP_SUCCESS != err)
        {
            printf("[DA7212] I2C read failed.(%d)\r\n", err);
        }
        status = da7212_i2c_wait_cb( &cb_event );
        if(status != TX_SUCCESS)
        {
            printf("da7212_i2c_wait_cb failed (%d)\r\n", status);
        }
        else
        {
            switch(cb_event)
            {
            case I2C_MASTER_EVENT_RX_COMPLETE:
            case I2C_MASTER_EVENT_TX_COMPLETE:
                printf("Read data is (%#x)\r\n", calboffset_data);
                break;
            case I2C_MASTER_EVENT_ABORTED:
            default:
                printf("[R]I2C_MASTER_EVENT_ABORTED or UNKNOWN (%d)\r\n", cb_event);
                return;
                break;
            }
        }
    }
    // set ALC_CTRL1
    da7212_reg_data = DA7212_ALC_CTRL1_INI_VAL;
    da7212_reg_data |= (DA7212_ALC_CTRL1_R_EN_ON | DA7212_ALC_CTRL1_L_EN_ON | DA7212_ALC_CTRL1_SYNC_MODE_ON | DA7212_ALC_CTRL1_OFST_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_ALC_CTRL1], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_ALC_CTRL1]);
#endif
    // set MIC_1_CTRL
    da7212_reg_data = DA7212_MIC_1_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_MIC_1_CTRL_AMP_EN_BIT | DA7212_MIC_1_CTRL_AMP_MUTE_EN_BIT | DA7212_MIC_1_CTRL_AMP_IN_SEL_BIT));
    da7212_reg_data |= (DA7212_MIC_1_CTRL_AMP_EN_ON | DA7212_MIC_1_CTRL_AMP_MUTE_EN_OFF | DA7212_MIC_1_CTRL_AMP_IN_SEL_PEND);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_MIC_1_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_MIC_1_CTRL]);
#endif
    // set ADC_L_CTRL
    da7212_reg_data = DA7212_ADC_L_CTRL_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_ADC_L_CTRL_EN_BIT | DA7212_ADC_L_CTRL_MUTE_BIT | DA7212_ADC_L_CTRL_RMP_EN_BIT));
    da7212_reg_data |= (DA7212_ADC_L_CTRL_EN_ON | DA7212_ADC_L_CTRL_MUTE_OFF | DA7212_ADC_L_CTRL_RMP_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_ADC_L_CTRL], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_ADC_L_CTRL]);
#endif
    // set SYSTEM_MODES_INPUT
    da7212_reg_data = DA7212_SYSTEM_MODES_INPUT_INI_VAL;
    da7212_reg_data |= ( DA7212_SYSTEM_MODES_INPUT_MIXIN_L_ON | DA7212_SYSTEM_MODES_INPUT_MIC_1_ON | DA7212_SYSTEM_MODES_INPUT_ADC_L_ON
    		           | DA7212_SYSTEM_MODES_INPUT_MD_SUB_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_SYSTEM_MODES_INPUT], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_SYSTEM_MODES_INPUT]);
#endif
    // set ADC_FILTERS1
    da7212_reg_data = DA7212_ADC_FILTERS1_INI_VAL;
    da7212_reg_data &= (uint8_t)(~(DA7212_ADC_FILTERS1_HPF_EN_BIT | DA7212_ADC_FILTERS1_VC_EN_BIT));
    da7212_reg_data |= (DA7212_ADC_FILTERS1_HPF_EN_OFF | DA7212_ADC_FILTERS1_VC_EN_ON);
    da7212_send_data(p_i2c, da7212_reg_addr[DA7212_REG_ADC_FILTERS1], da7212_reg_data);
#ifdef DEBUG_A3M_DEV_BOARD
    da7212_read_data(p_i2c, da7212_reg_addr[DA7212_REG_ADC_FILTERS1]);
#endif

    #endif
}

static void da7212_send_data(i2c_master_instance_t *p_i2c, uint8_t addr, uint8_t data)
{
    uint8_t send_data[2];
    fsp_err_t err;
    UINT status;
    i2c_master_event_t cb_event;

    send_data[0] = addr;
    send_data[1] = data;

    //p_i2c->p_api->slaveAddressSet(p_i2c->p_ctrl, DA7212_ADDR, I2C_MASTER_ADDR_MODE_7BIT);
    err = p_i2c->p_api->write(p_i2c->p_ctrl, send_data, sizeof(send_data), false);
    if(FSP_SUCCESS != err)
    {
        printf("[DA7212] I2C write failed.(%d)\r\n", err);
    }

    status = da7212_i2c_wait_cb( &cb_event );
    if(status != TX_SUCCESS)
    {
        printf("da7212_i2c_wait_cb failed (%d)\r\n", status);
    }
    else
    {
        switch(cb_event)
        {
        case I2C_MASTER_EVENT_RX_COMPLETE:
        case I2C_MASTER_EVENT_TX_COMPLETE:
            break;
        case I2C_MASTER_EVENT_ABORTED:
        default:
        printf("[W]I2C_MASTER_EVENT_ABORTED or UNKNOWN (%d)\r\n", cb_event);
            return;
            break;
        }
    }
}

#ifdef DEBUG_A3M_DEV_BOARD
static void da7212_read_data(i2c_master_instance_t *p_i2c, uint8_t addr)
{
    uint8_t send_data[1];
    uint8_t read_data[1];
    fsp_err_t err;
    UINT status;
    i2c_master_event_t cb_event;

    send_data[0] = addr;

    // Access register
    //p_i2c->p_api->slaveAddressSet(p_i2c->p_ctrl, DA7212_ADDR, I2C_MASTER_ADDR_MODE_7BIT);
    err = p_i2c->p_api->write(p_i2c->p_ctrl, send_data, sizeof(send_data), true);
    if(FSP_SUCCESS != err)
    {
        printf("[DA7212] I2C write failed.(%d)\r\n", err);
    }

    status = da7212_i2c_wait_cb( &cb_event );
    if(status != TX_SUCCESS)
    {
        printf("da7212_i2c_wait_cb failed (%d)\r\n", status);
    }
    else
    {
        switch(cb_event)
        {
        case I2C_MASTER_EVENT_RX_COMPLETE:
        case I2C_MASTER_EVENT_TX_COMPLETE:
            break;
        case I2C_MASTER_EVENT_ABORTED:
        default:
        printf("[W]I2C_MASTER_EVENT_ABORTED or UNKNOWN (%d)\r\n", cb_event);
            return;
            break;
        }
    }

    // Read data
    err = p_i2c->p_api->read(p_i2c->p_ctrl, read_data, sizeof(send_data), false);
    if(FSP_SUCCESS != err)
    {
        printf("[DA7212] I2C read failed.(%d)\r\n", err);
    }
    status = da7212_i2c_wait_cb( &cb_event );
    if(status != TX_SUCCESS)
    {
        printf("da7212_i2c_wait_cb failed (%d)\r\n", status);
    }
    else
    {
        switch(cb_event)
        {
        case I2C_MASTER_EVENT_RX_COMPLETE:
        case I2C_MASTER_EVENT_TX_COMPLETE:
            printf("Read data is (%#x)\r\n", read_data[0]);
            break;
        case I2C_MASTER_EVENT_ABORTED:
        default:
        printf("[R]I2C_MASTER_EVENT_ABORTED or UNKNOWN (%d)\r\n", cb_event);
            return;
            break;
        }
    }
}
#endif

static void da7212_i2c_callback( i2c_master_callback_args_t *p_arg )
{
    FSP_PARAMETER_NOT_USED(p_arg);

    switch(p_arg->event)
    {
    case I2C_MASTER_EVENT_ABORTED:
        tx_event_flags_set(&event_flag, (ULONG)AUDIO_EVENT_FLAG_I2C_ABRT, TX_OR );
        break;
    case I2C_MASTER_EVENT_RX_COMPLETE:
        tx_event_flags_set(&event_flag, (ULONG)AUDIO_EVENT_FLAG_I2C_RXCP, TX_OR );
        break;
    case I2C_MASTER_EVENT_TX_COMPLETE:
        tx_event_flags_set(&event_flag, (ULONG)AUDIO_EVENT_FLAG_I2C_TXCP, TX_OR );
        break;
    default:
        tx_event_flags_set(&event_flag, (ULONG)AUDIO_EVENT_FLAG_I2C_ERR, TX_OR );
        break;
    }
}

static void da7212_i2c_open(i2c_master_instance_t *p_i2c)
{
    fsp_err_t err;

    /* open i2c */
    err = p_i2c->p_api->open(p_i2c->p_ctrl, p_i2c->p_cfg);
    if( FSP_SUCCESS != err )
    {
        printf("[DA7212] I2C open failed.(%d)\r\n", err);
        return;
    }

    /* set callback */
    err = p_i2c->p_api->callbackSet(p_i2c->p_ctrl, da7212_i2c_callback, NULL, NULL);
    if( FSP_SUCCESS != err )
    {
        printf("[DA7212] I2C callbackSet failed.(%d)\r\n", err);
        return;
    }

    err = p_i2c->p_api->slaveAddressSet( p_i2c->p_ctrl, 0x1A, I2C_MASTER_ADDR_MODE_7BIT );
    if( FSP_SUCCESS != err )
    {
        printf("[DA7212] I2C slaveAddressSet failed.(%d)\r\n", err);
    }
}

static void da7212_i2c_close(i2c_master_instance_t *p_i2c)
{
    fsp_err_t err;

    // I2C Close
    err = p_i2c->p_api->close(p_i2c->p_ctrl);
    if(FSP_SUCCESS != err)
    {
        printf("[DA7212] I2C close failed.(%d)\r\n", err);
    }
}
static UINT da7212_i2c_wait_cb( i2c_master_event_t *p_event )
{
    UINT status;
    ULONG statusbit;

    status = tx_event_flags_get( &event_flag, AUDIO_EVENT_FLAG_I2C_MASK, TX_OR_CLEAR, &statusbit, DA7212_I2C_WAIT );
    if( TX_SUCCESS == status )
    {
        if( (statusbit & AUDIO_EVENT_FLAG_I2C_ABRT) == AUDIO_EVENT_FLAG_I2C_ABRT )
        {
            *p_event = I2C_MASTER_EVENT_ABORTED;
        }
        else if( (statusbit & AUDIO_EVENT_FLAG_I2C_RXCP) == AUDIO_EVENT_FLAG_I2C_RXCP )
        {
            *p_event = I2C_MASTER_EVENT_RX_COMPLETE;
        }
        else if( (statusbit & AUDIO_EVENT_FLAG_I2C_TXCP) == AUDIO_EVENT_FLAG_I2C_TXCP )
        {
            *p_event = I2C_MASTER_EVENT_TX_COMPLETE;
        }
        else if( (statusbit & AUDIO_EVENT_FLAG_I2C_ERR) == AUDIO_EVENT_FLAG_I2C_ERR )
        {
            status = TX_NOT_AVAILABLE;
        }
        else
        {
            status = TX_NOT_AVAILABLE;
        }
        return status;
    }
    else
    {
        printf("[DA7212] tx_event_flags_get failed(%X)\r\n", status);
        return TX_WAIT_ERROR;
    }
}
