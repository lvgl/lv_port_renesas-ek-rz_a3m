/***********************************************************************************************************************
 * Copyright [2020-2022] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
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

#ifndef DEVICE_ILI9881_ILI9881_H_
#define DEVICE_ILI9881_ILI9881_H_

#include <r_mipi_dsi_b.h>
#include "bsp_api.h"

#define COMMAND_SEND_DATA(_op, _data0, _data1)  \
        {\
    .operation = (_op),\
    .data0 = (_data0),\
    .data1 = (_data1),\
        },


typedef struct command_type
{
    uint8_t operation;  /* 0    :change page to data0 page
                         * 1    :short packet without parameter
                         * 2    :short packet with parameter
                         * 0x10 :Delay "data0" mili seconds wait
                         * 0xFF :terminate command flow
                         * */
    uint8_t data0;
    uint8_t data1;
}command_type_t;

extern command_type_t command_flow_ili9881[];

void ili9881_init(mipi_dsi_ctrl_t * const p_api_ctrl);
void ili9881_changepage(mipi_dsi_ctrl_t * const p_api_ctrl, uint8_t page);
void ili9881_cmd_send(mipi_dsi_ctrl_t * const p_api_ctrl, command_type_t * command_flow);

#endif /* DEVICE_ILI9881_ILI9881_H_ */
