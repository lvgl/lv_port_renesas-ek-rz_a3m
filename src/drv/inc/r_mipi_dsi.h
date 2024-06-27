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


#ifndef R_MIPI_DSI_H_
#define R_MIPI_DSI_H_

#include "hal_data.h"

#define BSP_MSTP_REG_FSP_IP_MIPI_DSI(channel) (R_MSTP->PERI_VIDEO)
#define BSP_MSTP_BIT_FSP_IP_MIPI_DSI(channel) (3U << 5)
#define R_BSP_MODULE_START_FSP_IP_MIPI_DSI(ip, ch) {FSP_CRITICAL_SECTION_DEFINE; \
 FSP_CRITICAL_SECTION_ENTER; \
 BSP_MSTP_REG_ ## ip(channel) = 0x00000000U \
 | (BSP_MSTP_BIT_ ## ip(channel) << \
 16U); \
 BSP_MSTP_REG_ ## ip(channel); \
 FSP_CRITICAL_SECTION_EXIT;}

/** Display signal timing setting */
typedef struct st_mipi_dsi_timing
{
    uint16_t total_cyc;                      ///< Total cycles in one line or total lines in one frame
    uint16_t display_cyc;                    ///< Active video cycles or lines
    uint16_t back_porch;                     ///< Back porch cycles or lines
    uint16_t sync_width;                     ///< Sync signal asserting width
    display_signal_polarity_t sync_polarity; ///< Sync signal polarity
} mipi_dsi_timing_t;

typedef struct st_mipi_dsi_timing_cfg
{
    uint32_t    t_init;
    uint8_t     tclk_prepare;
    uint8_t     ths_prepare;
    uint8_t     tclk_zero;
    uint8_t     tclk_pre;
    uint8_t     tclk_post;
    uint8_t     tclk_trail;
    uint8_t     ths_zero;
    uint8_t     ths_trail;
    uint8_t     ths_exit;
    uint8_t     tlpx;
}mipi_dsi_timing_cfg_t;

typedef struct st_mipi_dsi_longpacket_data
{
    uint8_t     virtual_channel;
    uint8_t     data_type;
    uint8_t *  send_data_addr;
    uint8_t     word_count;
}mipi_dsi_longpacket_data_t;

typedef struct st_mipi_dsi_cfg
{
    uint32_t bps;
}mipi_dsi_cfg_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_mipi_dsi_instance
{
    mipi_dsi_cfg_t const * p_cfg;       ///< Pointer to the configuration structure for this instance
} mipi_dsi_instance_t;

fsp_err_t mipi_dsi_init(mipi_dsi_cfg_t const * const p_cfg, display_cfg_t const * const cfg);
void mipi_dsi_cmd_send_short(uint8_t vc_dt, uint8_t data0, uint8_t data1);
void mipi_dsi_cmd_reception(uint8_t vc_dt, uint8_t data0, uint8_t data1);
void mipi_dsi_cmd_send_long(mipi_dsi_longpacket_data_t const * const p_cfg);

#endif /* R_MIPI_DSI_H_ */
