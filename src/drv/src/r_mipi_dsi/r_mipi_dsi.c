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

#include "r_mipi_dsi.h"
#include "hal_data.h"

/**
 *  Register value for SQCH0DSC0AR(31-24 bit).
 *  NXACT : Terminate Operation after this descriptor finished
 *  FMT : 0:short packet 1:long packet
 *  SPD : 0:HS 1:LP
 *  BTA : Bus turn around
 *  */
/* Command Send Short packet Register value */
#define CSS_NXACT    (0)
#define CSS_FMT      (0)
#define CSS_SPD      (1)
#define CSS_BTA      (0)
#define CSS_SQCH0DSC0AR  (CSS_NXACT << 4 | CSS_BTA << 2 | CSS_SPD << 1 | CSS_FMT)

/* Command Reception Register value */
#define CR_NXACT    (0)
#define CR_FMT      (0)
#define CR_SPD      (1)
#define CR_BTA      (2)
#define CR_SQCH0DSC0AR  (CR_NXACT << 4 | CR_BTA << 2 | CR_SPD << 1 | CR_FMT)

/* Command Send Long packet value */
#define CLS_NXACT    (0)
#define CLS_FMT      (1)
#define CLS_SPD      (1)
#define CLS_BTA      (0)
#define CLS_SQCH0DSC0AR  (CLS_NXACT << 4 | CLS_BTA << 2 | CLS_SPD << 1 | CLS_FMT)


static mipi_dsi_timing_cfg_t mipi_dsi_table[] =
{
 {
  /* 0:80Mbps */
  .t_init    = 0x137B9,
  .tclk_prepare = 0x8,
  .ths_prepare  = 0xD,
  .tclk_zero     = 0x21,
  .tclk_pre      = 0x18,
  .tclk_post  = 0x5E,
  .tclk_trail  = 0xA,
  .ths_zero  = 0x17,
  .ths_trail = 0x11,
  .ths_exit  = 0xD,
  .tlpx      = 0x6,
 },
 {
  /* 1:125Mbps */
  .t_init    = 0x137B9,
  .tclk_prepare = 0x8,
  .ths_prepare  = 0xC,
  .tclk_zero     = 0x21,
  .tclk_pre      = 0xF,
  .tclk_post  = 0x5E,
  .tclk_trail  = 0xA,
  .ths_zero  = 0x17,
  .ths_trail = 0x11,
  .ths_exit  = 0xD,
  .tlpx      = 0x6,
 },
 {
  /* 2:250Mbps */
  .t_init    = 0x137B9,
  .tclk_prepare = 0x8,
  .ths_prepare  = 0xC,
  .tclk_zero     = 0x21,
  .tclk_pre      = 0xD,
  .tclk_post  = 0x5E,
  .tclk_trail  = 0xA,
  .ths_zero  = 0x17,
  .ths_trail = 0x10,
  .ths_exit  = 0xD,
  .tlpx      = 0x6,
 },
 {
  /* 3:360Mbps */
  .t_init    = 0x137B9,
  .tclk_prepare = 0x8,
  .ths_prepare  = 0xA,
  .tclk_zero     = 0x21,
  .tclk_pre      = 0x4,
  .tclk_post  = 0x23,
  .tclk_trail  = 0x7,
  .ths_zero  = 0x10,
  .ths_trail = 0x9,
  .ths_exit  = 0xD,
  .tlpx      = 0x6,
 },
 {
  /* 4:720Mbps */
  .t_init    = 0x137B9,
  .tclk_prepare = 0x8,
  .ths_prepare  = 0x9,
  .tclk_zero     = 0x21,
  .tclk_pre      = 0x4,
  .tclk_post  = 0x23,
  .tclk_trail  = 0x7,
  .ths_zero  = 0x10,
  .ths_trail = 0x9,
  .ths_exit  = 0xD,
  .tlpx      = 0x6,
 },
 {
  /* 5:Over 720Mbps */
  .t_init    = 0x137B9,
  .tclk_prepare = 0x8,
  .ths_prepare  = 0x9,
  .tclk_zero     = 0x21,
  .tclk_pre      = 0x4,
  .tclk_post  = 0x23,
  .tclk_trail  = 0x7,
  .ths_zero  = 0x10,
  .ths_trail = 0x9,
  .ths_exit  = 0xD,
  .tlpx      = 0x6,
 }
};

/*******************************************************************************************************************//**
*  @brief       Initializes the MIPI-DSI modules
*  @param[in]   None
*  @retval      FSP_SUCCESS          Initializes the MIPI-DSI module
 **********************************************************************************************************************/
fsp_err_t mipi_dsi_init(mipi_dsi_cfg_t const * const p_cfg_mipi_dsi, display_cfg_t const * const p_cfg_display)
{
    fsp_err_t err = FSP_SUCCESS;

    static uint32_t h_fp;
    static uint32_t h_bp;
    static uint32_t h_active;
    static uint32_t h_sa;
    static uint32_t h_polarity;
    static uint32_t v_fp;
    static uint32_t v_bp;
    static uint32_t v_active;
    static uint32_t v_sa;
    static uint32_t v_polarity;

    /* power on reset and initial setting for all operations */
    R_BSP_MODULE_RSTON(FSP_IP_MIPI_DSI, 0);
    R_BSP_MODULE_CLKON(FSP_IP_MIPI_DSI, 0);

    R_BSP_MODULE_START(FSP_IP_MIPI_DSI, 0);

    R_BSP_MODULE_RSTOFF(FSP_IP_MIPI_DSI, 0);

    R_MIPI_DSI->DSIDPHYCTRL0_b.RE_VDD_DETVCCQLV18 = 0b1;
    R_MIPI_DSI->DSIDPHYCTRL0_b.EN_BGR = 0b1;
    R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MICROSECONDS);

    R_MIPI_DSI->DSIDPHYCTRL0_b.EN_LDO1200 = 0b1;
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MICROSECONDS);

    R_MIPI_DSI->DSIDPHYCTRL0_b.CAL_EN_HSRX_OFS = 0b1;

    /* Set Timing*/
    if(80 >= p_cfg_mipi_dsi->bps)
    {
        R_MIPI_DSI->DSIDPHYTIM0 = mipi_dsi_table[0].t_init;
        R_MIPI_DSI->DSIDPHYTIM1 = (uint32_t)((mipi_dsi_table[0].ths_prepare << 24)| (mipi_dsi_table[0].tclk_prepare << 16));
        R_MIPI_DSI->DSIDPHYTIM2 = (uint32_t)((mipi_dsi_table[0].tclk_trail << 24) | (mipi_dsi_table[0].tclk_post << 16) | (mipi_dsi_table[0].tclk_pre << 8) | (mipi_dsi_table[0].tclk_zero));
        R_MIPI_DSI->DSIDPHYTIM3 = (uint32_t)((mipi_dsi_table[0].tlpx << 24) | (mipi_dsi_table[0].ths_exit << 16) | (mipi_dsi_table[0].ths_trail << 8) | (mipi_dsi_table[0].ths_zero));
    }
    else if(80 < p_cfg_mipi_dsi->bps && 125 >= p_cfg_mipi_dsi->bps)
    {
        R_MIPI_DSI->DSIDPHYTIM0 = mipi_dsi_table[1].t_init;
        R_MIPI_DSI->DSIDPHYTIM1 = (uint32_t)((mipi_dsi_table[1].ths_prepare << 24)| (mipi_dsi_table[1].tclk_prepare << 16));
        R_MIPI_DSI->DSIDPHYTIM2 = (uint32_t)((mipi_dsi_table[1].tclk_trail << 24) | (mipi_dsi_table[1].tclk_post << 16) | (mipi_dsi_table[1].tclk_pre << 8) | (mipi_dsi_table[1].tclk_zero));
        R_MIPI_DSI->DSIDPHYTIM3 = (uint32_t)((mipi_dsi_table[1].tlpx << 24) | (mipi_dsi_table[1].ths_exit << 16) | (mipi_dsi_table[1].ths_trail << 8) | (mipi_dsi_table[1].ths_zero));
    }
    else if(125 < p_cfg_mipi_dsi->bps && 250 >= p_cfg_mipi_dsi->bps)
    {
        R_MIPI_DSI->DSIDPHYTIM0 = mipi_dsi_table[2].t_init;
        R_MIPI_DSI->DSIDPHYTIM1 = (uint32_t)((mipi_dsi_table[2].ths_prepare << 24)| (mipi_dsi_table[2].tclk_prepare << 16));
        R_MIPI_DSI->DSIDPHYTIM2 = (uint32_t)((mipi_dsi_table[2].tclk_trail << 24) | (mipi_dsi_table[2].tclk_post << 16) | (mipi_dsi_table[2].tclk_pre << 8) | (mipi_dsi_table[2].tclk_zero));
        R_MIPI_DSI->DSIDPHYTIM3 = (uint32_t)((mipi_dsi_table[2].tlpx << 24) | (mipi_dsi_table[2].ths_exit << 16) | (mipi_dsi_table[2].ths_trail << 8) | (mipi_dsi_table[2].ths_zero));
    }
    else if(250 < p_cfg_mipi_dsi->bps && 360 >= p_cfg_mipi_dsi->bps)
    {
        R_MIPI_DSI->DSIDPHYTIM0 = mipi_dsi_table[3].t_init;
        R_MIPI_DSI->DSIDPHYTIM1 = (uint32_t)((mipi_dsi_table[3].ths_prepare << 24)| (mipi_dsi_table[3].tclk_prepare << 16));
        R_MIPI_DSI->DSIDPHYTIM2 = (uint32_t)((mipi_dsi_table[3].tclk_trail << 24) | (mipi_dsi_table[3].tclk_post << 16) | (mipi_dsi_table[3].tclk_pre << 8) | (mipi_dsi_table[3].tclk_zero));
        R_MIPI_DSI->DSIDPHYTIM3 = (uint32_t)((mipi_dsi_table[3].tlpx << 24) | (mipi_dsi_table[3].ths_exit << 16) | (mipi_dsi_table[3].ths_trail << 8) | (mipi_dsi_table[3].ths_zero));
    }
    else if(360 < p_cfg_mipi_dsi->bps && 720 >= p_cfg_mipi_dsi->bps)
    {
        R_MIPI_DSI->DSIDPHYTIM0 = mipi_dsi_table[1].t_init;
        R_MIPI_DSI->DSIDPHYTIM1 = (uint32_t)((mipi_dsi_table[4].ths_prepare << 24)| (mipi_dsi_table[4].tclk_prepare << 16));
        R_MIPI_DSI->DSIDPHYTIM2 = (uint32_t)((mipi_dsi_table[4].tclk_trail << 24) | (mipi_dsi_table[4].tclk_post << 16) | (mipi_dsi_table[4].tclk_pre << 8) | (mipi_dsi_table[4].tclk_zero));
        R_MIPI_DSI->DSIDPHYTIM3 = (uint32_t)((mipi_dsi_table[4].tlpx << 24) | (mipi_dsi_table[4].ths_exit << 16) | (mipi_dsi_table[4].ths_trail << 8) | (mipi_dsi_table[4].ths_zero));
    }
    else if(720 < p_cfg_mipi_dsi->bps)
    {
        R_MIPI_DSI->DSIDPHYTIM0 = mipi_dsi_table[1].t_init;
        R_MIPI_DSI->DSIDPHYTIM1 = (uint32_t)((mipi_dsi_table[5].ths_prepare << 24)| (mipi_dsi_table[5].tclk_prepare << 16));
        R_MIPI_DSI->DSIDPHYTIM2 = (uint32_t)((mipi_dsi_table[5].tclk_trail << 24) | (mipi_dsi_table[5].tclk_post << 16) | (mipi_dsi_table[5].tclk_pre << 8) | (mipi_dsi_table[5].tclk_zero));
        R_MIPI_DSI->DSIDPHYTIM3 = (uint32_t)((mipi_dsi_table[5].tlpx << 24) | (mipi_dsi_table[5].ths_exit << 16) | (mipi_dsi_table[5].ths_trail << 8) | (mipi_dsi_table[5].ths_zero));
    }
    else
    {
        /* No operation */
    }

    /* DSI-Tx Module initial settings */
    R_MIPI_DSI->TXSETR = 0x00000303;
    R_MIPI_DSI->DSISETR = 0x80f10001;
    R_MIPI_DSI->CLSTPTSETR = 0xc;
    R_MIPI_DSI->LPTRNSTSETR = 0x4b;

    /* Start of HS CLK */
    R_MIPI_DSI->HSCLKSETR_b.HSCLKMODE = 0x1;
    R_MIPI_DSI->HSCLKSETR_b.HSCLKRUN = 0x1;

    /* Start of HS CLK end */
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

    /* video-input setting */
    R_MIPI_DSI->VICH1PPSETR = 0x003e8000;

    if (p_cfg_display->output.htiming.sync_polarity == DISPLAY_SIGNAL_POLARITY_LOACTIVE)
    {
        /* hsync polarity is Low active */
        h_polarity = 1;
    }
    else
    {
        /* hsync polarity is High active */
        h_polarity = 0;
    }

    if (p_cfg_display->output.vtiming.sync_polarity == DISPLAY_SIGNAL_POLARITY_LOACTIVE)
    {
        /* vsync polarity is Low active */
        v_polarity = 1;
    }
    else
    {
        /* vsync polarity is High active */
        v_polarity = 0;
    }
    h_active = (p_cfg_display->output.htiming.display_cyc);
    h_sa     = (p_cfg_display->output.htiming.sync_width);
    h_bp     = (uint32_t) (p_cfg_display->output.htiming.back_porch - p_cfg_display->output.htiming.sync_width);
    h_fp     =
            (uint32_t) (p_cfg_display->output.htiming.total_cyc - p_cfg_display->output.htiming.back_porch -
                    p_cfg_display->output.htiming.display_cyc);
    v_active = p_cfg_display->output.vtiming.display_cyc;
    v_sa     = p_cfg_display->output.vtiming.sync_width;
    v_bp     = (uint32_t) (p_cfg_display->output.vtiming.back_porch - p_cfg_display->output.vtiming.sync_width);
    v_fp     =
            (uint32_t) (p_cfg_display->output.vtiming.total_cyc - p_cfg_display->output.vtiming.back_porch -
                    p_cfg_display->output.vtiming.display_cyc);

    R_MIPI_DSI->VICH1HSSETR = (h_active << 16) | (h_polarity << 15) | h_sa;
    R_MIPI_DSI->VICH1HPSETR = (h_fp << 16) | h_bp;
    R_MIPI_DSI->VICH1VSSETR = (v_active << 16) | (v_polarity << 15) | v_sa;
    R_MIPI_DSI->VICH1VPSETR = (v_fp << 16) | v_bp;

    R_MIPI_DSI->VICH1SET1R = 0x00000360;
    R_MIPI_DSI->VICH1SET0R = 0x00000700;
    R_MIPI_DSI->VICH1SET0R_b.VSTART = 0b1;
    while(R_MIPI_DSI->VICH1SR_b.VIRDY != 0b1)
    {
        /* No operation */
    }

    return err;
}

void mipi_dsi_cmd_send_short(uint8_t vc_dt, uint8_t data0, uint8_t data1)
{
#if 0
    R_MIPI_DSI->SQCH0DSC00AR = 0x00000000;   //initialize SQCH1DSC00AR
    R_MIPI_DSI->SQCH0DSC00BR = 0x00000000;   //initialize SQCH1DSC00BR
    R_MIPI_DSI->SQCH0DSC00CR = 0x00000000;   //initialize SQCH1DSC00CR
    R_MIPI_DSI->SQCH0DSC00DR = 0x00000000;   //initialize SQCH1DSC00DR
    R_MIPI_DSI->SQCH0DSC00AR_b.NXACT = 0x0;  // Terminate Operation after this descriptor finished
    R_MIPI_DSI->SQCH0DSC00AR_b.FMT  = 0x0;   // 0:short packet 1:long packet
    R_MIPI_DSI->SQCH0DSC00AR_b.SPD  = 0x1;   // 0:HS 1:LP
    R_MIPI_DSI->SQCH0DSC00AR_b.BTA  = 0x0;   // no bus turn around
    R_MIPI_DSI->SQCH0DSC00AR |= (vc_dt << 16); // virtual channel and data type
    R_MIPI_DSI->SQCH0DSC00AR_b.DATA0 = data0;  //data0 of Tx packet header
    R_MIPI_DSI->SQCH0DSC00AR_b.DATA1 = data1;  //data1 of Tx packet header
    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x1;     //sequence operation start
#else
    R_MIPI_DSI->SQCH0DSC00AR = 0x00000000;   //initialize SQCH1DSC00AR
    R_MIPI_DSI->SQCH0DSC00BR = 0x00000000;   //initialize SQCH1DSC00BR
    R_MIPI_DSI->SQCH0DSC00CR = 0x00000000;   //initialize SQCH1DSC00CR
    R_MIPI_DSI->SQCH0DSC00DR = 0x00000000;   //initialize SQCH1DSC00DR

    R_MIPI_DSI->SQCH0DSC00AR = (uint32_t)((CSS_SQCH0DSC0AR << 24) |(vc_dt << 16) | (data1 << 8) | data0);
    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x1;     //sequence operation start
#endif
    while(R_MIPI_DSI->SQCH0SR_b.ADESFIN == 0)
    {
        //wait for descriptor finished
    }
    R_MIPI_DSI->SQCH0SCR_b.ADESFIN = 1;      //clear ADESFIN flag
    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x0;   // stop sequence(no effect??)
}

void mipi_dsi_cmd_reception(uint8_t vc_dt, uint8_t data0, uint8_t data1)
{
#if 0
    R_MIPI_DSI->SQCH0DSC00AR = 0x00000000;   //initialize SQCH1DSC00AR
    R_MIPI_DSI->SQCH0DSC00BR = 0x00000000;   //initialize SQCH1DSC00BR
    R_MIPI_DSI->SQCH0DSC00CR = 0x00000000;   //initialize SQCH1DSC00CR
    R_MIPI_DSI->SQCH0DSC00DR = 0x00000000;   //initialize SQCH1DSC00DR
    R_MIPI_DSI->SQCH0DSC00AR_b.NXACT = 0x0;  // Terminate Operation after this descriptor finished
    R_MIPI_DSI->SQCH0DSC00AR_b.FMT  = 0x0;   // 0:short packet 1:long packet
    R_MIPI_DSI->SQCH0DSC00AR_b.SPD  = 0x1;   // 0:HS 1:LP
    R_MIPI_DSI->SQCH0DSC00AR_b.BTA  = 0x2;   // read request with BTA
    R_MIPI_DSI->SQCH0DSC00AR |= (vc_dt << 16); // virtual channel and data type
    R_MIPI_DSI->SQCH0DSC00AR_b.DATA0 = data0;  //data0 of Tx packet header
    R_MIPI_DSI->SQCH0DSC00AR_b.DATA1 = data1;  //data1 of Tx packet header
    R_MIPI_DSI->SQCH0DSC00BR_b.DTSEL = 0x0;  // 0: Packet Payload Data register 1: Long Packet data use memory space
    R_MIPI_DSI->SQCH0DSC00CR_b.AUXOP = 0x0;
    R_MIPI_DSI->SQCH0DSC00CR_b.ACTCODE = 0x0; //When BTA action, indicate rx result save slot number
#else
    R_MIPI_DSI->SQCH0DSC00AR = 0x00000000;   //initialize SQCH1DSC00AR
    R_MIPI_DSI->SQCH0DSC00BR = 0x00000000;   //initialize SQCH1DSC00BR
    R_MIPI_DSI->SQCH0DSC00CR = 0x00000000;   //initialize SQCH1DSC00CR
    R_MIPI_DSI->SQCH0DSC00DR = 0x00000000;   //initialize SQCH1DSC00DR

    R_MIPI_DSI->SQCH0DSC00AR = (uint32_t)((CR_SQCH0DSC0AR << 24) |(vc_dt << 16) | (data1 << 8) | data0);
#endif

    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x1;   //sequence operation start
    while(R_MIPI_DSI->SQCH0SR_b.ADESFIN == 0)
    {
        //wait for descriptor finished
    }
    R_MIPI_DSI->SQCH0SCR_b.ADESFIN = 1;      //clear ADESFIN flag
    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x0;  // stop sequence(no effect??)
}

void mipi_dsi_cmd_send_long(mipi_dsi_longpacket_data_t const * const p_cfg)
{
    uint8_t vc_dt;
    uint32_t i;
    uint8_t data0;
#if 0
    R_MIPI_DSI->SQCH0DSC00AR = 0x00000000;   //initialize SQCH1DSC00AR
    R_MIPI_DSI->SQCH0DSC00BR = 0x00000000;   //initialize SQCH1DSC00BR
    R_MIPI_DSI->SQCH0DSC00CR = 0x00000000;   //initialize SQCH1DSC00CR
    R_MIPI_DSI->SQCH0DSC00DR = 0x00000000;   //initialize SQCH1DSC00DR
    R_MIPI_DSI->TXPPD0R      = 0x00000000;   //initialize TXPPD0R
    R_MIPI_DSI->TXPPD1R      = 0x00000000;   //initialize TXPPD1R
    R_MIPI_DSI->TXPPD2R      = 0x00000000;   //initialize TXPPD2R
    R_MIPI_DSI->TXPPD3R      = 0x00000000;   //initialize TXPPD3R
    R_MIPI_DSI->SQCH0DSC00AR_b.NXACT = 0x0;  // Terminate Operation after this descriptor finished
    R_MIPI_DSI->SQCH0DSC00AR_b.FMT  = 0x1;   // 0:short packet 1:long packet
    R_MIPI_DSI->SQCH0DSC00AR_b.SPD  = 0x1;   // 0:HS 1:LP
    R_MIPI_DSI->SQCH0DSC00AR_b.BTA  = 0x0;   // no bus turn around
    vc_dt = (uint8_t)(p_cfg->virtual_channel << 6) | (p_cfg->data_type);
    R_MIPI_DSI->SQCH0DSC00AR |= (vc_dt << 16); // virtual channel and data type
    R_MIPI_DSI->SQCH0DSC00AR_b.DATA0 = (p_cfg->word_count & 0xff); // lower 8 bits of the word count
    R_MIPI_DSI->SQCH0DSC00AR_b.DATA1 = 0x00; // upper 8 bits of the word count
    R_MIPI_DSI->SQCH0DSC00BR_b.DTSEL = 0x0;  // 0: Packet Payload Data register 1: Long Packet data use memory space
    R_MIPI_DSI->SQCH0DSC00CR_b.AUXOP = 0x0;  // no auxiliary operation
#endif
    R_MIPI_DSI->SQCH0DSC00AR = 0x00000000;   //initialize SQCH1DSC00AR
    R_MIPI_DSI->SQCH0DSC00BR = 0x00000000;   //initialize SQCH1DSC00BR
    R_MIPI_DSI->SQCH0DSC00CR = 0x00000000;   //initialize SQCH1DSC00CR
    R_MIPI_DSI->SQCH0DSC00DR = 0x00000000;   //initialize SQCH1DSC00DR
    R_MIPI_DSI->TXPPD0R      = 0x00000000;   //initialize TXPPD0R
    R_MIPI_DSI->TXPPD1R      = 0x00000000;   //initialize TXPPD1R
    R_MIPI_DSI->TXPPD2R      = 0x00000000;   //initialize TXPPD2R
    R_MIPI_DSI->TXPPD3R      = 0x00000000;   //initialize TXPPD3R
    vc_dt = (uint8_t)(p_cfg->virtual_channel << 6) | (p_cfg->data_type);
    data0 = (p_cfg->word_count & 0xff);
    R_MIPI_DSI->SQCH0DSC00AR = (uint32_t)((CLS_SQCH0DSC0AR << 24) |(vc_dt << 16) | (0x00 << 8) | data0);


    for(i = 0;i < p_cfg->word_count; i++)
    {
        if(3 >= i)
        {
            R_MIPI_DSI->TXPPD0R |= (uint32_t)(*(p_cfg->send_data_addr + i) << (8 * (i%4)));
        }
        else if(4 <= i && 7 >= i)
        {
            R_MIPI_DSI->TXPPD1R |= (uint32_t)(*(p_cfg->send_data_addr + i) << (8 * (i%4)));
        }
        else if(8 <= i && 11 >= i)
        {
            R_MIPI_DSI->TXPPD2R |= (uint32_t)(*(p_cfg->send_data_addr + i) << (8 * (i%4)));
        }
        else if(12 <= i && 15 >= i)
        {
            R_MIPI_DSI->TXPPD3R |= (uint32_t)(*(p_cfg->send_data_addr + i) << (8 * (i%4)));
        }
    }

    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x1;   //sequence operation start
    while(R_MIPI_DSI->SQCH0SR_b.ADESFIN == 0)
    {
        //wait for descriptor finished
    }
    R_MIPI_DSI->SQCH0SCR_b.ADESFIN = 1;      //clear ADESFIN flag
    R_MIPI_DSI->SQCH0SET0R_b.START  = 0x0;   // stop sequence(no effect??)
}
