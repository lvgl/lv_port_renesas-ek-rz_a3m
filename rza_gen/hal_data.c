/* generated HAL source file - do not edit */
#include "hal_data.h"
#define FSP_NOT_DEFINED (UINT32_MAX)
#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void scif_uart_tx_dmac_callback(scif_uart_instance_ctrl_t const * const p_ctrl);

void g_uart0_tx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    scif_uart_tx_dmac_callback(&g_uart0_ctrl);
}
#endif

#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void scif_uart_rx_dmac_callback(scif_uart_instance_ctrl_t const * const p_ctrl);

void g_uart0_rx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    scif_uart_rx_dmac_callback(&g_uart0_ctrl);
}
#endif
#undef FSP_NOT_DEFINED

scif_uart_instance_ctrl_t g_uart0_ctrl;

scif_baud_setting_t g_uart0_baud_setting = {
/* Baud rate calculated with 0.469% error. */.semr_baudrate_bits_b.abcs = 0,
		.semr_baudrate_bits_b.bgdm = 1, .semr_baudrate_bits_b.cks = 0,
		.brr = 53, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme = false };

/** UART extended configuration for UARTonSCIF HAL driver */
const scif_uart_extended_cfg_t g_uart0_cfg_extend = { .bri_ipl = 24, .bri_irq =
		SCIF1_BRK_IRQn, .clock = SCIF_UART_CLOCK_INT, .noise_cancel =
		SCIF_UART_NOISE_CANCELLATION_DISABLE, .p_baud_setting =
		&g_uart0_baud_setting, .rx_fifo_trigger = SCIF_UART_RX_FIFO_TRIGGER_MAX,
		.rts_fifo_trigger = SCIF_UART_RTS_TRIGGER_14, .uart_mode =
				SCIF_UART_MODE_RS232, .flow_control =
				SCIF_UART_FLOW_CONTROL_NONE, .rs485_setting = { .enable =
				(scif_uart_rs485_enable_t) NULL, .polarity =
				SCIF_UART_RS485_DE_POLARITY_HIGH, .de_control_pin =
				(bsp_io_port_pin_t) SCIF_UART_INVALID_16BIT_PARAM, }, };

/** UART interface configuration */
const uart_cfg_t g_uart0_cfg = { .channel = 1, .data_bits = UART_DATA_BITS_8,
		.parity = UART_PARITY_OFF, .stop_bits = UART_STOP_BITS_1, .p_callback =
				uart0_cb_stdio, .p_context = NULL, .p_extend =
				&g_uart0_cfg_extend, .p_transfer_tx = g_uart0_P_TRANSFER_TX,
		.p_transfer_rx = g_uart0_P_TRANSFER_RX, .rxi_ipl = 24, .txi_ipl = 24,
		.tei_ipl = 24, .eri_ipl = 24, .rxi_irq = SCIF1_RXI_IRQn, .txi_irq =
				SCIF1_TXI_IRQn, .tei_irq = SCIF1_TEI_DRI_IRQn, .eri_irq =
				SCIF1_RERR_IRQn, };

/* Instance structure to use this module. */
const uart_instance_t g_uart0 = { .p_ctrl = &g_uart0_ctrl,
		.p_cfg = &g_uart0_cfg, .p_api = &g_uart_on_scif };
gtm_instance_ctrl_t g_timer1_ctrl;
const gtm_extended_cfg_t g_timer1_extend = { .generate_interrupt_when_starts =
		GTM_GIWS_TYPE_DISABLED, .gtm_mode = GTM_TIMER_MODE_INTERVAL, };
const timer_cfg_t g_timer1_cfg = { .mode = TIMER_MODE_PERIODIC, .period_counts =
		(uint32_t) 0x1869f /* Actual period: 0.001 seconds. */, .channel = 1,
		.p_callback = timer_tick_callback, .p_context = NULL, .cycle_end_ipl =
				18, .p_extend = &g_timer1_extend,
#if defined(VECTOR_NUMBER_GTM1_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GTM1_COUNTER_OVERFLOW,
#else
		.cycle_end_irq = GTM1_OSTM1INT_IRQn,
#endif
		};
/* Instance structure to use this module. */
const timer_instance_t g_timer1 = { .p_ctrl = &g_timer1_ctrl, .p_cfg =
		&g_timer1_cfg, .p_api = &g_timer_on_gtm };
intc_irq_instance_ctrl_t g_external_irq1_ctrl;
const external_irq_cfg_t g_external_irq1_cfg = { .channel = 1, .trigger =
		EXTERNAL_IRQ_TRIG_FALLING, .filter_enable = false, .clock_source_div =
		EXTERNAL_IRQ_CLOCK_SOURCE_DIV_1, .p_callback = touch_irq_callback,
		.p_context = NULL, .p_extend = NULL, .ipl = 24, .irq = GPIO_IRQ1_IRQn, };
/* Instance structure to use this module. */
const external_irq_instance_t g_external_irq1 = { .p_ctrl =
		&g_external_irq1_ctrl, .p_cfg = &g_external_irq1_cfg, .p_api =
		&g_external_irq_on_intc_irq };
#define FSP_NOT_DEFINED (UINT32_MAX)
#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void ssi_tx_dmac_callback(ssi_instance_ctrl_t * p_ctrl);

void g_i2s0_tx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    ssi_tx_dmac_callback(&g_i2s0_ctrl);
}
#endif

#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void ssi_rx_dmac_callback(ssi_instance_ctrl_t * p_ctrl);

void g_i2s0_rx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    ssi_rx_dmac_callback(&g_i2s0_ctrl);
}
#endif
#undef FSP_NOT_DEFINED

ssi_instance_ctrl_t g_i2s0_ctrl;

/** SSI instance configuration */
const ssi_extended_cfg_t g_i2s0_cfg_extend = { .audio_clock =
		SSI_AUDIO_CLOCK_EXTERNAL1, .bit_clock_div = SSI_CLOCK_DIV_1 };

/** I2S interface configuration */
const i2s_cfg_t g_i2s0_cfg = { .channel = 0, .pcm_width = I2S_PCM_WIDTH_16_BITS,
		.operating_mode = I2S_MODE_MASTER, .word_length =
				I2S_WORD_LENGTH_16_BITS, .ws_continue = I2S_WS_CONTINUE_OFF,
		.p_callback = NULL, .p_context = NULL, .p_extend = &g_i2s0_cfg_extend,

#define SSI0_TX_IRQ_ENABLE  (1)
#if (BSP_IRQ_DISABLED) != SSI0_TX_IRQ_ENABLE
#if 2 != 0
		.txi_irq = I2S0_DMA_TX_IRQn,
#else
                .txi_irq             = I2S0_DMA_RT_IRQn,
 #endif
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#define SSI0_RX_IRQ_ENABLE  (1)
#if (BSP_IRQ_DISABLED) != SSI0_RX_IRQ_ENABLE
#if 2 != 0
		.rxi_irq = I2S0_DMA_RX_IRQn,
#else
                .rxi_irq             = I2S0_DMA_RT_IRQn,
 #endif
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
		.int_irq = I2S0_INT_REQ_IRQn,

		.txi_ipl = 24, .rxi_ipl = 24, .idle_err_ipl = 24, .p_transfer_tx =
				g_i2s0_P_TRANSFER_TX, .p_transfer_rx = g_i2s0_P_TRANSFER_RX,

};

/* Instance structure to use this module. */
const i2s_instance_t g_i2s0 = { .p_ctrl = &g_i2s0_ctrl, .p_cfg = &g_i2s0_cfg,
		.p_api = &g_i2s_on_ssi };
#define FSP_NOT_DEFINED (UINT32_MAX)
#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void riic_master_tx_dmac_callback(iic_master_instance_ctrl_t * p_ctrl);

void g_i2c_master1_tx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    riic_master_tx_dmac_callback(&g_i2c_master1_ctrl);
}
#endif

#if (FSP_NOT_DEFINED) != (FSP_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void riic_master_rx_dmac_callback(iic_master_instance_ctrl_t * p_ctrl);

void g_i2c_master1_rx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    riic_master_rx_dmac_callback(&g_i2c_master1_ctrl);
}
#endif
#undef FSP_NOT_DEFINED

iic_master_instance_ctrl_t g_i2c_master1_ctrl;
const riic_master_extended_cfg_t g_i2c_master1_extend =
		{ .timeout_mode = IIC_MASTER_TIMEOUT_MODE_LONG, .timeout_scl_low =
				IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
				/* Actual calculated bitrate: 396825. Actual calculated duty cycle: 51%. */.clock_settings.brl_value =
						25, .clock_settings.brh_value = 26,
				.clock_settings.cks_value = 2, .noise_filter_stage = 1,
				.naki_irq = RIIC1_NAKI_IRQn, .spi_irq = RIIC1_SPI_IRQn,
				.sti_irq = RIIC1_STI_IRQn, .ali_irq = RIIC1_ALI_IRQn,
				.tmoi_irq = RIIC1_TMOI_IRQn };
const i2c_master_cfg_t g_i2c_master1_cfg = { .channel = 1, .rate =
		I2C_MASTER_RATE_FAST, .slave = 0x2A, .addr_mode =
		I2C_MASTER_ADDR_MODE_7BIT, .p_transfer_tx = g_i2c_master1_P_TRANSFER_TX,
		.p_transfer_rx = g_i2c_master1_P_TRANSFER_RX, .p_callback =
				g_i2c_master1_cb, .p_context = NULL, .rxi_irq = RIIC1_RI_IRQn,
		.txi_irq = RIIC1_TI_IRQn, .tei_irq = RIIC1_TEI_IRQn, .ipl = 24,
		.p_extend = &g_i2c_master1_extend, };
/* Instance structure to use this module. */
const i2c_master_instance_t g_i2c_master1 = { .p_ctrl = &g_i2c_master1_ctrl,
		.p_cfg = &g_i2c_master1_cfg, .p_api = &g_i2c_master_on_iic };
/** Display framebuffer */

#if LCDC_CFG_LAYER_1_ENABLE
        #ifdef fb_background_DEFINE_CRU
        #else
        #ifdef DISPLAY_INPUT0_BUFFER_NUM_0
        /** FSP do not set buffer for layer1 */
        #else
        uint8_t fb_background[2][DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 * DISPLAY_VSIZE_INPUT0] BSP_ALIGN_VARIABLE(64) BSP_PLACE_IN_SECTION("CACHED_BSS")__attribute__ ((__aligned__(512)));
        #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_0
        uint8_t fb_background_CB[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0 * DISPLAY_VSIZE_UV_INPUT0] BSP_ALIGN_VARIABLE(64) BSP_PLACE_IN_SECTION("CACHED_BSS")__attribute__ ((__aligned__(512)));
        uint8_t fb_background_CR[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0 * DISPLAY_VSIZE_UV_INPUT0] BSP_ALIGN_VARIABLE(64) BSP_PLACE_IN_SECTION("CACHED_BSS")__attribute__ ((__aligned__(512)));
        #endif
        #define fb_background_DEFINE_LCDC
        #endif
        #endif
        #else
/** Graphics Layer 1 is specified not to be used when starting */
#endif

#if LCDC_CFG_LAYER_2_ENABLE
        #ifdef fb_foreground_DEFINE_CRU
        #else
        #ifdef DISPLAY_INPUT1_BUFFER_NUM_0
        /** FSP do not set buffer for layer2 */
        #else
        uint8_t fb_foreground[2][DISPLAY_BUFFER_STRIDE_BYTES_INPUT1 * DISPLAY_VSIZE_INPUT1] BSP_ALIGN_VARIABLE(64) BSP_PLACE_IN_SECTION("UNCACHED_BSS")__attribute__ ((__aligned__(512)));
        #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_1
        uint8_t fb_foreground_CB[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1 * DISPLAY_VSIZE_UV_INPUT1]BSP_ALIGN_VARIABLE(64) BSP_PLACE_IN_SECTION("UNCACHED_BSS")__attribute__ ((__aligned__(512)));
		uint8_t fb_foreground_CR[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1 * DISPLAY_VSIZE_UV_INPUT1]BSP_ALIGN_VARIABLE(64) BSP_PLACE_IN_SECTION("UNCACHED_BSS")__attribute__ ((__aligned__(512)));
        #endif
        #define fb_foreground_DEFINE_LCDC
        #endif
        #endif
        #else
/** Graphics Layer 2 is specified not to be used when starting */
#endif

/** Display control block instance */
lcdc_instance_ctrl_t g_display0_ctrl;

/* Display devie extended configuration */
const lcdc_extended_cfg_t g_display0_extend_cfg = { .frame_end_ipl = 18,
		.underrun_ipl = 18, .frame_end_irq = LCDC_VSPD_INT_IRQn, .underrun_irq =
				LCDC_VSPD_INT_IRQn, };

/** Display interface configuration */
const display_cfg_t g_display0_cfg = {
/** Input1(Graphics1 layer) configuration */
.input[0] =
{
#if LCDC_CFG_LAYER_1_ENABLE
                #ifdef DISPLAY_INPUT0_BUFFER_NUM_0
                .p_base              = NULL,
                #else
                .p_base              = (uint32_t *)&fb_background,
                #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_0
                .p_base_cb           = (uint32_t *)&fb_background_CB,
                .p_base_cr           = (uint32_t *)&fb_background_CR,
                .hstride_cbcr        = DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0,
                #endif
                #endif
                #else
		.p_base = NULL,
#endif
		.hsize = DISPLAY_HSIZE_INPUT0, .vsize = DISPLAY_VSIZE_INPUT0,
		.coordinate_x = 0, .coordinate_y = 0, .hstride =
				DISPLAY_BUFFER_STRIDE_BYTES_INPUT0, .format =
				DISPLAY_IN_FORMAT_16BITS_RGB565, .data_swap =
				DISPLAY_DATA_SWAP_64BIT | DISPLAY_DATA_SWAP_32BIT
						| DISPLAY_DATA_SWAP_16BIT | false, },

/** Input2(Graphics2 layer) configuration */
.input[1] =
{
#if LCDC_CFG_LAYER_2_ENABLE
                #ifdef DISPLAY_INPUT1_BUFFER_NUM_0
                .p_base              = NULL,
                #else
                .p_base              = (uint32_t *)&fb_foreground,
                #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_1
                .p_base_cb              = (uint32_t *)&fb_foreground_CB,
                .p_base_cr              = (uint32_t *)&fb_foreground_CR,
                .hstride_cbcr           = DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1,
                #endif
                #endif
                #else
		.p_base = NULL,
#endif
		.hsize = DISPLAY_HSIZE_INPUT1, .vsize = DISPLAY_VSIZE_INPUT1,
		.coordinate_x = 0, .coordinate_y = 0, .hstride =
				DISPLAY_BUFFER_STRIDE_BYTES_INPUT1, .format =
				DISPLAY_IN_FORMAT_32BITS_ARGB8888, .data_swap =
				DISPLAY_DATA_SWAP_64BIT | DISPLAY_DATA_SWAP_32BIT | false
						| false, },

/** Output configuration */
.output = { .htiming = { .total_cyc = 828, .display_cyc = 720, .back_porch = 56,
		.sync_width = 8, .sync_polarity = DISPLAY_SIGNAL_POLARITY_HIACTIVE },
		.vtiming = { .total_cyc = 1317, .display_cyc = 1280, .back_porch = 21,
				.sync_width = 6, .sync_polarity =
						DISPLAY_SIGNAL_POLARITY_HIACTIVE }, .format =
				DISPLAY_OUT_FORMAT_24BITS_RGB888, .data_enable_polarity =
				DISPLAY_SIGNAL_POLARITY_HIACTIVE, .sync_edge =
				DISPLAY_SIGNAL_SYNC_EDGE_RISING, .bg_color = { .byte = { .a = 0,
				.r = 0, .g = 0, .b = 0 } }, .dithering_on = false, },

/** Display device callback function pointer */
.p_callback = lcdc_callback, .p_context = NULL,

/** Display device extended configuration */
.p_extend = (void*) (&g_display0_extend_cfg), };

#if LCDC_CFG_LAYER_1_ENABLE
        /** Display on LCDC run-time configuration(for the graphics1 layer) */
        display_runtime_cfg_t g_display0_runtime_cfg_bg =
        {
            .input =
            {
                #if (true)
                #ifdef DISPLAY_INPUT0_BUFFER_NUM_0
                .p_base              = NULL,
                #else
                .p_base              = (uint32_t *)&fb_background,
                #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_0
                .p_base_cb              = (uint32_t *)&fb_background_CB,
                .p_base_cr              = (uint32_t *)&fb_background_CR,
                .hstride_cbcr           = DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0,
                #endif
                #endif
                #else
                .p_base              = NULL,
                #endif
                .hsize               = DISPLAY_HSIZE_INPUT0,
                .vsize               = DISPLAY_VSIZE_INPUT0,
                .coordinate_x        = 0,
                .coordinate_y        = 0,
                .hstride             = DISPLAY_BUFFER_STRIDE_BYTES_INPUT0,
                .format              = DISPLAY_IN_FORMAT_16BITS_RGB565,
                .data_swap			 = DISPLAY_DATA_SWAP_64BIT | DISPLAY_DATA_SWAP_32BIT | DISPLAY_DATA_SWAP_16BIT | false,
            },
        };
#endif
#if LCDC_CFG_LAYER_2_ENABLE

        /** Display on LCDC run-time configuration(for the graphics2 layer) */
        display_runtime_cfg_t g_display0_runtime_cfg_fg =
        {
            .input =
            {
                #if (false)
                #ifdef DISPLAY_INPUT1_BUFFER_NUM_0
                .p_base              = NULL,
                #else
                .p_base              = (uint32_t *)&fb_foreground,
                #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_1
                .p_base_cb              = (uint32_t *)&fb_foreground_CB,
                .p_base_cr              = (uint32_t *)&fb_foreground_CR,
                .hstride_cbcr           = DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1,
                #endif
                #endif
                #else
                .p_base              = NULL,
                #endif
                .hsize               = DISPLAY_HSIZE_INPUT1,
                .vsize               = DISPLAY_VSIZE_INPUT1,
                .coordinate_x        = 0,
                .coordinate_y        = 0,
                .hstride             = DISPLAY_BUFFER_STRIDE_BYTES_INPUT1,
                .format              = DISPLAY_IN_FORMAT_32BITS_ARGB8888,
                .data_swap			 = DISPLAY_DATA_SWAP_64BIT | DISPLAY_DATA_SWAP_32BIT | false | false,
             },
        };
#endif

/** Color Keying Configuration */
display_colorkeying_layer_t g_display0_ckey_cfg = { .layer[0] =
{ .src_color = { .byte = { .a = 0, .r = 0, .g = 0, .b = 0, }, }, .dst_color = {
		.byte = { .a = 0, .r = 0, .g = 0, .b = 0, }, }, .enable_ckey =
		DISPLAY_COLOR_KEYING_DISABLE, },

.layer[1] =
{ .src_color = { .byte = { .a = 0, .r = 0, .g = 0, .b = 0 }, }, .dst_color = {
		.byte = { .a = 0, .r = 0, .g = 0, .b = 0 }, }, .enable_ckey =
		DISPLAY_COLOR_KEYING_DISABLE, } };

/* Instance structure to use this module. */
const display_instance_t g_display0 = { .p_ctrl = &g_display0_ctrl, .p_cfg =
		(display_cfg_t*) &g_display0_cfg, .p_api =
		(display_api_t*) &g_display_on_lcdc };
gtm_instance_ctrl_t g_timer0_ctrl;
const gtm_extended_cfg_t g_timer0_extend = { .generate_interrupt_when_starts =
		GTM_GIWS_TYPE_DISABLED, .gtm_mode = GTM_TIMER_MODE_INTERVAL, };
const timer_cfg_t g_timer0_cfg = { .mode = TIMER_MODE_PERIODIC, .period_counts =
		(uint32_t) 0x1869f /* Actual period: 0.001 seconds. */, .channel = 0,
		.p_callback = NULL, .p_context = NULL, .cycle_end_ipl = 30, .p_extend =
				&g_timer0_extend,
#if defined(VECTOR_NUMBER_GTM0_COUNTER_OVERFLOW)
    .cycle_end_irq       = VECTOR_NUMBER_GTM0_COUNTER_OVERFLOW,
#else
		.cycle_end_irq = GTM0_OSTM0INT_IRQn,
#endif
		};
/* Instance structure to use this module. */
const timer_instance_t g_timer0 = { .p_ctrl = &g_timer0_ctrl, .p_cfg =
		&g_timer0_cfg, .p_api = &g_timer_on_gtm };
extern void FreeRTOS_Tick_Handler(timer_callback_args_t*);
__attribute__( ( weak ) ) void vPortSetupTimerInterrupt(void) /* PRIVILEGED_FUNCTION */
{
	/* Stop and reset the SysTick. */
	g_timer0.p_api->open(g_timer0.p_ctrl, g_timer0.p_cfg);
	/* Configure SysTick to interrupt at the requested rate. */
	g_timer0.p_api->callbackSet(g_timer0.p_ctrl, FreeRTOS_Tick_Handler, NULL,
			NULL);
	/* Start the SysTick. */
	g_timer0.p_api->start(g_timer0.p_ctrl);

}
#if configGENERATE_RUN_TIME_STATS
__attribute__( ( weak ) ) void vPortConfigureTimerForRunTimeStatus( void )
{
    /* Stop and reset the Status Timer. */
    FSP_NOT_DEFINED.p_api->open(FSP_NOT_DEFINED.p_ctrl, FSP_NOT_DEFINED.p_cfg);
    /* Start the Status Timer. */
    FSP_NOT_DEFINED.p_api->start(FSP_NOT_DEFINED.p_ctrl);
}

/* Count clock = 100MHz
   10-bits-right-shift makes 10.24us counting interval */
#define runtimeCLOCK_SCALE_SHIFT    ( 10 )

/* GTM overflows 32bit */
#define runtimeOVERFLOW_SHIFT       ( 32 - runtimeCLOCK_SCALE_SHIFT )
__attribute__( ( weak ) ) unsigned long ulGetRunTimeCounterValue( void )
{
    static unsigned long ulLastCounterValue = 0UL, ulOverflows = 0;
    unsigned long ulValueNow = 0;
    timer_status_t timer_status;

    /* Get count value */
    if (FSP_SUCCESS == FSP_NOT_DEFINED.p_api->statusGet(FSP_NOT_DEFINED.p_ctrl, &timer_status))
    {
        ulValueNow = timer_status.counter;

        /* Has the value overflowed since it was last read. */
        if( ulValueNow < ulLastCounterValue )
        {
            ulOverflows++;
        }
        ulLastCounterValue = ulValueNow;

        /* move the count value to  */
        ulValueNow >>= runtimeCLOCK_SCALE_SHIFT;
        ulValueNow += ulOverflows << runtimeOVERFLOW_SHIFT;
    }

    return ulValueNow;
}
#endif
void g_hal_init(void) {
	g_common_init();
}
