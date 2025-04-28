/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_uart_api.h"
#include "r_scif_uart.h"
#include "r_gtm.h"
#include "r_intc_irq.h"
#include "r_external_irq_api.h"
#include "r_i2s_api.h"
#include "r_ssi.h"
#include "r_riic_master.h"
#include "r_i2c_master_api.h"
#include "r_lcdc.h"
#include "r_display_api.h"
#include "FreeRTOS.h"
#include "r_timer_api.h"
FSP_HEADER
/** UART on SCIF Instance. */
extern const uart_instance_t g_uart0;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern scif_uart_instance_ctrl_t g_uart0_ctrl;
extern const uart_cfg_t g_uart0_cfg;
extern const scif_uart_extended_cfg_t g_uart0_cfg_extend;

void uart0_cb_stdio(uart_callback_args_t *p_args);

#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
#define g_uart0_P_TRANSFER_TX (NULL)
#else
    #define g_uart0_P_TRANSFER_TX (&FSP_NOT_DEFINED)
#endif
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
#define g_uart0_P_TRANSFER_RX (NULL)
#else
    #define g_uart0_P_TRANSFER_RX (&FSP_NOT_DEFINED)
#endif
#undef FSP_NOT_DEFINED
/** GTM Timer Instance */
extern const timer_instance_t g_timer1;

/** Access the GTM instance using these structures when calling API functions directly (::p_api is not used). */
extern gtm_instance_ctrl_t g_timer1_ctrl;
extern const timer_cfg_t g_timer1_cfg;

void timer_tick_callback(timer_callback_args_t *p_args);
/** External IRQ on INTC_IRQ Instance. */
extern const external_irq_instance_t g_external_irq1;

/** Access the INTC_IRQ instance using these structures when calling API functions directly (::p_api is not used). */
extern intc_irq_instance_ctrl_t g_external_irq1_ctrl;
extern const external_irq_cfg_t g_external_irq1_cfg;

void touch_irq_callback(external_irq_callback_args_t *p_args);
/** SSI Instance. */
extern const i2s_instance_t g_i2s0;

/** Access the SSI instance using these structures when calling API functions directly (::p_api is not used). */
extern ssi_instance_ctrl_t g_i2s0_ctrl;
extern const i2s_cfg_t g_i2s0_cfg;

#ifndef NULL
void NULL(i2s_callback_args_t *p_args);
#endif

#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
#define g_i2s0_P_TRANSFER_TX (NULL)
#else
    #define g_i2s0_P_TRANSFER_TX (&FSP_NOT_DEFINED)
#endif

#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
#define g_i2s0_P_TRANSFER_RX (NULL)
#else
    #define g_i2s0_P_TRANSFER_RX (&FSP_NOT_DEFINED)
#endif
#undef FSP_NOT_DEFINED
/* I2C Master on RIIC Instance. */
extern const i2c_master_instance_t g_i2c_master1;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_master1_ctrl;
extern const i2c_master_cfg_t g_i2c_master1_cfg;

void g_i2c_master1_cb(i2c_master_callback_args_t *p_args);

#define FSP_NOT_DEFINED (1)
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
#define g_i2c_master1_P_TRANSFER_TX (NULL)
#else
    #define g_i2c_master1_P_TRANSFER_TX (&FSP_NOT_DEFINED)
#endif
#if (FSP_NOT_DEFINED == FSP_NOT_DEFINED)
#define g_i2c_master1_P_TRANSFER_RX (NULL)
#else
    #define g_i2c_master1_P_TRANSFER_RX (&FSP_NOT_DEFINED)
#endif
#undef FSP_NOT_DEFINED
#define LCDC_CFG_LAYER_1_ENABLE (true)
#define LCDC_CFG_LAYER_2_ENABLE (false)

/* Display on LCDC Instance. */
extern const display_instance_t g_display0;
extern display_runtime_cfg_t g_display0_runtime_cfg_fg;
extern display_runtime_cfg_t g_display0_runtime_cfg_bg;
extern display_colorkeying_layer_t g_display0_ckey_cfg;

/** Access the LCDC instance using these structures when calling API functions directly (::p_api is not used). */
extern lcdc_instance_ctrl_t g_display0_ctrl;
extern const display_cfg_t g_display0_cfg;

void lcdc_callback(display_callback_args_t *p_args);

#define DISPLAY_IN_FORMAT_16BITS_RGB565_0
#if defined (DISPLAY_IN_FORMAT_32BITS_ARGB8888_0) || defined (DISPLAY_IN_FORMAT_32BITS_RGBA8888_0) || defined (DISPLAY_IN_FORMAT_32BITS_ABGR8888_0)
            #define DISPLAY_BITS_PER_PIXEL_INPUT0 (32)
            #elif defined (DISPLAY_IN_FORMAT_24BITS_RGB888_0) || defined (DISPLAY_IN_FORMAT_24BITS_BGR888_0) || defined (DISPLAY_IN_FORMAT_24BITS_YCBCR444_INTERLEAVED_0)
            #define DISPLAY_BITS_PER_PIXEL_INPUT0 (24)
            #elif defined (DISPLAY_IN_FORMAT_16BITS_RGB565_0) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE0_UYVY_0) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE0_YUY2_0) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE0_YVYU_0) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE1_0)
#define DISPLAY_BITS_PER_PIXEL_INPUT0 (16)
#elif defined (DISPLAY_IN_FORMAT_16BITS_YCBCR420_INTERLEAVED_0)
            #define DISPLAY_BITS_PER_PIXEL_INPUT0 (24)
            #elif defined (DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_0)
            #define DISPLAY_BITS_PER_PIXEL_INPUT0 (8)
            #else
            #define DISPLAY_BITS_PER_PIXEL_INPUT0 (1)
            #endif
#define DISPLAY_HSIZE_INPUT0                 (720)
#define DISPLAY_HSIZE_UV_INPUT0              (720 >> 1)
#define DISPLAY_VSIZE_INPUT0                 (1280)
#define DISPLAY_VSIZE_UV_INPUT0              (1280 >> 1)
#define DISPLAY_BUFFER_STRIDE_BYTES_INPUT0   ((DISPLAY_HSIZE_INPUT0 * DISPLAY_BITS_PER_PIXEL_INPUT0) >> 3)
#define DISPLAY_BUFFER_STRIDE_PIXELS_INPUT0  ((DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 * 8) / DISPLAY_BITS_PER_PIXEL_INPUT0)
#define DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0   ((DISPLAY_HSIZE_UV_INPUT0 * DISPLAY_BITS_PER_PIXEL_INPUT0) >> 3)
#define DISPLAY_INPUT0_BUFFER_NUM_2 (true)

#define DISPLAY_IN_FORMAT_32BITS_ARGB8888_1
#if defined (DISPLAY_IN_FORMAT_32BITS_ARGB8888_1) || defined (DISPLAY_IN_FORMAT_32BITS_RGBA8888_1) || defined (DISPLAY_IN_FORMAT_32BITS_ABGR8888_1)
#define DISPLAY_BITS_PER_PIXEL_INPUT1 (32)
#elif defined (DISPLAY_IN_FORMAT_24BITS_RGB888_1) || defined (DISPLAY_IN_FORMAT_24BITS_BGR888_1) || defined (DISPLAY_IN_FORMAT_24BITS_YCBCR444_INTERLEAVED_1)
            #define DISPLAY_BITS_PER_PIXEL_INPUT1 (24)
            #elif defined (DISPLAY_IN_FORMAT_16BITS_RGB565_1) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE0_UYVY_1) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE0_YUY2_1) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE0_YVYU_1) || defined (DISPLAY_IN_FORMAT_16BITS_YCBCR422_INTERLEAVED_TYPE1_1)
            #define DISPLAY_BITS_PER_PIXEL_INPUT1 (16)
            #elif defined (DISPLAY_IN_FORMAT_16BITS_YCBCR420_INTERLEAVED_1)
            #define DISPLAY_BITS_PER_PIXEL_INPUT1 (24)
            #elif defined (DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_1)
            #define DISPLAY_BITS_PER_PIXEL_INPUT1 (8)
            #else
            #define DISPLAY_BITS_PER_PIXEL_INPUT1 (1)
            #endif
#define DISPLAY_HSIZE_INPUT1                 (720)
#define DISPLAY_HSIZE_UV_INPUT1              (720 >> 1)
#define DISPLAY_VSIZE_INPUT1                 (800)
#define DISPLAY_VSIZE_UV_INPUT1              (800 >> 1)
#define DISPLAY_BUFFER_STRIDE_BYTES_INPUT1   ((DISPLAY_HSIZE_INPUT1 * DISPLAY_BITS_PER_PIXEL_INPUT1) >> 3)
#define DISPLAY_BUFFER_STRIDE_PIXELS_INPUT1  ((DISPLAY_BUFFER_STRIDE_BYTES_INPUT1 * 8) / DISPLAY_BITS_PER_PIXEL_INPUT1)
#define DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1   ((DISPLAY_HSIZE_UV_INPUT1 * DISPLAY_BITS_PER_PIXEL_INPUT1) >> 3)
#define DISPLAY_INPUT1_BUFFER_NUM_2 (true)

#if LCDC_CFG_LAYER_1_ENABLE
        #ifdef fb_background_DEFINE_CRU
        #else
        #ifdef DISPLAY_INPUT0_BUFFER_NUM_0
        /** FSP do not set buffer for layer1 */
        #else
        extern uint8_t fb_background[2][DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 * DISPLAY_VSIZE_INPUT0];
        #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_0
        extern uint8_t fb_background_CB[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0 * DISPLAY_VSIZE_UV_INPUT0];
        extern uint8_t fb_background_CR[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT0 * DISPLAY_VSIZE_UV_INPUT0];
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
        extern uint8_t fb_foreground[2][DISPLAY_BUFFER_STRIDE_BYTES_INPUT1 * DISPLAY_VSIZE_INPUT1];
        #ifdef DISPLAY_IN_FORMAT_16BITS_YCBCR420_PLANAR_1
        extern uint8_t fb_foreground_CB[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1 * DISPLAY_VSIZE_UV_INPUT1];
		extern uint8_t fb_foreground_CR[2][DISPLAY_BUFFER_STRIDE_UV_BYTES_INPUT1 * DISPLAY_VSIZE_UV_INPUT1];
        #endif
        #define fb_foreground_DEFINE_LCDC
        #endif
        #endif
        #else
/** Graphics Layer 2 is specified not to be used when starting */
#endif
/** GTM Timer Instance */
extern const timer_instance_t g_timer0;

/** Access the GTM instance using these structures when calling API functions directly (::p_api is not used). */
extern gtm_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

extern void vPortSetupTimerInterrupt(void); /* PRIVILEGED_FUNCTION */
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
