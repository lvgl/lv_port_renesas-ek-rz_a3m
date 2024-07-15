/**
 * @file lv_port_disp_templ.c
 *
 */


/*********************
 *      INCLUDES
 *********************/
#include "LVGL_thread.h"
#include "touch_GT911.h"
#include "arducam.h"
#include <stdbool.h>
#include "lv_port_disp.h"
#include "lvgl/src/display/lv_display_private.h"
#include "mipi_dsi_hal.h"
#include "../device/ili9881/ili9881.h"
#include "../device/isl97671a/inc/isl97671a.h"

/*********************
 *      DEFINES
 *********************/


#define RGB_565_BLACK  (0)
#define RGB_565_RED    (0x1F << 11)
#define RGB_565_GREEN  (0x3F << 5)
#define RGB_565_BLUE   (0x1F << 0)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void vsync_wait_cb(struct _lv_display_t * disp);


static void * rotation_buffer = NULL;
static uint32_t partial_buffer_size = 0;


/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
#if (1)
    uint8_t* p_fb_background_cached_0;
    uint8_t* p_fb_background_cached_1;
    R_MMU_VAtoPA(&g_mmu_ctrl, &fb_background[0][0], &p_fb_background_cached_0);
    R_MMU_VAtoPA(&g_mmu_ctrl, &fb_background[1][0], &p_fb_background_cached_1);
#endif
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/
    lv_display_t * disp = lv_display_create(DISPLAY_HSIZE_INPUT0, DISPLAY_VSIZE_INPUT0);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_flush_wait_cb(disp, vsync_wait_cb);
#if (1)
    static lv_color_t partial_draw_buf[DISPLAY_HSIZE_INPUT0 * DISPLAY_VSIZE_INPUT0 / 10] BSP_PLACE_IN_SECTION(".bss2") BSP_ALIGN_VARIABLE(1024);

    partial_buffer_size = sizeof(partial_draw_buf);
    lv_display_set_buffers(disp, partial_draw_buf, NULL, sizeof(partial_draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
//    lv_display_set_buffers(disp, p_fb_background_cached_0, p_fb_background_cached_1, sizeof(fb_background[0]), );
#else
    lv_display_set_buffers(disp, &fb_background[0][0], &fb_background[1][0], sizeof(fb_background[0]), LV_DISPLAY_RENDER_MODE_DIRECT);
#endif

    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void _display_panel_clock_setup( void )
{
    /* Setting for PIXEL Clock (PLL5 Pixel Clock) */
#if 0
    /* PLL: 792.0MHz, division ratio: 1/12  */
    //R_CPG->CPG_OTHERFUNC1_REG = 0x1;
    R_CPG->CPG_SIPLL5_CLK1 = 0x01110231;
    R_CPG->CPG_SIPLL5_CLK3 = 0x00000000;
    R_CPG->CPG_SIPLL5_CLK4 = 0x00C60000;
    R_CPG->CPG_PL5_SDIV    = 0x01010B00;
    R_CPG->CPG_SIPLL5_STBY = 0x00150001;
#else
    /* 66MHz 800x1280 */
    uint32_t reg;
    uint32_t fracin;
    uint32_t intin;
    uint32_t refdiv;
    uint32_t postdiv1;
    uint32_t postdiv2;
    uint32_t divdsia_set;
    uint32_t divdsib_set;

    fracin   = 0;
    intin    = 66;
    refdiv    = 1;
    postdiv1 = 2;
    postdiv2 = 1;
    divdsia_set = 0;
    divdsib_set = 11;

    /* Setting for PIXEL Clock (PLL5 Pixel Clock) */
    //R_CPG->CPG_OTHERFUNC1_REG = 0x00010001;//@@
    reg  = R_CPG->CPG_SIPLL5_CLK3;
    reg &= 0x0000FFFF;
    reg |= (fracin << 16);
    R_CPG->CPG_SIPLL5_CLK3 = reg;
    R_CPG->CPG_SIPLL5_CLK4 = (intin << 16);
    reg = R_CPG->CPG_SIPLL5_CLK1;
    reg &= 0xFFFF0000;
    reg |= ( (1 << 24) | (1 << 20) | (1 << 16) | (refdiv << 8) | (postdiv2 << 4) | postdiv1 );
    R_CPG->CPG_SIPLL5_CLK1 = reg;
    reg = 0x01010000;
    reg |= (divdsib_set << 8) | (divdsia_set);
    R_CPG->CPG_PL5_SDIV  = reg;
    R_CPG->CPG_SIPLL5_STBY = 0x00150001;
#endif
}

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    fsp_err_t err;
    uint8_t * p_fb = &fb_background[1][0];

    _display_panel_clock_setup();

    /* MIPI-DSI setup */
    mipi_dsi_init(g_mipi_dsi0.p_cfg, g_display0.p_cfg);
    ili9881_init();

    /* Fill the Frame buffer with a colour, to zero out info from previous execution runs */
    uint32_t count;
    uint16_t * p = (uint16_t *)&fb_background[0][0];

    for (count = 0; count < sizeof(fb_background)/2; count++)
    {
        *p++ = RGB_565_BLACK;
    }

    err = R_LCDC_Open(&g_display0_ctrl, &g_display0_cfg);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0); //TODO: Better error handling
    }

    err = R_LCDC_Start(&g_display0_ctrl);
    if (FSP_SUCCESS != err)
    {
        __BKPT(0); //TODO: Better error handling
    }

    do
    {
        err =
            R_LCDC_BufferChange(&g_display0_ctrl,
                                 (uint8_t *) p_fb,
                                 (display_frame_layer_t) DISPLAY_FRAME_LAYER_1);
    } while (FSP_ERR_INVALID_UPDATE_TIMING == err);

    /* Enable the backlight */
    uint32_t duty = 0;
    while (1) {
        isl87671a_ctrl_func(NULL, LED_METHOD_1, duty);
        if( duty >= 100 )
        {
            break;
        }
        else
        {
            duty += 5;
        }
    }
}

void lcdc_callback(display_callback_args_t *p_args)
{
    if (DISPLAY_EVENT_FRAME_END == p_args->event)
    {
#if BSP_CFG_RTOS == 2               // FreeRTOS
       BaseType_t context_switch;

       //
       // Set Vsync semaphore
       //
       xSemaphoreGiveFromISR(_SemaphoreVsync, &context_switch);

       //
       // Return to the highest priority available task
       //
       portYIELD_FROM_ISR(context_switch);
#else
#endif
    }
    else if (DISPLAY_EVENT_GR1_UNDERFLOW == p_args->event)
    {
        __BKPT(0); //Layer 1 Underrun
    }
    else //DISPLAY_EVENT_LINE_DETECTION
    {
        __BKPT(0);
    }

}

static void vsync_wait_cb(lv_display_t * display)
{
    LV_UNUSED(display);


}

#define BYTES_PER_PIXEL 2

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    uint16_t * img = (uint16_t *)px_map;

    lv_color_format_t cf = lv_display_get_color_format(display);
    lv_display_rotation_t rotation = lv_display_get_rotation(display);

    if(rotation != LV_DISPLAY_ROTATION_0) {
        int32_t w = lv_area_get_width(area);
        int32_t h = lv_area_get_height(area);
        uint32_t w_stride = lv_draw_buf_width_to_stride(w, cf);
        uint32_t h_stride = lv_draw_buf_width_to_stride(h, cf);

        // only allocate if rotation is actually being used
        if(!rotation_buffer) {
            rotation_buffer = lv_malloc(partial_buffer_size);
            LV_ASSERT_MALLOC(rotation_buffer);
        }

        if(rotation == LV_DISPLAY_ROTATION_180)
            lv_draw_sw_rotate(img, rotation_buffer, w, h, w_stride, w_stride, rotation, cf);
        else /* 90 or 270 */
            lv_draw_sw_rotate(img, rotation_buffer, w, h, w_stride, h_stride, rotation, cf);

        img = rotation_buffer;
        lv_display_rotate_area(display, (lv_area_t *)area);
    }

    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);

    uint16_t * fb = (uint16_t *)fb_background[1];

    fb = fb + area->y1 * DISPLAY_HSIZE_INPUT0;
    fb = fb + area->x1;

    int32_t i;
    for(i = 0; i < h; i++) {
        lv_memcpy(fb, img, w * BYTES_PER_PIXEL);

        //R_BSP_CACHE_CleanRange(fb, w * BYTES_PER_PIXEL);

        fb += DISPLAY_HSIZE_INPUT0;
        img += w;
    }
}
