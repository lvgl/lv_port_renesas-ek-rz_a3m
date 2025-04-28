/* generated thread source file - do not edit */
#include "LVGL_thread.h"

#if 0
                static StaticTask_t LVGL_thread_memory;
                #if defined(__ARMCC_VERSION)           /* AC6 compiler */
                static uint8_t LVGL_thread_stack[8192] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #else
                static uint8_t LVGL_thread_stack[8192] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.LVGL_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #endif
                #endif
TaskHandle_t LVGL_thread;
void LVGL_thread_create(void);
static void LVGL_thread_func(void *pvParameters);
void rtos_startup_err_callback(void *p_instance, void *p_data);
void rtos_startup_common_init(void);
extern uint32_t g_fsp_common_thread_count;

const rm_freertos_port_parameters_t LVGL_thread_parameters = { .p_context =
		(void*) NULL, };

void LVGL_thread_create(void) {
	/* Increment count so we will know the number of threads created in the RA Configuration editor. */
	g_fsp_common_thread_count++;

	/* Initialize each kernel object. */

#if 0
                    LVGL_thread = xTaskCreateStatic(
                    #else
	BaseType_t LVGL_thread_create_err = xTaskCreate(
#endif
			LVGL_thread_func, (const char*) "LVGL Thread", 8192 / 4, // In words, not bytes
			(void*) &LVGL_thread_parameters, //pvParameters
			1,
#if 0
                        (StackType_t *)&LVGL_thread_stack,
                        (StaticTask_t *)&LVGL_thread_memory
                        #else
			&LVGL_thread
#endif
			);

#if 0
                    if (NULL == LVGL_thread)
                    {
                        rtos_startup_err_callback(LVGL_thread, 0);
                    }
                    #else
	if (pdPASS != LVGL_thread_create_err) {
		rtos_startup_err_callback(LVGL_thread, 0);
	}
#endif
}
static void LVGL_thread_func(void *pvParameters) {
	/* Initialize common components */
	rtos_startup_common_init();

	/* Initialize each module instance. */

	/* Enter user code for this thread. Pass task handle. */
	LVGL_thread_entry(pvParameters);
}
