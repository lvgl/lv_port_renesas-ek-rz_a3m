/* generated thread header file - do not edit */
#ifndef LVGL_THREAD_H_
#define LVGL_THREAD_H_
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal_data.h"
#include  <stdbool.h>
#ifdef __cplusplus
                extern "C" void LVGL_thread_entry(void * pvParameters);
                #else
extern void LVGL_thread_entry(void *pvParameters);
#endif
FSP_HEADER
FSP_FOOTER
#endif /* LVGL_THREAD_H_ */
