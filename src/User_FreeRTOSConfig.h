#ifndef USER_FREERTOSCONFIG_H_
#define USER_FREERTOSCONFIG_H_

#define configASSERT(x)  assert(x)

void lv_port_freertos_task_switch_in(const char * name);
void lv_port_freertos_task_switch_out(void);

#define traceTASK_SWITCHED_IN()   lv_port_freertos_task_switch_in(pxCurrentTCB->pcTaskName);
#define traceTASK_SWITCHED_OUT()  lv_port_freertos_task_switch_out();

#define configUSE_RECURSIVE_MUTEXES (1)

#endif /* USER_FREERTOSCONFIG_H_ */
