/* generated common header file - do not edit */
#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "r_mipi_phy_b.h"
#include "r_mipi_dsi_b.h"
#include "r_mipi_dsi_api.h"
#include "r_display_api.h"
#include "r_mmu.h"
#include "r_ioport.h"
#include "bsp_pin_cfg.h"
FSP_HEADER
/* MIPI PHY on MIPI PHY Instance. */
extern const mipi_phy_instance_t g_mipi_phy0;

/* Access the MIPI DSI instance using these structures when calling API functions directly (::p_api is not used). */
extern mipi_phy_b_ctrl_t g_mipi_phy0_ctrl;
extern const mipi_phy_b_cfg_t g_mipi_phy0_cfg;
/* MIPI DSI on MIPI DSI Instance. */
extern const mipi_dsi_instance_t g_mipi_dsi0;

/* Access the MIPI DSI instance using these structures when calling API functions directly (::p_api is not used). */
extern mipi_dsi_b_instance_ctrl_t g_mipi_dsi0_ctrl;
extern const mipi_dsi_cfg_t g_mipi_dsi0_cfg;

#ifndef mipi_dsi0_callback
void mipi_dsi0_callback(mipi_dsi_callback_args_t *p_args);
#endif
/* MMU Instance */
extern const mmu_instance_t g_mmu;

/* MMU control structure. */
extern mmu_instance_ctrl_t g_mmu_ctrl;
extern const mmu_cfg_t g_mmu_cfg;
#define IOPORT_CFG_NAME g_bsp_pin_cfg
#define IOPORT_CFG_OPEN R_IOPORT_Open
#define IOPORT_CFG_CTRL g_ioport_ctrl

/* IOPORT Instance */
extern const ioport_instance_t g_ioport;

/* IOPORT control structure. */
extern ioport_instance_ctrl_t g_ioport_ctrl;
extern SemaphoreHandle_t _SemaphoreVsync;
extern EventGroupHandle_t g_i2c_event_group;
extern SemaphoreHandle_t g_irq_binary_semaphore;
extern SemaphoreHandle_t g_mipi_binary_semaphore;
void g_common_init(void);
FSP_FOOTER
#endif /* COMMON_DATA_H_ */
