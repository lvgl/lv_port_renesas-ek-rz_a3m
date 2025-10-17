/* generated pin source file - do not edit */
#include "bsp_api.h"
#include "r_ioport_api.h"
#if __has_include("r_ioport.h")
#include "r_ioport.h"
#endif

extern const ioport_extend_cfg_t g_ioport_cfg_extend;

const ioport_pin_cfg_t g_bsp_pin_cfg_data[] = { { .pin = BSP_IO_PORT_01_PIN_00,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
				| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
				| (uint32_t) IOPORT_PERIPHERAL_MODE2) }, { .pin =
		BSP_IO_PORT_01_PIN_01, .pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
		| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
		| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
		| (uint32_t) IOPORT_PERIPHERAL_MODE2) }, { .pin = BSP_IO_PORT_08_PIN_01,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
				| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
				| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin =
		BSP_IO_PORT_20_PIN_00, .pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
		| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
		| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
		| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin = BSP_IO_PORT_20_PIN_01,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
				| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
				| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin =
		BSP_IO_PORT_20_PIN_02, .pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
		| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
		| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
		| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin = BSP_IO_PORT_20_PIN_03,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
				| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
				| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin =
		BSP_IO_PORT_20_PIN_04, .pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
		| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
		| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
		| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin = BSP_IO_PORT_20_PIN_05,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
				| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
				| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin =
		BSP_IO_PORT_21_PIN_00, .pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
		| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
		| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
		| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin = BSP_IO_PORT_21_PIN_01,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
				| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
				| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin =
		BSP_IO_PORT_21_PIN_02, .pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
		| (uint32_t) IOPORT_CFG_PERIPHERAL_PIN
		| (uint32_t) IOPORT_CFG_SLEW_RATE_FAST
		| (uint32_t) IOPORT_PERIPHERAL_MODE1) }, { .pin = BSP_IO_TMS_SWDIO,
		.pin_cfg = ((uint32_t) IOPORT_CFG_DRIVE_B01
				| (uint32_t) IOPORT_CFG_SPECIAL_PURPOSE_PORT_INPUT_ENABLE) }, };

const ioport_cfg_t g_bsp_pin_cfg = { .number_of_pins =
		sizeof(g_bsp_pin_cfg_data) / sizeof(ioport_pin_cfg_t), .p_pin_cfg_data =
		&g_bsp_pin_cfg_data[0], .p_extend = &g_ioport_cfg_extend };
