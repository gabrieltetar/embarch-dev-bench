/*
 * UICR REGOUT0 = 3.3 V for an nRF52840 DK powered from its native USB.
 *
 * In high-voltage mode (supply on VDDH, which is how native USB powers the
 * chip) the output of regulator stage 0 is VDD for the whole board, and
 * REGOUT0 selects its level. Its erased default is 1.8 V, which cannot
 * supply an external analog front end wired to the DK's VDD. The register is
 * UICR, so it survives resets and reflashing of the image but not a chip
 * erase; this writes it once when it still holds the default and resets,
 * because the new level only applies after a reset.
 *
 * Same pattern as Zephyr's boards/nordic/nrf52840dongle/board.c (which sets
 * 3.0 V for the dongle's LEDs).
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(board_regout0, LOG_LEVEL_INF);

static bool regout0_is_default(void)
{
	return (NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
	       (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos);
}

static void nvmc_wait_ready(void)
{
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
}

/* Sets UICR.REGOUT0.VOUT to 3.3 V, leaving the other bits of the register. */
static void regout0_write_3v3(void)
{
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
	nvmc_wait_ready();

	NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
			    (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);
	nvmc_wait_ready();

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
	nvmc_wait_ready();
}

/* Runs before the kernel starts (EARLY), so nothing else has used VDD yet. */
static int board_regout0_init(void)
{
	if (nrf_power_mainregstatus_get(NRF_POWER) != NRF_POWER_MAINREGSTATUS_HIGH) {
		return 0;
	}

	if (!regout0_is_default()) {
		return 0;
	}

	regout0_write_3v3();

	/* The new level only applies after a reset. */
	NVIC_SystemReset();

	return 0;
}

SYS_INIT(board_regout0_init, EARLY, 0);
