/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2020- 2021 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
 * Modified by Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This file implements the SWD specific functions of the
 * ARM Debug Interface v5 Architecture Specification, ARM doc IHI0031A.
 */

#include "general.h"
#include "exception.h"
#include "adiv5.h"
#include "swd.h"
#include "target.h"
#include "target_internal.h"

extern void swdptap_seq_out_buffer(const uint32_t *tms_states, size_t clock_cycles);
extern uint8_t rp2040_pio_adiv5_swd_write_no_check(uint8_t request, uint32_t data);
extern uint8_t rp2040_pio_adiv5_swd_read_no_check(uint8_t request, uint32_t *data);
extern uint8_t rp2040_pio_adiv5_swd_write_check(uint8_t request, uint32_t data);
extern uint8_t rp2040_pio_adiv5_swd_read_check(uint8_t request, uint32_t *data, bool *parity);

extern uint8_t rp2040_pio_adiv5_swd_raw_access_req(uint8_t request);
extern uint8_t rp2040_pio_adiv5_swd_raw_access_data(uint32_t data_out, uint32_t *data_in, uint8_t rnw);

uint8_t make_packet_request(uint8_t rnw, uint16_t addr)
{
	bool is_ap = addr & ADIV5_APnDP;

	addr &= 0xffU;

	uint8_t request = 0x81U; /* Park and Startbit */

	if (is_ap)
		request ^= 0x22U;
	if (rnw)
		request ^= 0x24U;

	addr &= 0xcU;
	request |= (addr << 1U) & 0x18U;
	if (addr == 4U || addr == 8U)
		request ^= 0x20U;

	return request;
}

/* Provide bare DP access functions without timeout and exception */

static void swd_line_reset_sequence(const bool idle_cycles)
{
	/*
	 * A line reset is achieved by holding the SWDIOTMS HIGH for at least 50 SWCLKTCK cycles, followed by at least two idle cycles
	 * Note: in some non-conformant devices (STM32) at least 51 HIGH cycles and/or 3/4 idle cycles are required
	 *
	 * for robustness, we use 60 HIGH cycles and 4 idle cycles
	 */
	const uint32_t data[] = {0xffffffffU, 0x0fffffffU};
	swdptap_seq_out_buffer(data, 32 + (idle_cycles ? 32U : 28U));
}

/* Switch out of dormant state into SWD */
static void dormant_to_swd_sequence(void)
{
	/*
	 * ARM Debug Interface Architecture Specification, ADIv5.0 to ADIv5.2. ARM IHI 0031C
	 * §5.3.4 Switching out of Dormant state
	 */

	DEBUG_INFO("Switching out of dormant state into SWD\n");

	/* Send at least 8 SWCLKTCK cycles with SWDIOTMS HIGH */
	swd_line_reset_sequence(false);
	/* Send the 128-bit Selection Alert sequence on SWDIOTMS */
	/*
	 * We combine the last two sequences in a single seq_out as an optimization
	 *
	 * Send 4 SWCLKTCK cycles with SWDIOTMS LOW
	 * Send the required 8 bit activation code sequence on SWDIOTMS
	 *
	 * The bits are shifted out to the right, so we shift the second sequence left by the size of the first sequence
	 * The first sequence is 4 bits and the second 8 bits, totaling 12 bits in the combined sequence
	 */
	const uint32_t data[] = {ADIV5_SELECTION_ALERT_SEQUENCE_0, ADIV5_SELECTION_ALERT_SEQUENCE_1,
		ADIV5_SELECTION_ALERT_SEQUENCE_2, ADIV5_SELECTION_ALERT_SEQUENCE_3, ADIV5_ACTIVATION_CODE_ARM_SWD_DP << 4U};
	swdptap_seq_out_buffer(data, (32U * 4) + 12U);

	/*
	 * The target is in the protocol error state after selecting SWD
	 * Ensure the interface is in a known state by performing a line reset
	 */
	swd_line_reset_sequence(true);
}

/* Deprecated JTAG-to-SWD select sequence */
static void jtag_to_swd_sequence(void)
{
	/*
	 * ARM Debug Interface Architecture Specification, ADIv5.0 to ADIv5.2. ARM IHI 0031C
	 * §5.2.1 Switching from JTAG to SWD operation
	 */

	/* ARM deprecates use of these sequences on devices where the dormant state of operation is implemented */
	DEBUG_WARN("Deprecated JTAG to SWD sequence\n");

	/* SWD interface must be in reset state */
	swd_line_reset_sequence(false);

	/* Send the 16-bit JTAG-to-SWD select sequence on SWDIO/TMS */
	swd_proc.seq_out(ADIV5_JTAG_TO_SWD_SELECT_SEQUENCE, 16U);

	/*
	 * This ensures that if a SWD/JTAG DP was already in SWD operation before sending the select sequence,
	 * the interface enters reset state
	 */
	swd_line_reset_sequence(true);
}

bool adiv5_swd_write_no_check(const uint16_t addr, const uint32_t data)
{
	const uint8_t request = make_packet_request(ADIV5_LOW_WRITE, addr);
	return rp2040_pio_adiv5_swd_write_no_check(request, data) != SWD_ACK_OK;
}

uint32_t adiv5_swd_read_no_check(const uint16_t addr)
{
	const uint8_t request = make_packet_request(ADIV5_LOW_READ, addr);
	uint32_t data = 0;
	const uint8_t res = rp2040_pio_adiv5_swd_read_no_check(request, &data);
	return res == SWD_ACK_OK ? data : 0;
}

bool adiv5_swd_scan(const uint32_t targetid)
{
	/* Free the device list if any */
	target_list_free();

	adiv5_debug_port_s *dp = calloc(1, sizeof(*dp));
	if (!dp) { /* calloc failed: heap exhaustion */
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return false;
	}

	dp->write_no_check = adiv5_swd_write_no_check;
	dp->read_no_check = adiv5_swd_read_no_check;
	dp->dp_read = adiv5_swd_read;
	dp->low_access = adiv5_swd_raw_access;
	dp->error = adiv5_swd_clear_error;
	dp->abort = adiv5_swd_abort;

#if PC_HOSTED == 0
	swdptap_init();
#else
	if (!bmda_swd_dp_init(dp)) {
		free(dp);
		return false;
	}
#endif

	platform_target_clk_output_enable(true);

	/* Switch out of dormant state */
	dormant_to_swd_sequence();

	uint32_t dp_targetid = targetid;

	if (!dp_targetid) {
		/* No targetID given on the command line Try to read ID */
		/*
		 * ARM Debug Interface Architecture Specification, ADIv5.0 to ADIv5.2. ARM IHI 0031C
		 *
		 * §4.2.6 Limitations of multi-drop
		 *
		 * It is not possible to interrogate a multi-drop Serial Wire Debug system that includes multiple devices to establish
		 * which devices are connected. Because all devices are selected on coming out of a line reset, no communication with
		 * a device is possible without prior selection of that target using its target ID. Therefore, connection to a multi-drop
		 * Serial Wire Debug system that includes multiple devices requires that either:
		 * - The host has prior knowledge of the devices in the system and is configured before target connection.
		 * - The host attempts auto-detection by issuing a target select command for each of the devices it has been
		 * configured to support.
	 	 */

		/* Read DPIDR, if the first read fails, try the JTAG to SWD sequence, if that fails, give up */
		uint32_t dpidr = 0;
		bool tried_jtag_to_swd = false;
		while (true) {
			dpidr = adiv5_dp_read_dpidr(dp);
			if (dpidr != 0)
				/* Successfully read the DPIDR */
				break;

			if (!tried_jtag_to_swd) {
				jtag_to_swd_sequence();
				dp->fault = 0;

				tried_jtag_to_swd = true;

				/* Try again */
				continue;
			}

			/* Give up */
			DEBUG_ERROR("No usable DP found\n");
			free(dp);
			return false;
		}

		/* DP must have the version field set so adiv5_dp_read() does protocol recovery correctly */
		dp->version = (dpidr & ADIV5_DP_DPIDR_VERSION_MASK) >> ADIV5_DP_DPIDR_VERSION_OFFSET;
		if (dp->version >= 2U) {
			/* Read TargetID. Can be done with device in WFI, sleep or reset! */
			/* TARGETID is on bank 2 */
			adiv5_dp_write(dp, ADIV5_DP_SELECT, ADIV5_DP_BANK2);
			dp_targetid = adiv5_dp_read(dp, ADIV5_DP_TARGETID);
			adiv5_dp_write(dp, ADIV5_DP_SELECT, ADIV5_DP_BANK0);
		}
	}

	/* If we were given targetid or we have a DPv2+ device, do a multi-drop scan */
#if PC_HOSTED == 0
	/* On non hosted platforms, scan_multidrop can be constant */
	const
#endif
		bool scan_multidrop = targetid || dp->version >= 2U;

#if PC_HOSTED == 1
	if (scan_multidrop && !dp->write_no_check) {
		DEBUG_WARN("Discovered multi-drop enabled target but CMSIS_DAP < v1.2 cannot handle multi-drop\n");
		scan_multidrop = false;
	}
#endif

	if (scan_multidrop)
		adiv5_swd_multidrop_scan(dp, dp_targetid);
	else {
		adiv5_dp_abort(dp, ADIV5_DP_ABORT_STKERRCLR);
		adiv5_dp_init(dp);
	}

	return target_list != NULL;
}

/*
 * ARM Debug Interface Architecture Specification, ADIv5.0 to ADIv5.2. ARM IHI 0031C
 *
 * §4.2.6 Limitations of multi-drop
 *
 * Each device must be configured with a unique target ID, that includes a 4-bit instance ID, to differentiate between
 * otherwise identical targets. This places a limit of 16 such targets in any system, and means that identical devices
 * must be configured before they are connected together to ensure that their instance IDs do not conflict.
 * Auto-detection of the target
 *
 * It is not possible to interrogate a multi-drop Serial Wire Debug system th at includes multiple devices to establish
 * which devices are connected. Because all devices are selected on coming out of a line reset, no communication with
 * a device is possible without prior selection of that target using its target ID. Therefore, connection to a multi-drop
 * Serial Wire Debug system that includes multiple devices requires that either:
 * - The host has prior knowledge of the devices in the system and is configured before target connection.
 * - The host attempts auto-detection by issuing a target select command for each of the devices it has been
 * configured to support.
 *
 * This means that debug tools cannot connect seamlessly to targets in a multi-drop Serial Wire Debug system that they
 * have never seen before. However, if the debug tools can be provided with the target ID of such targets by the user
 * then the contents of the target can be auto-detected as normal.
 * To protect against multiple selected devices all driving the line simultaneously SWD protocol version 2 requires:
 * - For multi-drop SWD/JTAG DPs, the JTAG connection is selected out of powerup reset. JTAG does not drive the line.
 * - For multi-drop SWD DPs, the DP is in the dormant state out of powerup reset.
 */
void adiv5_swd_multidrop_scan(adiv5_debug_port_s *const dp, const uint32_t targetid)
{
	DEBUG_INFO("Handling SWD multi-drop, TARGETID 0x%08" PRIx32 "\n", targetid);

	/* Scan all 16 possible instances (4-bit instance ID) */
	for (size_t instance = 0; instance < 16U; instance++) {
		/*
		 * On a write to TARGETSEL immediately following a line reset sequence, the target is selected if both the following
		 * conditions are met:
		 * Bits [31:28] match bits [31:28] in the DLPIDR. (i.e. the instance ID matches)
		 * Bits [27:0] match bits [27:0] in the TARGETID register.
		 * Writing any other value deselects the target.
		 * During the response phase of a write to the TARGETSEL register, the target does not drive the line
		 */

		/* Line reset sequence */
		swd_line_reset_sequence(true);
		dp->fault = 0;

		/* Select the instance */
		dp->write_no_check(ADIV5_DP_TARGETSEL,
			instance << ADIV5_DP_TARGETSEL_TINSTANCE_OFFSET |
				(targetid & (ADIV5_DP_TARGETID_TDESIGNER_MASK | ADIV5_DP_TARGETID_TPARTNO_MASK)) | 1U);

		/* Read DPIDR */
		if (adiv5_dp_read_dpidr(dp) == 0)
			/* No DP here, next instance */
			continue;

		/* Allocate a new target DP for this instance */
		adiv5_debug_port_s *const target_dp = calloc(1, sizeof(*dp));
		if (!target_dp) { /* calloc failed: heap exhaustion */
			DEBUG_ERROR("calloc: failed in %s\n", __func__);
			break;
		}

		/* Populate the target DP from the initial one */
		memcpy(target_dp, dp, sizeof(*dp));
		target_dp->dev_index = instance;

		/* Yield the target DP to adiv5_dp_init */
		adiv5_dp_abort(target_dp, ADIV5_DP_ABORT_STKERRCLR);
		adiv5_dp_init(target_dp);
	}

	/* free the initial DP */
	free(dp);
}

uint32_t adiv5_swd_read(adiv5_debug_port_s *dp, uint16_t addr)
{
	if (addr & ADIV5_APnDP) {
		adiv5_dp_recoverable_access(dp, ADIV5_LOW_READ, addr, 0);
		return adiv5_dp_low_access(dp, ADIV5_LOW_READ, ADIV5_DP_RDBUFF, 0);
	}
	return adiv5_dp_recoverable_access(dp, ADIV5_LOW_READ, addr, 0);
}

uint32_t adiv5_swd_clear_error(adiv5_debug_port_s *const dp, const bool protocol_recovery)
{
	/* Only do the comms reset dance on DPv2+ w/ fault or to perform protocol recovery. */
	if ((dp->version >= 2U && dp->fault) || protocol_recovery) {
		/*
		 * Note that on DPv2+ devices, during a protocol error condition
		 * the target becomes deselected during line reset. Once reset,
		 * we must then re-select the target to bring the device back
		 * into the expected state.
		 */
		swd_line_reset_sequence(true);
		if (dp->version >= 2U)
			adiv5_write_no_check(dp, ADIV5_DP_TARGETSEL, dp->targetsel);
		adiv5_read_no_check(dp, ADIV5_DP_DPIDR);
		/* Exception here is unexpected, so do not catch */
	}
	const uint32_t err = adiv5_read_no_check(dp, ADIV5_DP_CTRLSTAT) &
		(ADIV5_DP_CTRLSTAT_STICKYORUN | ADIV5_DP_CTRLSTAT_STICKYCMP | ADIV5_DP_CTRLSTAT_STICKYERR |
			ADIV5_DP_CTRLSTAT_WDATAERR);
	uint32_t clr = 0;

	if (err & ADIV5_DP_CTRLSTAT_STICKYORUN)
		clr |= ADIV5_DP_ABORT_ORUNERRCLR;
	if (err & ADIV5_DP_CTRLSTAT_STICKYCMP)
		clr |= ADIV5_DP_ABORT_STKCMPCLR;
	if (err & ADIV5_DP_CTRLSTAT_STICKYERR)
		clr |= ADIV5_DP_ABORT_STKERRCLR;
	if (err & ADIV5_DP_CTRLSTAT_WDATAERR)
		clr |= ADIV5_DP_ABORT_WDERRCLR;

	if (clr)
		adiv5_write_no_check(dp, ADIV5_DP_ABORT, clr);
	dp->fault = 0;
	return err;
}

uint32_t adiv5_swd_raw_access(adiv5_debug_port_s *dp, const uint8_t rnw, const uint16_t addr, const uint32_t value)
{
	if ((addr & ADIV5_APnDP) && dp->fault)
		return 0;

	const uint8_t request = make_packet_request(rnw, addr);
	uint32_t response = 0;
	uint8_t ack = SWD_ACK_WAIT;
	bool parity = false;
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 250U);
	do {
		if (rnw) {
			ack = rp2040_pio_adiv5_swd_read_check(request, &response, &parity);
		} else {
			ack = rp2040_pio_adiv5_swd_write_check(request, value);
		}
		if (ack == SWD_ACK_FAULT) {
			DEBUG_ERROR("SWD access resulted in fault, retrying\n");
			/* On fault, abort the request and repeat */
			/* Yes, this is self-recursive.. no, we can't think of a better option */
			adiv5_dp_write(dp, ADIV5_DP_ABORT,
				ADIV5_DP_ABORT_ORUNERRCLR | ADIV5_DP_ABORT_WDERRCLR | ADIV5_DP_ABORT_STKERRCLR |
					ADIV5_DP_ABORT_STKCMPCLR);
		}
	} while ((ack == SWD_ACK_WAIT || ack == SWD_ACK_FAULT) && !platform_timeout_is_expired(&timeout));

	if (ack == SWD_ACK_WAIT) {
		DEBUG_ERROR("SWD access resulted in wait, aborting\n");
		dp->abort(dp, ADIV5_DP_ABORT_DAPABORT);
		dp->fault = ack;
		return 0;
	}

	if (ack == SWD_ACK_FAULT) {
		DEBUG_ERROR("SWD access resulted in fault\n");
		dp->fault = ack;
		return 0;
	}

	if (ack == SWD_ACK_NO_RESPONSE) {
		DEBUG_ERROR("SWD access resulted in no response\n");
		dp->fault = ack;
		return 0;
	}

	if (ack != SWD_ACK_OK) {
		DEBUG_ERROR("SWD access has invalid ack %x\n", ack);
		raise_exception(EXCEPTION_ERROR, "SWD invalid ACK");
	}

	if ((rnw) && (!parity)) {
		dp->fault = 1U;
		DEBUG_ERROR("SWD access resulted in parity error\n");
		raise_exception(EXCEPTION_ERROR, "SWD parity error");
	}

	return response;
}

void adiv5_swd_abort(adiv5_debug_port_s *dp, uint32_t abort)
{
	adiv5_dp_write(dp, ADIV5_DP_ABORT, abort);
}