/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* AT90CAN128 driver */

#include <stdio.h>
#include <inttypes.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <csp/csp.h>
#include <csp/arch/csp_semaphore.h>
#include <csp/interfaces/csp_if_can.h>

#include <csp/drivers/can.h>
#include "can_at90can128.h"

/* MOB segmentation */
#define CAN_RX_MOBS 7
#define CAN_TX_MOBS 8
#define CAN_MOBS	15

/** Identifier and mask */
uint32_t can_id;
uint32_t can_mask;

/** Mailbox */
typedef enum {
	MBOX_FREE = 0,
	MBOX_USED = 1,
} mbox_t;

/* fcpu and bitrate */
static unsigned long int clock_speed;
static uint32_t bitrate;

/** List of mobs */
static mbox_t mbox[CAN_TX_MOBS];

/** Mailbox signalling semaphore */
csp_bin_sem_handle_t tx_sem;

void can_configure_mobs(void) {

	int mob;

	/* Initialize MOBs */
	for (mob = 0; mob < CAN_MOBS; mob++) {
		/* Set MOB */
		CAN_SET_MOB(mob);
		CAN_CLEAR_MOB();

		/* Only RX mobs require special initialization */
		if (mob >= CAN_TX_MOBS) {
			/* Set id and mask */
			CAN_SET_EXT_ID(can_id);
			CAN_SET_EXT_MSK(can_mask);

			/* Accept extended frames */
			CAN_CLEAR_IDEMSK();
			CAN_CONFIG_RX();
		}

	}
}

int can_configure_bitrate(unsigned long int afcpu, uint32_t bps) {

	/* Set baud rate (500 kbps) */
	if (bps != 500000) {
		csp_log_error("CAN bitrate must be 500000 (was %d)\r\n");
		return -1;
	}

	/* TODO: Set arbitrary bitrate */
	if (afcpu == 8000000) {
		CANTCON = 0x02;	  // Set CAN Timer Prescaler
		CANBT1 = 0x02;	   //!< Tscl  = 2x Tclkio = 250 ns
		CANBT2 = 0x04;	   //!< Tsync = 1x Tscl, Tprs = 3x Tscl, Tsjw = 1x Tscl
		CANBT3 = 0x13;	   //!< Tpsh1 = 2x Tscl, Tpsh2 = 2x Tscl, 3 sample points
	} else if (afcpu == 12000000) {
		CANTCON = 0x02;	  // Set CAN Timer Prescaler
		CANBT1 = 0x02;	   //!< Tscl  = 2x Tclkio = 166.666 ns
		CANBT2 = 0x08;	   //!< Tsync = 1x Tscl, Tprs = 5x Tscl, Tsjw = 1x Tscl
		CANBT3 = 0x25;	   //!< Tpsh1 = 3x Tscl, Tpsh2 = 3x Tscl, 3 sample points
	} else if (afcpu == 16000000) {
		CANTCON = 128;	   // Set CAN Timer Prescaler
		CANBT1 = 0x06;	   //!< Tscl  = 4x Tclkio = 250 ns
		CANBT2 = 0x04;	   //!< Tsync = 1x Tscl, Tprs = 3x Tscl, Tsjw = 1x Tscl
		CANBT3 = 0x13;	   //!< Tpsh1 = 2x Tscl, Tpsh2 = 2x Tscl, 3 sample points
	} else {
		csp_log_error("Error, missing CAN driver defines for that FCPU=%d\r\n", afcpu);
		return -1;
	}

	return 0;

}

int can_reset(unsigned long int afcpu, uint32_t bps) {
	/* Configure CAN module */
	CAN_DISABLE();
	CAN_RESET();

	/* Enables mob 0-14 interrupts */
	CANIE1 = 0x7F;
	CANIE2 = 0xFF;

	/* Configure bitrate */
	can_configure_bitrate(afcpu, bps);

	/* Configure MOBS */
	can_configure_mobs();

	/* Enable and return */
	CAN_ENABLE();
	CAN_SET_INTERRUPT();

	return 0;

}

int can_init(uint32_t id, uint32_t mask, struct csp_can_config *conf) {

	csp_assert(conf && conf->bitrate && conf->clock_speed);

	/* Set id and mask */
	can_id = id;
	can_mask = mask;

	/* Set fcpu and bps */
	clock_speed = conf->clock_speed;
	bitrate = conf->bitrate;

	/* Create semaphore for signalling mailboxes */
	if(csp_bin_sem_create(&tx_sem) != CSP_SEMAPHORE_OK) {
		csp_log_error("Could not create tx_sem.");
		return -1;
	}

	return can_reset(clock_speed, bitrate);

}

int can_send(can_id_t id, uint8_t data[], uint8_t dlc) {

	int i, m = CAN_TX_MOBS;
	int trials = 0;

	while(m >= CAN_TX_MOBS && trials < 100) {
		/* Disable CAN interrupt while looping MOBs */
		CAN_CLEAR_INTERRUPT();

		/* Disable interrupts while looping mailboxes */
		portENTER_CRITICAL();

		/* Search free MOB from CAN_TX_MOBs -> 0  */
		/* Low MOB id is high priority. Since we must send in
		   the order can_send() is called, we must use an id which
		   is higher than the currently used ids. */
		m = 0; /* Fallback */
		for(i = CAN_TX_MOBS-1; i >= 0; i--) {
			if (mbox[i] == MBOX_USED || (CANEN2 & (1 << i))) {
				/* Use previous */
				m = i+1;
				break;
			}
		}
		if (m < CAN_TX_MOBS) {
			/* We found one */
			mbox[m] = MBOX_USED;
		}
	
		/* Enable interrupts */
		portEXIT_CRITICAL();

		/* Enable CAN interrupt */
		CAN_SET_INTERRUPT();

		if (m >= CAN_TX_MOBS) {
			/* We found none. Wait until new MOBs are available. */
			if(csp_bin_sem_wait(&tx_sem, 1000) == CSP_SEMAPHORE_ERROR) {
				/* Timeout */
				csp_log_warn("TX sem timeout\n");
				return -1;
			}
		}
		trials++;
	}

	/* Return if no available MOB was found */
	if (m >= CAN_TX_MOBS) {
		csp_log_warn("Too many trials, no available MOB\r\n");
		return -1;
	}

	/* Select and clear mob */
	CAN_SET_MOB(m);
	CAN_MOB_ABORT();
	CAN_CLEAR_STATUS_MOB();

	/* Set identifier */
	CAN_SET_EXT_ID(id);

	/* Set data - CANMSG is auto incrementing */
	for (i = 0; i < dlc; i++)
		CANMSG = data[i];

	/* Set DLC */
	CAN_CLEAR_DLC();
	CAN_SET_DLC(dlc);

	/* Start TX */
	CAN_CONFIG_TX();

	return 0;

}

/** Find Oldest MOB
 * Searches the CAN timestamp and returns the
 * can-mob with the oldest timestamp.
 */
static inline uint8_t can_find_oldest_mob(void) {

	static uint8_t mob, mob_winner;
	static uint16_t mob_time;
	static uint32_t time;
	static uint16_t diff, diff_highest;

	diff_highest = 0;
	for (mob = 0; mob <= 14; mob++) {
		CAN_SET_MOB(mob);
		if (CANSTMOB) {
			time = CANTIMH << 8 | CANTIML;
			mob_time = CANSTMH << 8 | CANSTML;

			/* Check for overflow */
			if (mob_time > time) {
				diff = (time + 0xFFFF) - mob_time;				
			} else {
				diff = time - mob_time;
			}
			if (diff >= diff_highest) {
				mob_winner = mob;
				diff_highest = diff;
			}
		}
	}

	return mob_winner;

}

/** Can interrupt vector 
 * This is the heart of the driver
 * In here frames are accepted and the hardware is freed
 * before returning from the ISR the frame is passed to the 
 * lower handlers of the protocol layer through a callback.
 */
ISR(CANIT_vect) {
	static can_id_t id;
	static uint8_t mob;
	static portBASE_TYPE xTaskWoken = pdFALSE;

	/* Handle general interrupt */
	if (CANGIT & INT_GEN_MSK) {
		CAN_CLEAR_GENINT();

		/* Reset controller and all interrupts */
		can_reset(clock_speed, bitrate);
	}
	
	/* For each MOB that has interrupted */
	while (CAN_HPMOB() != 0xF) {

		/* Locate MOB */
		mob = can_find_oldest_mob();
		CAN_SET_MOB(mob);

		if (CANSTMOB & ERR_MOB_MSK) {
			/* Error */
			CAN_MOB_ABORT();

			CAN_GET_EXT_ID(id);

			/* Remember to re-enable RX */
			if (mob >= CAN_TX_MOBS)
				CAN_CONFIG_RX();

			/* Release mailbox */
			mbox[mob] = MBOX_FREE;

			/* Clear status */
			CAN_CLEAR_STATUS_MOB();

			/* Report new MOBs available. This may fail
			   if there are no users waiting for the sem, but
			   we don't care. */
			csp_bin_sem_post_isr(&tx_sem, &xTaskWoken);

		} else if (CANSTMOB & MOB_RX_COMPLETED) {
			/* RX Complete */
			int i;
			can_frame_t frame;

			/* Clear status */
			CAN_CLEAR_STATUS_MOB();

			if (mob == CAN_MOBS - 1) {
				/* RX overflow */
				CAN_DISABLE();
				can_configure_mobs();
				CAN_ENABLE();
			}

			/* Read DLC */
			frame.dlc = CAN_GET_DLC();

			/* Read data */
			for (i = 0; i < frame.dlc; i++)
				frame.data[i] = CANMSG;

			/* Read identifier */
			CAN_GET_EXT_ID(frame.id);

			/* Do RX-Callback */
			csp_can_rx_frame(&frame, &xTaskWoken);

			/* The callback handler might have changed active mob */
			CAN_SET_MOB(mob);
			CAN_CONFIG_RX();

		} else if (CANSTMOB & MOB_TX_COMPLETED) {
			/* Clear status */
			CAN_CLEAR_STATUS_MOB();

			/* Release mailbox */
			mbox[mob] = MBOX_FREE;

			/* Signal new mailbox available */
			csp_bin_sem_post_isr(&tx_sem, &xTaskWoken);
		}
	}

	/* End of ISR */
	if (xTaskWoken == pdTRUE)
		taskYIELD();

}
