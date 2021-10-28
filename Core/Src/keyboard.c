/*
 * keyboard.c
 *
 *  Created on: Oct 27, 2021
 *      Author: Toni
 */
#include <stdint.h>

#include "main.h"

// Possible key states
typedef enum {
	KEY_IS_UP, KEY_IS_DOWN, KEY_IS_GOING_UP, // We increment the timer in this state
	KEY_IS_GOING_DOWN,  // We increment the timer in this state
} state_t;

// Possible events
typedef enum {
	KEY_PRESSED,    // Key is pressed
	KEY_DOWN,       // Key reached bottom
	KEY_RELEASED,   // Key was released
	KEY_UP,         // Key reached top
} event_t;

typedef struct {
	char 	 midi_note;
	state_t  state :4; 	// Bit fields
	uint32_t t; 		// Lines up nicely to 16bits, t overflows at 4096
} midikey_t;

typedef struct {
	uint8_t top;
	uint8_t bottom;
} bank_t;
#define NUM_KEYS 61

midikey_t keys[NUM_KEYS];

#define NUM_BANKS 8

// For scanning banks
bank_t banks[NUM_BANKS];
bank_t prev_banks[NUM_BANKS];
uint8_t program = 0;
uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
uint8_t const channel = 1; // 0 for channel 1
//Velocity curves
//To save on calculation effort for the CPU, these are precalculated in Excel, the index is the number of milliseconds
//between contact, 0-80ms.
#define MAX_VEL_CURVE_INDEX 80
const uint8_t linearCurve[MAX_VEL_CURVE_INDEX+1]={127, 127, 127, 125, 124, 122, 121, 119, 118, 116, 114, 113, 111, 110, 108, 107, 105, 103, 102, 100, 99, 97, 95, 94, 92, 91, 89, 88, 86, 84, 83, 81, 80, 78, 77, 75, 73, 72, 70, 69, 67, 66, 64, 62, 61, 59, 58, 56, 54, 53, 51, 50, 48, 47, 45, 43, 42, 40, 39, 37, 36, 34, 32, 31, 29, 28, 26, 25, 23, 21, 20, 18, 17, 15, 13, 12, 10, 9, 7, 6, 4};
const uint8_t convexCurve[MAX_VEL_CURVE_INDEX+1]={127, 127, 127, 127, 127, 127, 127, 126, 126, 126, 125, 125, 125, 124, 123, 123, 122, 121, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 109, 108, 107, 105, 104, 102, 101, 99, 98, 96, 94, 93, 91, 89, 87, 86, 84, 82, 80, 78, 76, 74, 72, 70, 68, 66, 63, 61, 59, 57, 54, 52, 50, 48, 45, 43, 41, 38, 36, 33, 31, 29, 26, 24, 21, 19, 16, 14, 11, 9, 6, 4};
const uint8_t saturatedCurve[MAX_VEL_CURVE_INDEX+1]={127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 126, 125, 125, 124, 122, 121, 119, 118, 116, 114, 111, 109, 106, 104, 101, 98, 94, 91, 87, 84, 80, 76, 72, 68, 64, 60, 55, 51, 47, 42, 37, 33, 28, 23, 18, 14, 9, 4};
const uint8_t concaveCurve[MAX_VEL_CURVE_INDEX+1]={127, 127, 127, 122, 116, 111, 106, 102, 97, 93, 88, 84, 80, 76, 73, 69, 66, 62, 59, 56, 53, 50, 48, 45, 43, 40, 38, 36, 34, 32, 30, 28, 26, 25, 23, 22, 21, 19, 18, 17, 16, 15, 14, 13, 12, 11, 11, 10, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
const uint8_t* velocityCurves[]={
  linearCurve,
  convexCurve,
  saturatedCurve,
  concaveCurve
};

void initialize() {
	tu_printf("%s\n", __func__);

	// Init keys
	for (int key = 0; key < NUM_KEYS; key++) {
		keys[key].midi_note = 24 + key;
		keys[key].t = 0;
	}

	for (int i = 0; i < NUM_BANKS; i++) {
		banks[i].top = 0;
		banks[i].bottom = 1;
	}
	memcpy(prev_banks, banks, sizeof(prev_banks));

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim2);
}

void trigger(midikey_t *key, event_t event) {
	uint32_t elapsed;
	uint8_t  velocity;
	const uint8_t* curve=velocityCurves[1];

	if (event == KEY_PRESSED) {
		key->state = KEY_IS_GOING_DOWN;
		tu_printf("GOING DOWN %d\n", key->midi_note);
		key->t = HAL_GetTick();
	} else if (event == KEY_DOWN) {
		if (key->state == KEY_IS_GOING_DOWN) {
			key->state = KEY_IS_DOWN;
			elapsed = HAL_GetTick() - key->t;
			velocity=curve[tu_min32(MAX_VEL_CURVE_INDEX, elapsed)];
			// Send Note On on channel 1.
			uint8_t note_on[3] = { 0x90 | channel, key->midi_note, velocity };
			tud_midi_stream_write(cable_num, note_on, 3);
			tu_printf("DN %d %d %ld\n", key->midi_note, velocity, elapsed);
			key->t = 0;
		}
	} else if (event == KEY_RELEASED) {
		key->state = KEY_IS_GOING_UP;
		tu_printf("GOING UP %d\n", key->midi_note);
		key->t = HAL_GetTick();
	} else if (event == KEY_UP) {
		if (key->state == KEY_IS_GOING_UP) {
			key->state = KEY_IS_UP;
			elapsed = HAL_GetTick() - key->t;
			velocity=curve[tu_min32(MAX_VEL_CURVE_INDEX, elapsed)];
			// Send Note Off on channel 1.
			uint8_t note_off[3] = { 0x80 | channel, key->midi_note, velocity };
			tud_midi_stream_write(cable_num, note_off, 3);
			tu_printf("UP %d %d %ld\n", key->midi_note, velocity, elapsed);
			key->t = 0;
		}
	}
}

void increment() {
	// Advance timers
	for (int key = 0; key < NUM_KEYS; key++) {
		state_t state = keys[key].state;
		if (state == KEY_IS_GOING_UP || state == KEY_IS_GOING_DOWN) {
			if (keys[key].t < 126)
				keys[key].t++;
		}
	}
}

//void delay_us (uint16_t us)
//{
//	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
//}

void delay_us(uint32_t delay_us) {
	volatile unsigned int num;
	volatile unsigned int t;

	for (num = 0; num < delay_us; num++) {
		t = 11;
		while (t != 0) {
			t--;
		}
	}
}

uint32_t deb[NUM_BANKS];
void scan(void) {
	uint8_t count[16];
	uint32_t val[3];
	int i;

	// Scan and store
	for (int bank = 0; bank < NUM_BANKS; bank++) {
		prev_banks[bank] = banks[bank]; // Store previous state so we can look for changes

		GPIOA->ODR |= (1 << bank) & 0xff; // Selects bottom row
		delay_us(10); // wait voltage stabilize
		for (i = 0; i < 3; i++) {
			val[i] = GPIOB->IDR;
			delay_us(10); // wait voltage stabilize
		}

		for (i = 0; i < 16; i++) {
			count[i] = 0;
			if (val[0] & 1 << i)
				++count[i];
			if (val[1] & 1 << i)
				++count[i];
			if (val[2] & 1 << i)
				++count[i];
			if (count[i] == 3) {
				deb[bank] |= (1 << i);
			} else {
				deb[bank] &= ~(1 << i);
			}
		}

		banks[bank].bottom = deb[bank] & 0xff;
		banks[bank].top = deb[bank] >> 8;
		GPIOA->ODR &= ~((1 << bank) & 0xff); // Selects bottom row
	}

	// Process
	for (int bank = 0; bank < NUM_BANKS; bank++) {

		uint8_t diff;

		// Check top switches and fire events
		diff = banks[bank].top ^ prev_banks[bank].top;
		if (diff) {
			for (int key = 0; key < 8; key++) {
				if (diff & (1 << key)) {
					event_t event =
							banks[bank].top & (1 << key) ? KEY_UP : KEY_PRESSED;
					trigger(&keys[bank + 8 * key], event);
				}
			}
		}

		// Check bottom switches and fire events
		diff = banks[bank].bottom ^ prev_banks[bank].bottom;
		if (diff) {
			for (int key = 0; key < 8; key++) {
				if (diff & (1 << key)) {
					event_t event =
							banks[bank].bottom & (1 << key) ?
									KEY_DOWN : KEY_RELEASED;
					trigger(&keys[bank + 8 * key], event);
				}
			}
		}
	}
}

void encoder(void) {
	uint32_t encoder_val;
	static uint32_t old_encoder = 0;
	static uint32_t now = 0;
	static int update = 0;

	encoder_val = (TIM1->CNT) >> 2;

	if (old_encoder != encoder_val) {
		now = HAL_GetTick();
		update = 1;
		old_encoder = encoder_val;
	}

	if (HAL_GetTick() - now > 200 && update) {
		program = encoder_val % 127;
		// Send program change on channel 1.
		uint8_t prog_change[2] = { 0xC0 | channel, program };
		tud_midi_stream_write(cable_num, prog_change, 2);
		tu_printf("PC %d\n", program);
		update = 0;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2) {
		//scan();
		increment();
	}
}

void midi_task() {
	scan();
	encoder();
}

