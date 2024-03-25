/*
 * vectrl_atmega_usb.c
 *
 * Created: 18/11/2023 17:23:17
 * Author : Alexandre
 */ 

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb.h"

// Encoder state machine logic based on
// http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
// This has the advantage of ignoring input jitter and noise by only
// allowing valid state changes

// possible states
#define ENC_STATE_START      0x0
#define ENC_STATE_CW_BEGIN   0x1
#define ENC_STATE_CW_MIDDLE  0x2
#define ENC_STATE_CW_END     0x3
#define ENC_STATE_CCW_BEGIN  0x4
#define ENC_STATE_CCW_MIDDLE 0x5
#define ENC_STATE_CCW_END    0x6
// flags added to the state when rotation occurs
#define ENC_TURNED_CW        0x10
#define ENC_TURNED_CCW       0x20

const uint8_t ENC_STATE_TABLE[7][4] = {
	// ENC_STATE_START
	{ENC_STATE_START     , ENC_STATE_CCW_BEGIN , ENC_STATE_CW_BEGIN  , ENC_STATE_START},
	// ENC_STATE_CW_BEGIN
	{ENC_STATE_CW_MIDDLE , ENC_STATE_START     , ENC_STATE_CW_BEGIN  , ENC_STATE_START},
	// ENC_STATE_CW_MIDDLE
	{ENC_STATE_CW_MIDDLE , ENC_STATE_CW_END    , ENC_STATE_CW_BEGIN  , ENC_STATE_START},
	// ENC_STATE_CW_END
	{ENC_STATE_CW_MIDDLE , ENC_STATE_CW_END    , ENC_STATE_START     , ENC_STATE_START | ENC_TURNED_CW},
	// ENC_STATE_CCW_BEGIN
	{ENC_STATE_CCW_MIDDLE, ENC_STATE_CCW_BEGIN , ENC_STATE_START     , ENC_STATE_START},
	// ENC_STATE_CCW_MIDDLE
	{ENC_STATE_CCW_MIDDLE, ENC_STATE_CCW_BEGIN , ENC_STATE_CCW_END   , ENC_STATE_START},
	// ENC_STATE_CCW_END
	{ENC_STATE_CCW_MIDDLE, ENC_STATE_START     , ENC_STATE_CCW_END   , ENC_STATE_START | ENC_TURNED_CCW},
};

const uint8_t BUTTON_MAP[4][3] = {
	{
		1, 5, 7
	},
	{
		2, 0, 0
	},
	{
		3, 6, 8,
	},
	{
		4, 9, 0
	}
};

volatile int8_t enc0_counter = 0;
volatile uint8_t enc0_state = 0;

void process_encoder() {
	uint8_t pins = PIND & 0x03;
	enc0_state = ENC_STATE_TABLE[enc0_state & 0xF][pins];
	if (enc0_state & ENC_TURNED_CCW) {
		enc0_counter--;
	} else if (enc0_state & ENC_TURNED_CW) {
		enc0_counter++;
	}
}

ISR(INT0_vect) {
	process_encoder();
}

ISR(INT1_vect) {
	process_encoder();
}

int main(void)
{
	uint8_t pressed_btns[9];
	uint8_t pressed_btn_count = 0;
	uint8_t btn_port = 0;
	int8_t enc0_local = 0;
	// enables pull-up resistors globally
	MCUCR &= ~(1 << PUD);
	// sets up port D (board LED and encoder)
	DDRD = 1 << 5; // LED set as output
	PORTD = 0x00;
	// sets up port B (button rows)
	DDRB = 0; // all pins set as inputs
	// sets up port F (button columns)
	DDRF = 0x00; // F4 - F7 as inputs
	PORTF = 0xF0; // Activates pull-ups for F4-F7
	// initializes USB	
	usb_init();
	// sets up interrupts
	cli();
	EICRA = (1 << ISC00) | (0 << ISC01) | (1 << ISC10) | (0 << ISC11); // any edge on INT0 and INT1 (PD0 and PD1)
	EIMSK = (1 << INT0) | (1 << INT1); // enables INT0 and INT1
	sei();

	while (!get_usb_config_status()) {
		// LED animation
		PORTD ^=(1<<5);
		_delay_ms(100);
		PORTD ^=(1<<5);
		_delay_ms(100);
	}
	
	while(1)
	{
		pressed_btn_count = 0;
		/*if (btn_port == 0) {
			PORTD &= ~(1<<5);
		} else {
			PORTD |= (1<<5);
		}*/
		
		for (uint8_t row = 0; row < 3; row++) {
			uint8_t rowBit = 1 << (row + 1);
			DDRB = rowBit; // Sets column as output
			PORTB = 0x0E & ~rowBit; // Sets column to low
			_delay_ms(1);
			btn_port = (~(PINF >> 4)) & 0x0F;
			for (uint8_t col=0; col < 4; col++) {			
				uint8_t button_index = BUTTON_MAP[col][row];
				if ((btn_port & 1)!= 0 && button_index > 0) {
					pressed_btns[pressed_btn_count] = button_index;
					pressed_btn_count++;
				}
				btn_port >>= 1;
			}
		}

		cli();
		enc0_local = enc0_counter;
		enc0_counter = 0;
		sei();
		usb_send(enc0_local, pressed_btns, pressed_btn_count);
		_delay_ms(100);
	}
}

