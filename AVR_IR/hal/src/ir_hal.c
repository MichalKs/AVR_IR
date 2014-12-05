/**
 * @file: 	ir_hal.c
 * @brief:	
 * @date: 	3 gru 2014
 * @author: Michal Ksiezopolski
 *
 *
 * @verbatim
 * Copyright (c) 2014 Michal Ksiezopolski.
 * All rights reserved. This program and the
 * accompanying materials are made available
 * under the terms of the GNU Public License
 * v3.0 which accompanies this distribution,
 * and is available at
 * http://www.gnu.org/licenses/gpl.html
 * @endverbatim
 */


#include <avr/io.h>
#include <avr/interrupt.h>

static void (*readDataCallback)(uint16_t pulseWidth, uint8_t edge); ///< Callback for sending received pulses to higher layer
static void (*resetFrameCallback)(void); ///< Callback for resetting frame if timeout occurs.

typedef enum {
  FALLING_EDGE,
  RISING_EDGE
} IR_EdgeTypedef;

static IR_EdgeTypedef edge = FALLING_EDGE;

/**
 * @brief Initialize hardware for decoding IR codes.
 *
 * @details IR receiver pin should be connected to ICP1 pin
 * (PD6) on ATmega32.
 *
 * @param readDataCb Callback for reading data
 * @param resetFrameCb Callback for resetting frame
 * @param timeout Frame timeout in us.
 */
void IR_HAL_Init(
    void (*readDataCb)(uint16_t pulseWidth, uint8_t edge),
    void (*resetFrameCb)(void),
    uint32_t timeout) {

  DDRD &= ~(1<<PD6); // input
  PORTD |= (1<<PD6); // pullup

  readDataCallback = readDataCb;
  resetFrameCallback = resetFrameCb;

  TCNT1 = 0; // zero out timer value

  TCCR1A = (1<<WGM12); // CTC mode
  TCCR1B = (1<<CS11); // prescaler 8-bit - counts every 2

  // input compare and compare interrupt
  TIMSK |= (1<<TICIE1) | (1<<OCIE1A);

  OCR1A = timeout<<1; // we count 0.5 us

  sei(); // enable global interrupts

}

/**
 * @brief Timer 1 overflow interrupt
 * @param TIMER1_OVF_vect
 */
ISR(TIMER1_COMPA_vect) {

  // If timeout occurs, clear frame state.
  resetFrameCallback();
  TCCR1B &= ~(1<<ICES1); // interrupt on falling edge
}
/**
 * @brief Timer 1 input capture interrupt
 * @param TIMER1_CAPT_vect
 */
ISR(TIMER1_CAPT_vect) {

  uint16_t pulse;  // high pulse duration

  if (edge == FALLING_EDGE) {
    // Get the Input Capture value
    pulse = ICR1>>1; // we count 0.5 us

    readDataCallback(pulse, 1); // decode low pulse
    TCCR1B |= (1<<ICES1); // interrupt on rising edge
    edge = RISING_EDGE;
  } else {
    // Get the Input Capture value
    pulse = ICR1>>1; // we count 0.5 us

    readDataCallback(pulse, 0); // decode high pulse
    TCCR1B &= ~(1<<ICES1); // interrupt on falling edge
    edge = FALLING_EDGE;
  }

  // zero out timer
  TCNT1 = 0;

}

