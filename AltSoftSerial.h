/* An Alternative Software Serial Library
 * http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 * Copyright (c) 2014 PJRC.COM, LLC, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef AltSoftSerial_h
#define AltSoftSerial_h

#ifdef USE_ALT_SOFT_SERIAL

#include <inttypes.h>

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#if defined(__arm__) && defined(CORE_TEENSY)
#define ALTSS_BASE_FREQ F_BUS
#else
#define ALTSS_BASE_FREQ F_CPU // 16000000UL ?
#endif

#define RX_BUFFER_SIZE 256 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#define TX_BUFFER_SIZE 128

void    AltSerialOpen(uint32_t baud); // ok
uint8_t AltSerialRead();              // ok
void    AltSerialWrite(uint8_t c);    // ok
uint8_t AltSerialAvailable();         // ok
void    AltSerialEnd();               // ok
uint8_t AltSerialPeek();              // ok
bool    AltSerialTXfree();            // ok
uint8_t AltSerialUsedTXBuff();        // ok
void    AltSerialSerialize(uint8_t a);// ok
void    AltUartSendData();

// void AltSerialWrite16(int16_t val);

/*
class AltSoftSerial : public Stream
{
public:
	AltSoftSerial() { }
	~AltSoftSerial() { end(); }
	static void begin(uint32_t baud) { init((ALTSS_BASE_FREQ + baud / 2) / baud); }
	static void end();
	int peek();
	int read();
	int available();
	int availableForWrite();
#if ARDUINO >= 100
	size_t write(uint8_t byte) { writeByte(byte); return 1; }
	void flush() { flushOutput(); }
#else
	void write(uint8_t byte) { writeByte(byte); }
	void flush() { flushInput(); }
#endif
	using Print::write;
	static void flushInput();
	static void flushOutput();
	// for drop-in compatibility with NewSoftSerial, rxPin & txPin ignored
	AltSoftSerial(uint8_t rxPin, uint8_t txPin, bool inverse = false) { }
	bool listen() { return false; }
	bool isListening() { return true; }
	bool overflow() { bool r = timing_error; timing_error = false; return r; }
	static int library_version() { return 1; }
	static void enable_timer0(bool enable) { }
	static bool timing_error;
private:
	static void init(uint32_t cycles_per_bit);
	static void writeByte(uint8_t byte);
};
*/

/////////////////////////////////////////////////
// Board Config
/////////////////////////////////////////////////

// Teensy 2.0
//
#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)

 //#define ALTSS_USE_TIMER1
 //#define INPUT_CAPTURE_PIN		22 // receive
 //#define OUTPUT_COMPARE_A_PIN		14 // transmit
 //#define OUTPUT_COMPARE_B_PIN		15 // unusable PWM
 //#define OUTPUT_COMPARE_C_PIN		 4 // unusable PWM

 #define ALTSS_USE_TIMER3
 #define INPUT_CAPTURE_PIN		10 // receive
 #define OUTPUT_COMPARE_A_PIN		 9 // transmit



// Teensy++ 2.0
//
#elif defined(__AVR_AT90USB1286__) && defined(CORE_TEENSY)

 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN		 4 // receive
 #define OUTPUT_COMPARE_A_PIN		25 // transmit
 #define OUTPUT_COMPARE_B_PIN		26 // unusable PWM
 #define OUTPUT_COMPARE_C_PIN		27 // unusable PWM

 //#define ALTSS_USE_TIMER3
 //#define INPUT_CAPTURE_PIN		17 // receive
 //#define OUTPUT_COMPARE_A_PIN		16 // transmit
 //#define OUTPUT_COMPARE_B_PIN		15 // unusable PWM
 //#define OUTPUT_COMPARE_C_PIN		14 // unusable PWM


// Teensy 3.x
//
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
 #define ALTSS_USE_FTM0
 #define INPUT_CAPTURE_PIN		20 // receive       (FTM0_CH5)
 #define OUTPUT_COMPARE_A_PIN		21 // transmit      (FTM0_CH6)
 #define OUTPUT_COMPARE_B_PIN		22 // unusable PWM  (FTM0_CH0)
 #define OUTPUT_COMPARE_C_PIN		23 // PWM usable fixed freq
 #define OUTPUT_COMPARE_D_PIN		 5 // PWM usable fixed freq
 #define OUTPUT_COMPARE_E_PIN		 6 // PWM usable fixed freq
 #define OUTPUT_COMPARE_F_PIN		 9 // PWM usable fixed freq
 #define OUTPUT_COMPARE_G_PIN		10 // PWM usable fixed freq


// Wiring-S
//
#elif defined(__AVR_ATmega644P__) && defined(WIRING)

 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN		 6 // receive
 #define OUTPUT_COMPARE_A_PIN		 5 // transmit
 #define OUTPUT_COMPARE_B_PIN		 4 // unusable PWM



// Arduino Uno, Duemilanove, LilyPad, etc
//
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN		 8 // receive
 #define OUTPUT_COMPARE_A_PIN		 9 // transmit
 #define OUTPUT_COMPARE_B_PIN		10 // unusable PWM


// Arduino Leonardo & Yun (from Cristian Maglie)
//
#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)

  //#define ALTSS_USE_TIMER1
  //#define INPUT_CAPTURE_PIN		4  // receive
  //#define OUTPUT_COMPARE_A_PIN	9 // transmit
  //#define OUTPUT_COMPARE_B_PIN	10 // unusable PWM
  //#define OUTPUT_COMPARE_C_PIN	11 // unusable PWM

  #define ALTSS_USE_TIMER3
  #define INPUT_CAPTURE_PIN		13 // receive
  #define OUTPUT_COMPARE_A_PIN		5 // transmit


// Arduino Mega
//
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

 //#define ALTSS_USE_TIMER4
 //#define INPUT_CAPTURE_PIN		49 // receive
 //#define OUTPUT_COMPARE_A_PIN		 6 // transmit
 //#define OUTPUT_COMPARE_B_PIN		 7 // unusable PWM
 //#define OUTPUT_COMPARE_C_PIN		 8 // unusable PWM

 #define ALTSS_USE_TIMER5
 #define INPUT_CAPTURE_PIN		48 // receive
 #define OUTPUT_COMPARE_A_PIN		46 // transmit
 #define OUTPUT_COMPARE_B_PIN		45 // unusable PWM
 #define OUTPUT_COMPARE_C_PIN		44 // unusable PWM



// EnviroDIY Mayfly, Sodaq Mbili
#elif defined ARDUINO_AVR_ENVIRODIY_MAYFLY || defined ARDUINO_AVR_SODAQ_MBILI
 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN		6 // receive
 #define OUTPUT_COMPARE_A_PIN	5 // transmit
 #define OUTPUT_COMPARE_B_PIN	4 // unusable PWM



// Sanguino, Mighty 1284
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN		14 // receive
 #define OUTPUT_COMPARE_A_PIN		13 // transmit
 #define OUTPUT_COMPARE_B_PIN		12 // unusable PWM



// Unknown board
#else
#error "Please define your board timer and pins"
#endif

/////////////////////////////////////////////////
// Timer Config
/////////////////////////////////////////////////


#if defined(ALTSS_USE_TIMER1)
  #define CONFIG_TIMER_NOPRESCALE()	(TIMSK1 = 0, TCCR1A = 0, TCCR1B = (1<<ICNC1) | (1<<CS10))
  #define CONFIG_TIMER_PRESCALE_8()	(TIMSK1 = 0, TCCR1A = 0, TCCR1B = (1<<ICNC1) | (1<<CS11))
  #define CONFIG_TIMER_PRESCALE_256()	(TIMSK1 = 0, TCCR1A = 0, TCCR1B = (1<<ICNC1) | (1<<CS12))
  #define CONFIG_MATCH_NORMAL()		(TCCR1A = TCCR1A & ~((1<<COM1A1) | (1<<COM1A0)))
  #define CONFIG_MATCH_TOGGLE()		(TCCR1A = (TCCR1A & ~(1<<COM1A1)) | (1<<COM1A0))
  #define CONFIG_MATCH_CLEAR()		(TCCR1A = (TCCR1A | (1<<COM1A1)) & ~(1<<COM1A0))
  #define CONFIG_MATCH_SET()		(TCCR1A = TCCR1A | ((1<<COM1A1) | (1<<COM1A0)))
  #define CONFIG_CAPTURE_FALLING_EDGE()	(TCCR1B &= ~(1<<ICES1))
  #define CONFIG_CAPTURE_RISING_EDGE()	(TCCR1B |= (1<<ICES1))
  #define ENABLE_INT_INPUT_CAPTURE()	(TIFR1 = (1<<ICF1), TIMSK1 = (1<<ICIE1))
  #define ENABLE_INT_COMPARE_A()	(TIFR1 = (1<<OCF1A), TIMSK1 |= (1<<OCIE1A))
  #define ENABLE_INT_COMPARE_B()	(TIFR1 = (1<<OCF1B), TIMSK1 |= (1<<OCIE1B))
  #define DISABLE_INT_INPUT_CAPTURE()	(TIMSK1 &= ~(1<<ICIE1))
  #define DISABLE_INT_COMPARE_A()	(TIMSK1 &= ~(1<<OCIE1A))
  #define DISABLE_INT_COMPARE_B()	(TIMSK1 &= ~(1<<OCIE1B))
  #define GET_TIMER_COUNT()		(TCNT1)
  #define GET_INPUT_CAPTURE()		(ICR1)
  #define GET_COMPARE_A()		(OCR1A)
  #define GET_COMPARE_B()		(OCR1B)
  #define SET_COMPARE_A(val)		(OCR1A = (val))
  #define SET_COMPARE_B(val)		(OCR1B = (val))
  #define CAPTURE_INTERRUPT		TIMER1_CAPT_vect
  #define COMPARE_A_INTERRUPT		TIMER1_COMPA_vect
  #define COMPARE_B_INTERRUPT		TIMER1_COMPB_vect


#elif defined(ALTSS_USE_TIMER3)
  #define CONFIG_TIMER_NOPRESCALE()	(TIMSK3 = 0, TCCR3A = 0, TCCR3B = (1<<ICNC3) | (1<<CS30))
  #define CONFIG_TIMER_PRESCALE_8()	(TIMSK3 = 0, TCCR3A = 0, TCCR3B = (1<<ICNC3) | (1<<CS31))
  #define CONFIG_TIMER_PRESCALE_256()	(TIMSK3 = 0, TCCR3A = 0, TCCR3B = (1<<ICNC3) | (1<<CS32))
  #define CONFIG_MATCH_NORMAL()		(TCCR3A = TCCR3A & ~((1<<COM3A1) | (1<<COM3A0)))
  #define CONFIG_MATCH_TOGGLE()		(TCCR3A = (TCCR3A & ~(1<<COM3A1)) | (1<<COM3A0))
  #define CONFIG_MATCH_CLEAR()		(TCCR3A = (TCCR3A | (1<<COM3A1)) & ~(1<<COM3A0))
  #define CONFIG_MATCH_SET()		(TCCR3A = TCCR3A | ((1<<COM3A1) | (1<<COM3A0)))
  #define CONFIG_CAPTURE_FALLING_EDGE()	(TCCR3B &= ~(1<<ICES3))
  #define CONFIG_CAPTURE_RISING_EDGE()	(TCCR3B |= (1<<ICES3))
  #define ENABLE_INT_INPUT_CAPTURE()	(TIFR3 = (1<<ICF3), TIMSK3 = (1<<ICIE3))
  #define ENABLE_INT_COMPARE_A()	(TIFR3 = (1<<OCF3A), TIMSK3 |= (1<<OCIE3A))
  #define ENABLE_INT_COMPARE_B()	(TIFR3 = (1<<OCF3B), TIMSK3 |= (1<<OCIE3B))
  #define DISABLE_INT_INPUT_CAPTURE()	(TIMSK3 &= ~(1<<ICIE3))
  #define DISABLE_INT_COMPARE_A()	(TIMSK3 &= ~(1<<OCIE3A))
  #define DISABLE_INT_COMPARE_B()	(TIMSK3 &= ~(1<<OCIE3B))
  #define GET_TIMER_COUNT()		(TCNT3)
  #define GET_INPUT_CAPTURE()		(ICR3)
  #define GET_COMPARE_A()		(OCR3A)
  #define GET_COMPARE_B()		(OCR3B)
  #define SET_COMPARE_A(val)		(OCR3A = (val))
  #define SET_COMPARE_B(val)		(OCR3B = (val))
  #define CAPTURE_INTERRUPT		TIMER3_CAPT_vect
  #define COMPARE_A_INTERRUPT		TIMER3_COMPA_vect
  #define COMPARE_B_INTERRUPT		TIMER3_COMPB_vect


#elif defined(ALTSS_USE_TIMER4)
  #define CONFIG_TIMER_NOPRESCALE()	(TIMSK4 = 0, TCCR4A = 0, TCCR4B = (1<<ICNC4) | (1<<CS40))
  #define CONFIG_TIMER_PRESCALE_8()	(TIMSK4 = 0, TCCR4A = 0, TCCR4B = (1<<ICNC4) | (1<<CS41))
  #define CONFIG_TIMER_PRESCALE_256()	(TIMSK4 = 0, TCCR4A = 0, TCCR4B = (1<<ICNC4) | (1<<CS42))
  #define CONFIG_MATCH_NORMAL()		(TCCR4A = TCCR4A & ~((1<<COM4A1) | (1<<COM4A0)))
  #define CONFIG_MATCH_TOGGLE()		(TCCR4A = (TCCR4A & ~(1<<COM4A1)) | (1<<COM4A0))
  #define CONFIG_MATCH_CLEAR()		(TCCR4A = (TCCR4A | (1<<COM4A1)) & ~(1<<COM4A0))
  #define CONFIG_MATCH_SET()		(TCCR4A = TCCR4A | ((1<<COM4A1) | (1<<COM4A0)))
  #define CONFIG_CAPTURE_FALLING_EDGE()	(TCCR4B &= ~(1<<ICES4))
  #define CONFIG_CAPTURE_RISING_EDGE()	(TCCR4B |= (1<<ICES4))
  #define ENABLE_INT_INPUT_CAPTURE()	(TIFR4 = (1<<ICF4), TIMSK4 = (1<<ICIE4))
  #define ENABLE_INT_COMPARE_A()	(TIFR4 = (1<<OCF4A), TIMSK4 |= (1<<OCIE4A))
  #define ENABLE_INT_COMPARE_B()	(TIFR4 = (1<<OCF4B), TIMSK4 |= (1<<OCIE4B))
  #define DISABLE_INT_INPUT_CAPTURE()	(TIMSK4 &= ~(1<<ICIE4))
  #define DISABLE_INT_COMPARE_A()	(TIMSK4 &= ~(1<<OCIE4A))
  #define DISABLE_INT_COMPARE_B()	(TIMSK4 &= ~(1<<OCIE4B))
  #define GET_TIMER_COUNT()		(TCNT4)
  #define GET_INPUT_CAPTURE()		(ICR4)
  #define GET_COMPARE_A()		(OCR4A)
  #define GET_COMPARE_B()		(OCR4B)
  #define SET_COMPARE_A(val)		(OCR4A = (val))
  #define SET_COMPARE_B(val)		(OCR4B = (val))
  #define CAPTURE_INTERRUPT		TIMER4_CAPT_vect
  #define COMPARE_A_INTERRUPT		TIMER4_COMPA_vect
  #define COMPARE_B_INTERRUPT		TIMER4_COMPB_vect


#elif defined(ALTSS_USE_TIMER5)
  #define CONFIG_TIMER_NOPRESCALE()	(TIMSK5 = 0, TCCR5A = 0, TCCR5B = (1<<ICNC5) | (1<<CS50))
  #define CONFIG_TIMER_PRESCALE_8()	(TIMSK5 = 0, TCCR5A = 0, TCCR5B = (1<<ICNC5) | (1<<CS51))
  #define CONFIG_TIMER_PRESCALE_256()	(TIMSK5 = 0, TCCR5A = 0, TCCR5B = (1<<ICNC5) | (1<<CS52))
  #define CONFIG_MATCH_NORMAL()		(TCCR5A = TCCR5A & ~((1<<COM5A1) | (1<<COM5A0)))
  #define CONFIG_MATCH_TOGGLE()		(TCCR5A = (TCCR5A & ~(1<<COM5A1)) | (1<<COM5A0))
  #define CONFIG_MATCH_CLEAR()		(TCCR5A = (TCCR5A | (1<<COM5A1)) & ~(1<<COM5A0))
  #define CONFIG_MATCH_SET()		(TCCR5A = TCCR5A | ((1<<COM5A1) | (1<<COM5A0)))
  #define CONFIG_CAPTURE_FALLING_EDGE()	(TCCR5B &= ~(1<<ICES5))
  #define CONFIG_CAPTURE_RISING_EDGE()	(TCCR5B |= (1<<ICES5))
  #define ENABLE_INT_INPUT_CAPTURE()	(TIFR5 = (1<<ICF5), TIMSK5 = (1<<ICIE5))
  #define ENABLE_INT_COMPARE_A()	(TIFR5 = (1<<OCF5A), TIMSK5 |= (1<<OCIE5A))
  #define ENABLE_INT_COMPARE_B()	(TIFR5 = (1<<OCF5B), TIMSK5 |= (1<<OCIE5B))
  #define DISABLE_INT_INPUT_CAPTURE()	(TIMSK5 &= ~(1<<ICIE5))
  #define DISABLE_INT_COMPARE_A()	(TIMSK5 &= ~(1<<OCIE5A))
  #define DISABLE_INT_COMPARE_B()	(TIMSK5 &= ~(1<<OCIE5B))
  #define GET_TIMER_COUNT()		(TCNT5)
  #define GET_INPUT_CAPTURE()		(ICR5)
  #define GET_COMPARE_A()		(OCR5A)
  #define GET_COMPARE_B()		(OCR5B)
  #define SET_COMPARE_A(val)		(OCR5A = (val))
  #define SET_COMPARE_B(val)		(OCR5B = (val))
  #define CAPTURE_INTERRUPT		TIMER5_CAPT_vect
  #define COMPARE_A_INTERRUPT		TIMER5_COMPA_vect
  #define COMPARE_B_INTERRUPT		TIMER5_COMPB_vect


#elif defined(ALTSS_USE_FTM0)
  // CH5 = input capture (input, pin 20)
  // CH6 = compare a     (output, pin 21)
  // CH0 = compare b     (input timeout)
  #define CONFIG_TIMER_NOPRESCALE()	FTM0_SC = 0; FTM0_CNT = 0; FTM0_MOD = 0xFFFF; \
					FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); \
					digitalWriteFast(21, HIGH); \
					NVIC_SET_PRIORITY(IRQ_FTM0, 48); \
					FTM0_C0SC = 0x18; \
					NVIC_ENABLE_IRQ(IRQ_FTM0);
  #define CONFIG_TIMER_PRESCALE_8()	FTM0_SC = 0; FTM0_CNT = 0; FTM0_MOD = 0xFFFF; \
					FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(3); \
					digitalWriteFast(21, HIGH); \
					NVIC_SET_PRIORITY(IRQ_FTM0, 48); \
					FTM0_C0SC = 0x18; \
					NVIC_ENABLE_IRQ(IRQ_FTM0);
  #define CONFIG_TIMER_PRESCALE_128()	FTM0_SC = 0; FTM0_CNT = 0; FTM0_MOD = 0xFFFF; \
					FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(7); \
					digitalWriteFast(21, HIGH); \
					NVIC_SET_PRIORITY(IRQ_FTM0, 48); \
					FTM0_C0SC = 0x18; \
					NVIC_ENABLE_IRQ(IRQ_FTM0);
  #define CONFIG_MATCH_NORMAL()		(FTM0_C6SC = 0)
  #define CONFIG_MATCH_TOGGLE()		(FTM0_C6SC = (FTM0_C6SC & 0xC3) | 0x14)
  #define CONFIG_MATCH_CLEAR()		(FTM0_C6SC = (FTM0_C6SC & 0xC3) | 0x18)
  #define CONFIG_MATCH_SET()		(FTM0_C6SC = (FTM0_C6SC & 0xC3) | 0x1C)
  #define CONFIG_CAPTURE_FALLING_EDGE()	(FTM0_C5SC = (FTM0_C5SC & 0xC3) | 0x08)
  #define CONFIG_CAPTURE_RISING_EDGE()	(FTM0_C5SC = (FTM0_C5SC & 0xC3) | 0x04)
  #define ENABLE_INT_INPUT_CAPTURE()	FTM0_C5SC = 0x48; \
					CORE_PIN20_CONFIG = PORT_PCR_MUX(4)|PORT_PCR_PE|PORT_PCR_PS
  #define ENABLE_INT_COMPARE_A()	FTM0_C6SC |= 0x40; \
					CORE_PIN21_CONFIG = PORT_PCR_MUX(4)|PORT_PCR_DSE|PORT_PCR_SRE
  #define ENABLE_INT_COMPARE_B()	(FTM0_C0SC = 0x58)
  #define DISABLE_INT_INPUT_CAPTURE()	FTM0_C5SC &= ~0x40; \
					CORE_PIN20_CONFIG = PORT_PCR_MUX(1)|PORT_PCR_PE|PORT_PCR_PS
  #define DISABLE_INT_COMPARE_A()	FTM0_C6SC &= ~0x40; \
					CORE_PIN21_CONFIG = PORT_PCR_MUX(1)|PORT_PCR_DSE|PORT_PCR_SRE; \
					digitalWriteFast(21, HIGH)
  #define DISABLE_INT_COMPARE_B()	(FTM0_C0SC &= ~0x40)
  #define GET_TIMER_COUNT()		(FTM0_CNT)
  #define GET_INPUT_CAPTURE()		(FTM0_C5V)
  #define GET_COMPARE_A()		(FTM0_C6V)
  #define GET_COMPARE_B()		(FTM0_C0V)
  #define SET_COMPARE_A(val)		(FTM0_C6V = val)
  #define SET_COMPARE_B(val)		if (FTM0_C0SC & FTM_CSC_CHF) FTM0_C0SC = 0x18; \
					do { FTM0_C0V = (val); } while (FTM0_C0V != (val));
  #define CAPTURE_INTERRUPT		altss_capture_interrupt
  #define COMPARE_A_INTERRUPT		altss_compare_a_interrupt
  #define COMPARE_B_INTERRUPT		altss_compare_b_interrupt
  #ifdef ISR
  #undef ISR
  #endif
  #define ISR(f) static void f (void)


#endif

/////////////////////////////////////////////////

#endif // USE_ALT_SOFT_SERIAL

#endif
