/*
 * oscilloscope.c
 *
 * Created: 12/12/2018 12:38:10 PM
 *  Author: WRL
 */ 

#define F_CPU 8000000UL		// 8 MHz
#define BAUD_RATE 9600

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "oscilloscope.h"
#include "uart.h"

#define _CMD_START_CNT 1
#define _CMD_END_CNT   1
#define _CMD_SPACING   1
#define _CMD_PINS_ST   1
#define _CMD_TIME_SNAP 4

#define FULL_SAMPLE_CNT (_CMD_START_CNT + _CMD_PINS_ST +  _CMD_TIME_SNAP + _CMD_END_CNT)

#define _SAMPLE_PIN  (_CMD_START_CNT)
#define _SAMPLE_TIME (_CMD_START_CNT + _CMD_PINS_ST)

#define MARKER_END   (FULL_SAMPLE_CNT - 1)
#define MARKER_START (0)

//#define TIMER_PERIOD_MS ((int)65)

// Send the following frame for each sample:
// @PIN TIME3 TIME2 TIME1 TIME0;

#define _SAMPLES_NUM 100

#define LOGIC_PORT PINB

typedef enum {SAMPLING, SENDING, IDLE} states_t;

static states_t currentState = SAMPLING;
static uint8_t  analog_samples[_SAMPLES_NUM];
static uint32_t time_snap[_SAMPLES_NUM];

volatile uint32_t tot_overflow;

static volatile uint8_t samples_cnt = 0;
static uint8_t _go_signal_buf = (uint8_t)'N';

volatile uint32_t dummy_timer = 0;

void uart_config(void)
{
	UART_cfg my_uart_cfg;

	my_uart_cfg.UBRRL_cfg = (BAUD_RATE_VALUE)&0x00FF;
	my_uart_cfg.UBRRH_cfg = (((BAUD_RATE_VALUE)&0xFF00)>>8);

	my_uart_cfg.UCSRA_cfg = 0;
	my_uart_cfg.UCSRB_cfg = (1<<RXEN)  | (1<<TXEN) | (1<<TXCIE) | (1<<RXCIE);
	my_uart_cfg.UCSRC_cfg = (1<<URSEL) | (3<<UCSZ0);

	UART_Init(&my_uart_cfg);
}

void TIMER1_init()
{
	// set up timer with prescaler = 2
	TCCR1B |= (1 << CS10);

	// initialize counter
	TCNT1 = 0;

	// enable overflow interrupt
	TIMSK |= (1 << TOIE1);

	// enable global interrupts
	sei();

	// initialize overflow counter variable
	tot_overflow = 0;

}

void ADC_init()
{
	ADMUX = 0b01100000;    // PA0 -> ADC0, ADLAR=1 (8-bit)
	ADCSRA |= ((1<<ADEN) | (1<<ADSC) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)); // ADC prescaler at 4

}

// TIMER1 overflow interrupt service routine
// called whenever TCNT1 overflows
ISR(TIMER1_OVF_vect)
{
	tot_overflow++;
}

static uint32_t getTime(void)
{
	return tot_overflow /* TIMER_PERIOD_MS */;
}

uint8_t getADCSample(void)
{
	uint8_t ADCvalue = 0;
	ADCSRA |= (1 << ADSC);
	while(!(ADCSRA & (1<<ADIF))); // waiting for ADIF, conversion complete
	ADCvalue = ADCH;
	return ADCvalue;
}

void OSCI_Init(void)
{
	/* Init UART driver. */
	uart_config();
	
	// Init TIMER1
	TIMER1_init();

	// Init ADC.
	ADC_init();

	/* Start with analog sampling. */
	currentState = SAMPLING;
}

void OSCI_MainFunction(void)
{
	//static volatile uint8_t samples_cnt = 0;
	//static uint8_t _go_signal_buf = (uint8_t)'N';
	
	// Main function must have two states,
	// First state is command parsing and waveform selection.
	// second state is waveform executing.
	switch(currentState)
	{
		case SAMPLING:
		{
			// DO here sampling.
			analog_samples[samples_cnt] = getADCSample();
			time_snap[samples_cnt]      = getTime();
			
			// Increment sample count.
			samples_cnt++;

			_delay_ms(10);
			// Start sending the collected _SAMPLES_NUM samples.
			currentState = (samples_cnt >= _SAMPLES_NUM) ? SENDING : SAMPLING;
			break;
		}
		case SENDING:
		{
			// For _SAMPLES_NUM samples send the construct the buffer.
			static uint8_t _sample_buf[FULL_SAMPLE_CNT];
			for(uint8_t i = 0; i < _SAMPLES_NUM; ++i)
			{
				// Construct the buffer.
				
				// Add buffer marker
				_sample_buf[MARKER_START] = '@';

				// Add pin value.
				_sample_buf[_SAMPLE_PIN]  = analog_samples[i];

//				_sample_buf[_SAMPLE_PIN]  = ((uint8_t) dummy_timer & 0x000000FF);

				// Add time snap value.
				_sample_buf[_SAMPLE_TIME + 0] = ((time_snap[i] & 0xFF000000) >> 24);
				_sample_buf[_SAMPLE_TIME + 1] = ((time_snap[i] & 0x00FF0000) >> 16);
				_sample_buf[_SAMPLE_TIME + 2] = ((time_snap[i] & 0x0000FF00) >> 8);
				_sample_buf[_SAMPLE_TIME + 3] = ((time_snap[i] & 0x000000FF) >> 0);

				_sample_buf[MARKER_END]   = ';';

				// Send sample.
				UART_SendPayload(_sample_buf, FULL_SAMPLE_CNT);
				while (0 == UART_IsTxComplete());
			}

			// Trigger receiving for go signal.
			UART_ReceivePayload(&_go_signal_buf, 1);
		}
		case IDLE:
		{
			currentState = ((1 == UART_IsRxComplete())&&(_go_signal_buf == (uint8_t)'G')) ? SAMPLING : IDLE;

			if(currentState == SAMPLING)
			{
				_go_signal_buf = (uint8_t)'N';
				// TODO: Place your code here to reset the timer value.
				tot_overflow = 0;
				samples_cnt = 0;
			}

			//UART_SendPayload((uint8_t *)"\r", 1);
			break;
		}
		default: {/* Do nothing.*/}
	}
}
