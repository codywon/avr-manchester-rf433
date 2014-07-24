#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/crc16.h>
#include "uart.h"

#define RX_INPUT_PIN PINB
#define RX_PIN       PINB0

typedef struct {
    uint8_t  status;
    uint16_t temp;
    uint16_t humidity;
} data_t;

uint8_t  rx_buffer[512];
uint16_t rx_buffer_index = 0;
uint8_t  packet_counter  = 0;
uint8_t  rx_sample;
uint8_t  rx_pulse_width;
uint8_t  rx_last_sample;
char     msg_buf[10];

int main(void)
{
    uart_init();
    uart_newline();
    uart_sendstr_p(PSTR("===== Manchester RX ====="));
    uart_newline();

    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS11);   // prescaler 8
    TIMSK  |= (1 << OCIE1A); // interupt on compare match
    OCR1A  = 66;             // set compare value (8097 times pr es)
    sei();                   // enable global interrupts

    // everything happens in interrupts
    while(1) {}

    return 0;
}

void decode_data(uint8_t* buffer, uint8_t length) {
   //uint16_t data[3];

   data_t data;

   if (length - 1 == sizeof(data)) {
       buffer++; // skip the first byte, as it's the packet length

       // copy from data buffer into the struct
       memcpy(&data, buffer, sizeof(data));

       // print out decoded data
       uart_newline();
       uart_sendstr_p(PSTR("Decoded data:"));
       uart_newline();

       uart_sendstr_p(PSTR("Status: "));
       sprintf((char*) &msg_buf, "%d", data.status);
       uart_sendstr((char*) &msg_buf);
       uart_newline();

       uart_sendstr_p(PSTR("Temp: "));
       sprintf((char*) &msg_buf, "%d", data.temp);
       uart_sendstr((char*) &msg_buf);
       uart_newline();

       uart_sendstr_p(PSTR("Humidity: "));
       sprintf((char*) &msg_buf, "%d", data.humidity);
       uart_sendstr((char*) &msg_buf);
       uart_newline();

       //uart_sendstr_p(PSTR("Status: "));
       //sprintf((char*) &msg_buf, "%d", data[0]);
       //uart_sendstr((char*) &msg_buf);
       //uart_newline();

       //uart_sendstr_p(PSTR("Temp: "));
       //sprintf((char*) &msg_buf, "%d", data[1]);
       //uart_sendstr((char*) &msg_buf);
       //uart_newline();

       //uart_sendstr_p(PSTR("Humidity: "));
       //sprintf((char*) &msg_buf, "%d", data[2]);
       //uart_sendstr((char*) &msg_buf);
       //uart_newline();

       uart_newline();
   } else {
       // We didn't receive enough bytes to decode a whole packet
       uart_sendstr_p(PSTR("Expected to receive "));
       sprintf((char*) &msg_buf, "%d", sizeof(data));
       uart_sendstr((char*) &msg_buf);
       uart_sendstr_p(PSTR(" bytes, but received "));
       sprintf((char*) &msg_buf, "%d", length);
       uart_sendstr((char*) &msg_buf);
       uart_sendstr_p(PSTR(" bytes."));
       uart_newline();
   }
}

void decode_manchester() {
    uint16_t i;
    uint16_t data_start = 0xff;
    uint8_t  sample = 0;
    uint8_t  data_buffer[32];
    uint8_t  data_index = 0;

    // disable interrupts so we aren't... interrupted as we do a lot of
    // processing here
    cli();

    // print out some info about the packet
    uart_newline();
    uart_sendstr_p(PSTR("= Packet "));
    sprintf((char*) &msg_buf, "%d =", packet_counter++);
    uart_sendstr((char*) &msg_buf);
    uart_newline();

    uart_sendstr_p(PSTR("Raw bits ("));
    sprintf((char*) &msg_buf, "%d):", rx_buffer_index);
    uart_sendstr((char*) &msg_buf);
    uart_newline();

    // print out the raw bits we have received
    for(i = 0; i <= rx_buffer_index; i++) {
        if (rx_buffer[i]) {
            uart_sendchar('1');
        } else {
            uart_sendchar('0');
        }

        // add each bit to a sample looking for the start vector (0xA5)
        sample >>= 1;
        if (!rx_buffer[i]) {
            sample |= 0x80;
        }

        if (sample == 0xA5 && data_start == 0xFF) {
            // we found where the data starts
            data_start = i + 1;
        }
    }
    uart_newline();

    // add an indicator for debugging
    for(i = 0; i < data_start; i++) {
        uart_sendchar(' ');
    }
    uart_sendchar('^');
    uart_newline();

    // This is where we actually decode the manchester encoded data, we loop
    // over the rx_buffer from data_start until the end. We also print out
    // a listing of each manchester encoded bit-pair for debugging. We don't
    // actually look for a stop vector, in most cases there will be a few
    // excess bits but not enough to decode a whole byte.
    uint8_t bit = 7;
    for(i = data_start; i <= rx_buffer_index; i += 2) {
        // Print debug info
        sprintf((char*) &msg_buf, "%d%d,", rx_buffer[i] ? 1 : 0, rx_buffer[i + 1] ? 1 : 0);
        uart_sendstr((char*) &msg_buf);

        if (rx_buffer[i] && !rx_buffer[i + 1]) {
            // It is a manchester encoded high bit
            sample |= (1 << bit);
        } else {
            // It is a manchester encoded low bit (it could be something else
            // too, but we just ignore that and hope for the best)
            sample &= ~(1 << bit);
        }

        // Count until we decode a whole byte
        if (bit-- == 0) {
            bit = 7;

            sprintf((char*) &msg_buf, " - %d\t%x\t%c", sample, sample, sample);
            uart_sendstr((char*) &msg_buf);
            uart_newline();

            // Add it to the data buffer
            data_buffer[data_index++] = sample;
        }
    }
    uart_newline();

    // Next we decode the packet, first we verify the CRC
    uint16_t crc = 0xffff;
    for (i = 0; i < data_index - 2; i++)
    {
        crc = _crc_ccitt_update(crc, data_buffer[i]);
    }

    uint8_t crc_low  = crc & 0xff;
    uint8_t crc_high = crc >> 8;

    if (crc_low == data_buffer[data_index - 2] && crc_high == data_buffer[data_index - 1]) {
        uart_sendstr_p(PSTR("CRC verified"));
    } else {
        uart_sendstr_p(PSTR("CRC mismatch, expected "));
        sprintf((char*) &msg_buf, "%x %x", crc_low, crc_high);
        uart_sendstr((char*) &msg_buf);
        uart_sendstr_p(PSTR(" , got "));
        sprintf((char*) &msg_buf, "%x %x", data_buffer[data_index - 2], data_buffer[data_index - 1]);
        uart_sendstr((char*) &msg_buf);
    }
    uart_newline();

    // Then decode the actual data
    uint8_t* data_ptr = data_buffer;
    decode_data(data_ptr, data_index - 2);

    // re-enable interrupts
    sei();
}

void sample_bit() {
    // increment counter
    rx_pulse_width++;

    // check receive pin
    rx_sample = (RX_INPUT_PIN & (1 << RX_PIN));

    if (rx_sample != rx_last_sample) {
        // pin has changed
        rx_last_sample = rx_sample;

        if (rx_pulse_width < 5) {
            // the bit changed, but it was very short, so just ignore it and
            // assume it was an error
            rx_last_sample = !rx_last_sample;
        } else if (rx_pulse_width > 13 && rx_pulse_width < 18) {
            // we received a single bit
            rx_buffer[rx_buffer_index++] = rx_sample;
        } else if (rx_pulse_width > 28 && rx_pulse_width < 36) {
            // we received two bits the same
            rx_buffer[rx_buffer_index++] = rx_sample;
            rx_buffer[rx_buffer_index++] = rx_sample;
        } else {
            // we received a long pulse, assume that we have received all the
            // data and attempt to process it if the rx_buffer has received
            // a suitable amount of data
            if (rx_buffer_index > 30) {
                decode_manchester();
            }

            // reset and start again
            rx_buffer_index = 0;
        }

        // reset width for new bit
        rx_pulse_width = 0;
    }
}

ISR(TIMER1_COMPA_vect) {
    sample_bit();
}
