#include <avr/io.h>
#include <util/delay.h>
#include <util/crc16.h>

#define TX_PORT  PORTB
#define TX_PIN   PB1
#define TX_DELAY _delay_ms(2)

typedef struct {
    uint8_t  status;
    uint16_t temp;
    uint16_t humidity;
} data_t;

uint8_t tx_buffer[50];
uint8_t tx_index;

void tx_encode(uint8_t* data, uint8_t data_length) {
    uint8_t  packet_length = data_length + 3;
    uint16_t crc           = 0xffff;
    uint8_t  i;

    tx_index = 0;

    // Preamble
    tx_buffer[tx_index++] = 0;
    tx_buffer[tx_index++] = 0;

    // Sync vector (we can look for 0xA5 when manchester encoded)
    tx_buffer[tx_index++] = 3;

    // Packet length
    crc = _crc_ccitt_update(crc, packet_length);
    tx_buffer[tx_index++] = packet_length;

    // Data bytes
    for(i = 0; i < data_length; i++) {
        crc = _crc_ccitt_update(crc, data[i]);
        tx_buffer[tx_index++] = data[i];
    }

    // Two byte CRC
    tx_buffer[tx_index++] = crc & 0xff;
    tx_buffer[tx_index++] = crc >> 8;
}

void tx_manchester_byte(uint8_t byte) {
    int8_t bit = 7;
    for (; bit >= 0; --bit)
    {
        if (byte & (1 << bit))
        {
            // high bit, transmit low then high
            TX_PORT &= ~(1 << TX_PIN);
            TX_DELAY;
            TX_PORT |= (1 << TX_PIN);
            TX_DELAY;
        } else {
            // high bit, transmit high then low
            TX_PORT |= (1 << TX_PIN);
            TX_DELAY;
            TX_PORT &= ~(1 << TX_PIN);
            TX_DELAY;
        }

    }
}

void tx_send() {
    uint8_t i;
    for(i = 0; i < tx_index; i++) {
        // transmit each byte
        tx_manchester_byte(tx_buffer[i]);
    }

    // disable TX
    TX_PORT &= ~(1 << TX_PIN);
}

int main(void)
{
    uint8_t i;
    data_t data;
    data.status   = 0;
    data.temp     = 223;
    data.humidity = 675;

    while(1) {
        // encode and send data
        tx_encode((uint8_t*) &data, sizeof(data));
        tx_send();

        // sleep for 3 seconds
        for(i = 0; i < 12; i++) {
            _delay_ms(250);
        }
    }

    return 0;
}
