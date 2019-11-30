#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#define SERIAL_TRANSMIT_BUFFER_SIZE 64
#define SERIAL_RECEIVE_BUFFER_SIZE 64

#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06  // Default
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

typedef uint8_t transmit_buffer_index_t;
typedef uint8_t receive_buffer_index_t;

typedef struct serial_t
{
    // Pointers to relevant registers
    volatile uint8_t* const ubrrh;
    volatile uint8_t* const ubrrl;
    volatile uint8_t* const ucsra;
    volatile uint8_t* const ucsrb;
    volatile uint8_t* const ucsrc;
    volatile uint8_t* const udr;

    // Buffer contents
    bool written;

    // Buffer indices
    volatile transmit_buffer_index_t transmit_buffer_head;
    volatile transmit_buffer_index_t transmit_buffer_tail;
    volatile receive_buffer_index_t receive_buffer_head;
    volatile receive_buffer_index_t receive_buffer_tail;

    // Buffers
    uint8_t transmit_buffer[SERIAL_TRANSMIT_BUFFER_SIZE];
    uint8_t receive_buffer[SERIAL_RECEIVE_BUFFER_SIZE];

} serial_t;

void serial_construct(serial_t* serial, uint16_t bits_per_second, uint8_t config, bool double_scale);
void serial_destroy(serial_t* serial);
void serial_on_empty_interrupt(serial_t* serial);
void serial_on_receive_interrupt(serial_t* serial);

uint8_t serial_write(serial_t* serial, uint8_t data);
void serial_flush(serial_t* serial);

bool serial_peek(serial_t* serial);
uint8_t serial_read(serial_t* serial);

bool serial_read_available(serial_t* serial);

// Macro for setting up a serial channel, creates serial_t* S<channel>
#define SERIAL_SETUP(channel)\
serial_t SERIAL##channel = { &UBRR##channel##H, &UBRR##channel##L, &UCSR##channel##A, &UCSR##channel##B, &UCSR##channel##C, &UDR##channel };\
serial_t* const S##channel = &SERIAL##channel;\
ISR(USART_UDRE_vect) { serial_on_empty_interrupt(S##channel); }\
ISR(USART_RX_vect) { serial_on_receive_interrupt(S##channel); }\

#endif  // SERIAL_H
