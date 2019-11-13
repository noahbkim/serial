#include "serial.h"
#include <util/atomic.h>
#include <avr/io.h>
#include <stdbool.h>

// https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/HardwareSerial.h
// https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/HardwareSerial.cpp

#define SERIAL_MODE_ASYNCHRONOUS_NORMAL_SCALE 16
#define SERIAL_MODE_ASYNCHRONOUS_DOUBLE_SCALE 8
#define SERIAL_MODE_SYNCHRONOUS_MASTER 2

#define TRANSMIT_ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

inline uint8_t is_interrupt_enabled() { return SREG & (1 << SREG_I); }
inline uint8_t is_interrupt_disabled() { return (SREG & (1 << SREG_I)) == 0; }

#define MANAGE_SERIAL_BIT(name, field, bit)\
inline void serial_##name##_enable(serial_t* serial) { *serial->field |= (1 << bit); }\
inline void serial_##name##_disable(serial_t* serial) { *serial->field &= ~(1 << bit); }\
inline uint8_t serial_is_##name##_enabled(serial_t* serial) { return *serial->field & (1 << bit); }

MANAGE_SERIAL_BIT(double_speed, ucsra, U2X0);
MANAGE_SERIAL_BIT(transmit, ucsrb, TXEN0)
MANAGE_SERIAL_BIT(receive, ucsrb, RXEN0)
MANAGE_SERIAL_BIT(interrupt_transmit, ucsrb, TXCIE0)
MANAGE_SERIAL_BIT(interrupt_receive, ucsrb, RXCIE0)
MANAGE_SERIAL_BIT(interrupt_empty, ucsrb, UDRIE0)

inline uint8_t serial_is_empty(serial_t* serial) { return *serial->ucsra & (1 << UDRE0); }
inline uint8_t serial_is_transmit_complete(serial_t* serial) { return *serial->ucsra & (1 << TXC0); }

inline void serial_set_ubrr(serial_t* serial, uint16_t ubrr)
{
    *serial->ubrrh = ubrr >> 8;
    *serial->ubrrl = ubrr;
}

void serial_construct(serial_t* serial, uint16_t bits_per_second, uint8_t config)
{
    // Compute transmission rate
    uint16_t ubrr = F_CPU / SERIAL_MODE_ASYNCHRONOUS_DOUBLE_SCALE / bits_per_second - 1;
    serial_double_speed_enable(serial);

    // If too low or in special case for Uno firmware, no double speed
    if (ubrr > 4095 || bits_per_second == 57600) {
        ubrr = F_CPU / SERIAL_MODE_ASYNCHRONOUS_NORMAL_SCALE / bits_per_second - 1;
        serial_double_speed_disable(serial);
    }

    // Set transmission rate
    serial_set_ubrr(serial, ubrr);

    // Set config
    *serial->ucsrb = config;

    // Set data, parity, and stop bits
    serial_transmit_enable(serial);
    serial_receive_enable(serial);
    // serial_interrupt_receive_enable(serial);
    serial_interrupt_empty_disable(serial);
}

void serial_destroy(serial_t* serial)
{
    // Wait for transmission
    serial_flush(serial);

    // Clear flags
    serial_transmit_disable(serial);
    serial_receive_disable(serial);
    // serial_interrupt_receive_disable(serial);
    serial_interrupt_empty_disable(serial);
}

inline void serial_write_internal(serial_t* serial, uint8_t data)
{
    *serial->udr = data;

    // Clear the transmission complete bit
    *serial->ucsra |= (1 << TXC0);
}

uint8_t serial_write(serial_t* serial, uint8_t data)
{
    serial->written = true;

    // If the register and buffer are empty, just send the byte immediately
    if (serial->transmit_buffer_head == serial->transmit_buffer_tail && serial_is_empty(serial))
    {
        TRANSMIT_ATOMIC
        {
            serial_write_internal(serial, data);
        }
        return 1;
    }

    // Otherwise, manipulate the buffer
    else
    {
        transmit_buffer_index_t next = (serial->transmit_buffer_head + 1) % SERIAL_TRANSMIT_BUFFER_SIZE;

        // Hang if the buffer is full, interrupt will handle
        while (next == serial->transmit_buffer_tail)
        {
            // If interrupts are disabled, poll interrupt manually
            if (is_interrupt_disabled() && serial_is_empty(serial))
            {
                serial_on_empty_interrupt(serial);
            }
        }

        serial->transmit_buffer[serial->transmit_buffer_head] = data; // Place in buffer

        // Increment head and re-enable interrupt, cannot be interrupted
        TRANSMIT_ATOMIC
        {
            serial->transmit_buffer_head = next;
            serial_interrupt_empty_enable(serial);
        }
        return 1;
    }
}

void serial_on_empty_interrupt(serial_t* serial)
{
    // Consume byte and advance buffer tail
    uint8_t data = serial->transmit_buffer[serial->transmit_buffer_tail];
    serial->transmit_buffer_tail = (serial->transmit_buffer_tail + 1) % SERIAL_TRANSMIT_BUFFER_SIZE;

    // Send data, no atomic since we're in interrupt
    serial_write_internal(serial, data);

    // Disable interrupts if tail reaches head
    if (serial->transmit_buffer_tail == serial->transmit_buffer_head)
    {
        serial_interrupt_empty_disable(serial);
    }
}

void serial_flush(serial_t* serial)
{
    // If not written, easy short-circuit
    if (!serial->written)
    {
        return;
    }

    // Otherwise wait for interrupt disable or done transmitting
    while (serial_is_interrupt_empty_enabled(serial) || !serial_is_transmit_complete(serial))
    {
        // If interrupts are disabled, but the empty interrupt is set, run interrupt code manually
        if (is_interrupt_disabled() && serial_is_interrupt_empty_enabled(serial) && serial_is_empty(serial))
        {
            serial_on_empty_interrupt(serial);
        }
    }
}
