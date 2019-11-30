#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "../src/serial.h"

SERIAL_SETUP(0);

int serial_write_string(serial_t* serial, char* string)
{
    while (*string)
    {
        serial_write(serial, *string++);
    }
}

void buffer_setup()
{
    DDRD |= (1 << PD3);
    PORTD &= ~(1 << PD3);
}

int main()
{
    sei();
    buffer_setup();
    serial_construct(S0, 9600, SERIAL_8N1, false);

    _delay_ms(500);  // So we have time to connect
    serial_write_string(S0, "abcde\n");

    // Echo
    for (uint8_t i = 0; i < 6; i++)
    {
        while (!serial_read_available(S0));
        uint8_t byte = serial_read(S0);
        serial_write(S0, byte);
    }

    serial_destroy(S0);
    return 0;
}
