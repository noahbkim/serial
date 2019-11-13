# Serial

This library aims to provide convenient serial communication on the Arduino Uno.
Here's an example of how it works:

```c
#include "serial.h"

SERIAL_SETUP(0)  // Setup 0 channel, USB serial

int main()
{
    serial_constructor(S0, 9600, SERIAL_8N1);  // Setup serial speed and format
    serial_write(S0, 'h');
    serial_write(S0, 'i');
    serial_write(S0, '!');
    serial_destroy(S0)

    return 0;
}
```

More functionality is coming soon!
