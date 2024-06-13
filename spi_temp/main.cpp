#include <iostream>
#include <unistd.h>
#include <pigpio.h>

// SPI channel and speed
#define SPI_CHANNEL 0
#define SPI_SPEED 1000000  // 500 kHz

// Chip select (CS) pin
#define CS_PIN 8  // Change this to your desired GPIO pin for CS

void setup_cs_pin(int cs_pin) {
    gpioSetMode(cs_pin, PI_OUTPUT);  // Set CS pin as output
    gpioWrite(cs_pin, PI_HIGH);      // Set CS pin high (inactive)
}

void select_cs_pin(int cs_pin) {
    gpioWrite(cs_pin, PI_LOW);  // Set CS pin low (active)
}

void deselect_cs_pin(int cs_pin) {
    gpioWrite(cs_pin, PI_HIGH);  // Set CS pin high (inactive)
}

int main() {
    // Initialize the pigpio library
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed\n" << std::endl;
        return 1;
    }

    // Set up the CS pin
    setup_cs_pin(CS_PIN);

    // Open the SPI channel
    int spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);  // mode 0
    if (spi_handle < 0) {
        std::cerr << "spiOpen failed" << std::endl;
        gpioTerminate();
        return 1;
    }

    // Data to send (4 bytes)
    char data[4] = {0x01, 0x00, 0x03, 0x01};
    int i = 0;
    while(i < 2)
    {
        // Select the CS pin
        select_cs_pin(CS_PIN);

        // Send data over SPI
        int result = spiWrite(spi_handle, data, 4);
        if (result < 0) {
            std::cerr << "spiWrite failed" <<std::endl;
        }

        // Deselect the CS pin
        deselect_cs_pin(CS_PIN);
        i++;
        sleep(1);
    }

    // Close the SPI channel
    spiClose(spi_handle);

    // Terminate the pigpio library
    gpioTerminate();

    return 0;
}