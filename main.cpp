#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <bitset>
#include <cstdint>
#include <unistd.h>
#include <cmath>
#include "si5351.h"

#define SPI_CHANNEL 0
#define SPI_SPEED 1000000  // 1 MHz

#define HOST_I2C_DEVICE 1
#define SI5351_ADDR 0x60



int main() {


    SI5351 clock_generator = SI5351(HOST_I2C_DEVICE, SI5351_ADDR);

    if (clock_generator.begin())
    {
      std::cout << "No SI5351 detected. Check your wiring or I2C ADDR!" << std::endl;
      return -1;
    }

    clock_generator.setupPLL(SI5351_PLL_B, 24, 2, 3);
    clock_generator.setupMultisynth(2, SI5351_PLL_B, 900, 0, 1);
    clock_generator.setupRdiv(2, SI5351_R_DIV_32);
    clock_generator.enableOutputs(true);
    std::cout << "SI5351 setup and configuration complete!" << std::endl;

    // while(1)
    // {

    // }

    return 0;
}
