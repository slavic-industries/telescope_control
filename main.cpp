#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <bitset>
#include <cstdint>
#include <unistd.h>
#include <cmath>
#include "si5351.h"
#include "tmc2130.h"
#include "tmc429.h"

#define SPI_CHANNEL 1
#define SPI_SPEED 1000000  // 1 MHz

#define HOST_I2C_DEVICE 1
#define SI5351_ADDR 0x60

#define TMC2130_CS_PIN 8
#define TMC429_CS_PIN 7




int main() {


  SI5351 clock_generator = SI5351(HOST_I2C_DEVICE, SI5351_ADDR);

  TMC2130 stepper_driver;

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


  if(stepper_driver.setup(TMC2130_CS_PIN, SPI_CHANNEL))
  {
    std::cerr << "Stepper driver initialization failed." << std::endl;
    return -1;
  }
  if(stepper_driver.communicating()) std::cout << "TMC2130 setup and configuration complete!" << std::endl;



  // while(1)
  // {

  // }

  return 0;
}
