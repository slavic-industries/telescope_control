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
#define TMC429_CS_PIN 5

const int RUN_CURRENT_PERCENT = 60;
const int MICROSTEPS_PER_STEP = 256;

// Stepper controller settings
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 20;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const int ACCELERATION_MAX = MICROSTEPS_PER_REV / 1;
const long VELOCITY_MAX = REVS_PER_SEC_MAX * MICROSTEPS_PER_REV;
const long VELOCITY_MIN = 10;
const long VELOCITY_INC = 1000;
long target_velocity, actual_velocity, delta_velocity;
bool at_target_velocity;


void print_driver_settings(TMC2130::Settings settings)
{
  std::cout << "stealth_chop_enabled: " << std::bitset<8>(settings.stealth_chop_enabled) << std::endl;
  std::cout << "automatic_current_scaling_enabled: " << std::bitset<8>(settings.automatic_current_scaling_enabled) << std::endl;
  std::cout << "zero_hold_current_mode: " << std::bitset<8>(settings.zero_hold_current_mode) << std::endl;
  std::cout << "pwm_offset: " << std::bitset<8>(settings.pwm_offset) << std::endl;
  std::cout << "pwm_gradient: " << std::bitset<8>(settings.pwm_gradient) << std::endl;
  std::cout << "irun: " << std::bitset<8>(settings.irun) << std::endl;
  std::cout << "ihold: " << std::bitset<8>(settings.ihold) << std::endl;
  std::cout << "iholddelay: " << std::bitset<8>(settings.iholddelay) << std::endl;

}

 


int main() {

  target_velocity = 2000;

  SI5351 clock_generator = SI5351(HOST_I2C_DEVICE, SI5351_ADDR);

  TMC2130 stepper_driver;
  TMC429 stepper_controller;

  if (clock_generator.begin())
  {
    std::cout << "No SI5351 detected. Check your wiring or I2C ADDR!" << std::endl;
    return -1;
  }

  clock_generator.enableOutputs(false);
  clock_generator.setupPLLInt(SI5351_PLL_A, 15);
  clock_generator.setupMultisynth(2, SI5351_PLL_A, 23, 1, 2);
  clock_generator.enableOutputs(true);
  std::cout << "SI5351 setup and configuration complete!" << std::endl;

  

  std::cout << "TMC429 Setup " << std::endl;
  if(stepper_controller.setup(TMC429_CS_PIN, SPI_CHANNEL))
  {
    std::cerr << "Stepper driver initialization failed." << std::endl;
    return -1;
  }
  if(stepper_controller.communicating()) std::cout << "TMC429 setup and configuration complete!" << std::endl;



  stepper_controller.disableLeftSwitchStop(MOTOR_INDEX);
  stepper_controller.disableRightSwitches();
  // stepper_controller.setVelocityMode(MOTOR_INDEX);
  stepper_controller.setLimitsInHz(MOTOR_INDEX, VELOCITY_MIN, VELOCITY_MAX, ACCELERATION_MAX);
  stepper_controller.stopAll();
  usleep(2000000);


  std::cout << "TMC2130 Setup " << std::endl;
  if(stepper_driver.setup(TMC2130_CS_PIN, SPI_CHANNEL))
  {
    std::cerr << "Stepper driver initialization failed." << std::endl;
    return -1;
  }
  if(stepper_driver.communicating()) std::cout << "TMC2130 setup and configuration complete!" << std::endl;

  stepper_driver.setMicrostepsPerStep(MICROSTEPS_PER_STEP);
  stepper_driver.enableStealthChop();
  // stepper_driver.setAllCurrentValues(100, 0, 0);
  stepper_driver.initialize();

  TMC2130::Settings driver_settings = stepper_driver.getSettings();

  print_driver_settings(driver_settings);

  stepper_controller.setTargetVelocity(MOTOR_INDEX, 0);
  stepper_controller.setVelocityMode(MOTOR_INDEX);
  stepper_controller.setTargetVelocity(MOTOR_INDEX, 5000);
  std::cout << "Motor target: " << std::bitset<32>(stepper_controller.getTargetVelocity(MOTOR_INDEX)) << std::endl;


  bool at_target = false;
  while(!at_target)
  { //
    std::cout << "Motor at: " << std::bitset<32>(stepper_controller.getActualVelocity(MOTOR_INDEX)) << std::endl;
    at_target = stepper_controller.atTargetVelocity(MOTOR_INDEX);
    usleep(100);
  }
  std::cout << "Motor at target." << std::endl;

  usleep(10000000);
  stepper_controller.setTargetVelocity(MOTOR_INDEX, 0);
  while(!stepper_controller.atTargetVelocity(MOTOR_INDEX));
  std::cout << "Motor stopped." << std::endl;

  return 0;
  
  delta_velocity = VELOCITY_INC;
  stepper_controller.setTargetVelocityInHz(MOTOR_INDEX, 0);
  usleep(500000);
  stepper_controller.setTargetVelocityInHz(MOTOR_INDEX, target_velocity);

  stepper_driver.enable();

  std::cout << "Setup Complete!" <<  std::endl;

  // int pwm_pin = 6;
  // int frequency = 1000; //Hz
  // int dutyCycle = 120;

  // if (gpioInitialise() < 0) 
  // {
  //   std::cerr << "pigpio initialization failed." << std::endl;
  //   return 1;
  // }

  // gpioSetPWMfrequency(pwm_pin, frequency);
  // gpioPWM(pwm_pin, dutyCycle);


  while(1)
  {

  }

  return 0;
}
