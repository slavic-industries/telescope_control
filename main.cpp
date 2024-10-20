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
const int MICROSTEPS_PER_STEP = 16;

// Stepper controller settings
const int CHIP_SELECT_PIN = 10;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 2;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const int ACCELERATION_MAX = 10; // MICROSTEPS_PER_REV / 1;
const long VELOCITY_MAX = 0; //REVS_PER_SEC_MAX * MICROSTEPS_PER_REV;
const long VELOCITY_MIN = 0;
const long VELOCITY_INC = 5000;
long target_velocity, actual_velocity, delta_velocity;
bool at_target_velocity;

const int SETUP_DELAY = 4000;
const int LOOP_DELAY = 500;


void print_driver_settings(TMC2130::Settings settings)
{
  std::cout << "stealth_chop_enabled: " << std::bitset<1>(settings.stealth_chop_enabled) << std::endl;
  std::cout << "automatic_current_scaling_enabled: " << std::bitset<1>(settings.automatic_current_scaling_enabled) << std::endl;
  std::cout << "zero_hold_current_mode: " << std::bitset<8>(settings.zero_hold_current_mode) << std::endl;
  std::cout << "pwm_offset: " << std::bitset<8>(settings.pwm_offset) << std::endl;
  std::cout << "pwm_gradient: " << std::bitset<8>(settings.pwm_gradient) << std::endl;
  std::cout << "irun: " << std::bitset<8>(settings.irun) << std::endl;
  std::cout << "ihold: " << std::bitset<8>(settings.ihold) << std::endl;
  std::cout << "iholddelay: " << std::bitset<8>(settings.iholddelay) << std::endl;

}



 


int main() {


  SI5351 clock_generator = SI5351(HOST_I2C_DEVICE, SI5351_ADDR);

  TMC2130 tmc2130;
  TMC429 tmc429;

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

 // if(tmc429.setup(TMC429_CS_PIN, SPI_CHANNEL) != 0)
 // {
 //    std::cerr << "Steper controller initialization failed." << std::endl;
 //    return -1;
 // }
 // std::cout << "TMC429 communicating: " << tmc429.communicating() << tmc429.communicating() << std::endl;
 // std::cout << "TMC429 version: " << tmc429.getVersion() << std::endl;
 // if(!tmc429.communicating()){
 //   std::cerr << "TMC429 controller not communicating." << std::endl;
 //   return -1;
 // }

  std::cout << "***********"<< std::endl;
  std::cout << "TMC2130 Setup " << std::endl;

  if(tmc2130.setup(TMC2130_CS_PIN, SPI_CHANNEL))
  {
    std::cerr << "Stepper driver initialization failed." << std::endl;
    return -1;
  }
  // usleep(100000) 
  if(!tmc2130.communicating())
  {
    std::cout << "TMC2130 not communicating." << std::endl;
    return -1;
  }
  // usleep(100000);

  tmc2130.enableStealthChop();
  // tmc2130.enableAutomaticCurrentScaling();
  usleep(100000);
  tmc2130.setHoldCurrent(10);
  tmc2130.setRunCurrent(10);

  std::cout << "TMC2130 Setup Complete" << std::endl;

  std::cout << "***********"<< std::endl;
  std::cout << "TMC429 Setup " << std::endl;
  if(tmc429.setup(TMC429_CS_PIN, SPI_CHANNEL) != 0)
  {
    std::cerr << "Stepper driver initialization failed." << std::endl;
    return -1;
  }
  std::cerr << "TMC429 version: " << tmc429.getVersion() << std::endl;
  std::cerr << "TMC429 communi: " << tmc429.communicating() << std::endl;


  if(!tmc429.communicating()){
    std::cout << "TMC429 setup and configuration NOT complete!" << std::endl;
    return -1;
  }
  std::cout << "TMC429 setup and configuration complete!" << std::endl;
  usleep(100000); 

  tmc429.setPDiv(MOTOR_INDEX, 4);
  tmc429.setPMul(MOTOR_INDEX, 128);
  tmc429.setPulseDiv(MOTOR_INDEX, 5);
  tmc429.setRampDiv(MOTOR_INDEX, 13);

  tmc429.setStepDiv(15);
  tmc429.setStepDirOutput();

  tmc429.setVelocityMode(MOTOR_INDEX);
  tmc429.disableLeftSwitchStop(MOTOR_INDEX);
  tmc429.disableRightSwitchStop(MOTOR_INDEX);
  tmc429.disableInverseStepPolarity();
  tmc429.disableInverseDirPolarity();

  tmc429.setVelocityMin(MOTOR_INDEX, 0);
  usleep(1000);
  tmc429.setVelocityMax(MOTOR_INDEX, 2047);
  usleep(1000);
  tmc429.setAccelerationMaxInStepPerSS(MOTOR_INDEX, 2000);
  usleep(1000);  
  tmc429.setTargetVelocity(MOTOR_INDEX, 2047);
  std::cout << "Target Velocity: " <<  tmc429.getTargetVelocity(MOTOR_INDEX) << std::endl;
  

  

  time_t start_time, elapsed_time;
  start_time = time(0);
  elapsed_time = 10; // [s]

  TMC2130::Status tmc_status;
  tmc_status = tmc2130.getStatus();
  std::cout << "Over temp warning: " << tmc_status.over_temperature_warning << std::endl;

  while(1)
  {

    usleep(500000);
    // tmc429.printSettingsMotor0();
    // usleep(100000);
    // tmc2130.printSettings();

    std::cout << float(time(0) - start_time) << std::endl;


    if((time(0) - start_time) >= elapsed_time) break;

  }

  tmc429.setTargetVelocity(MOTOR_INDEX, 0);
  std::cout << "Loop complete." << std::endl;
  tmc429.stop(MOTOR_INDEX);

  return 0;
}
