
#include <Wire.h>
#include <Adafruit_SI5351.h>
#include <TMC2130Stepper.h>
#include <TMC429.h>
#include "SPI.h"

#define SERIAL_BAUD_RATE        115200

#define SPI_MOSI_PIN            16
#define SPI_MISO_PIN            14
#define SPI_CLK_PIN             15

#define TMC429_CS_PIN           10
#define TMC429_CLK_FREQ         32
#define TMC429_MODE             0

#define TMC2130_CS_PIN          9
#define TMC2130_EN_PIN          8
#define TMC2130_DIR_PIN         7
#define TMC2130_STEP_PIN        6
#define TMC2130_R_SENSE         0.11f

bool dir = true;

const int LOOP_DELAY = 500;


Adafruit_SI5351 clockgen = Adafruit_SI5351();

TMC429 tmc429;
TMC2130Stepper tmc2130 = TMC2130Stepper(TMC2130_EN_PIN, TMC2130_DIR_PIN, TMC2130_STEP_PIN, TMC2130_CS_PIN);

const int MICROSTEPS_PER_STEP = 256;
const int CLOCK_FREQUENCY_MHZ = 32;
const int MOTOR_INDEX = 0;
const int STEPS_PER_REV = 200;
const int REVS_PER_SEC_MAX = 2;
const int MICROSTEPS_PER_REV = STEPS_PER_REV*MICROSTEPS_PER_STEP;
const int ACCELERATION_MAX = MICROSTEPS_PER_REV / 8;
const long VELOCITY_MAX = REVS_PER_SEC_MAX * MICROSTEPS_PER_REV;
const long VELOCITY_MIN = 50;
const long VELOCITY_INC = 5000;

long target_velocity, actual_velocity, delta_velocity;
bool at_target_velocity;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    while(!Serial);

    // Initialize DS3231 32MHz Clock
    if (clockgen.begin() != ERROR_NONE)
    {
        Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    // Setup Output 2 to 31.9 MHz
    // 25MHz * 30 = 750MHz -> setupPLLInt()
    // 750MHz / (23 + 1/2) = 31.9 MHz -> setupMultisynth()
    clockgen.setupPLLInt(SI5351_PLL_A, 30);
    clockgen.setupMultisynth(2, SI5351_PLL_A, 23, 1, 2);

  
    clockgen.enableOutputs(true);
    delay(100);

    // *****************************************************
    // TMC2130 
    SPI.begin();
    pinMode(MISO, INPUT_PULLUP);
    tmc2130.begin(); 			// Initiate pins and registeries
    tmc2130.rms_current(50); 	// Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    tmc2130.stealthChop(1); 	// Enable extremely quiet stepping
    
    digitalWrite(TMC2130_EN_PIN, LOW);


  

    // *****************************************************
    // Initialize TMC429
    tmc429.setup(TMC429_CS_PIN,TMC429_CLK_FREQ); 
    if(!tmc429.communicating()) 
    {
        Serial.println("TMC 429 communication not established.");
    }
    else
    {
            Serial.println("TMC 429 communication established.");
    }

    tmc429.disableLeftSwitchStop(MOTOR_INDEX);
    tmc429.disableRightSwitches();
    tmc429.setVelocityMode(MOTOR_INDEX);
    tmc429.setLimitsInHz(MOTOR_INDEX, VELOCITY_MIN, VELOCITY_MAX, ACCELERATION_MAX);

    target_velocity = 50000;
    delta_velocity = VELOCITY_INC;
    tmc429.setTargetVelocityInHz(MOTOR_INDEX, target_velocity);

    

    Serial.println("Setup complete");
}

void loop()
{ 
/*  

  digitalWrite(TMC2130_STEP_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TMC2130_STEP_PIN, LOW);
	delayMicroseconds(10);
	uint32_t ms = millis();
	static uint32_t last_time = 0;
	if ((ms - last_time) > 2000) {
		if (dir) {
			Serial.println("Dir -> 0");
			tmc2130.shaft_dir(0);
		} else {
			Serial.println("Dir -> 1");
			tmc2130.shaft_dir(1);
		}
		dir = !dir;
		last_time = ms;
	}
*/

/*
  Serial.println("********************");
  Serial.println("Velocity Mode");

  Serial.print("target velocity (Hz): ");
  Serial.println(target_velocity);

  actual_velocity = tmc429.getActualVelocityInHz(MOTOR_INDEX);
  Serial.print("actual velocity (Hz): ");
  Serial.println(actual_velocity);

  at_target_velocity = tmc429.atTargetVelocity(MOTOR_INDEX);
  Serial.print("at target velocity: ");
  Serial.println(at_target_velocity);

  delay(LOOP_DELAY);

  if (at_target_velocity)
  {
    target_velocity += delta_velocity;
    if ((target_velocity > VELOCITY_MAX) || (target_velocity < -VELOCITY_MAX))
    {
      delta_velocity = -delta_velocity;
      target_velocity += 2* delta_velocity;
    }
    tmc429.setTargetVelocityInHz(MOTOR_INDEX, target_velocity);
  }
  */
  
}
