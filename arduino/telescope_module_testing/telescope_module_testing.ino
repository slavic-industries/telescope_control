
#include <Wire.h>
#include <ds3231.h>
#include <TMCStepper.h>
#include <TMC429.h>

#define SERIAL_BAUD_RATE        115200

#define SPI_MOSI_PIN            16
#define SPI_MISO_PIN            14
#define SPI_CLK_PIN             15

#define TMC429_CS_PIN           6
#define TMC429_CLK_FREQ         32
#define TMC429_MODE             0

#define TMC2130_CS_PIN          5
#define TMC2130_R_SENSE         0.11f



TMC429 tmc429;
TMC2130Stepper tmc2130 = TMC2130Stepper(TMC2130_CS_PIN, TMC2130_R_SENSE, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN); 



void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    while(!Serial);

    // Begin I2C for DS3231
    Wire.begin();

    // Initialize DS3231 32MHz Clock
    DS3231_init(DS3231_CONTROL_INTCN);
    DS3231_set_32kHz_output(true);
    delay(100);

    // Initialize TMC429
    tmc429.setup(TMC429_CS_PIN,TMC429_CLK_FREQ);
    if(!tmc429.communicating()) 
    {
        Serial.println("TMC 429 communication not established.");
    }

    tmc2130.begin();
    tmc2130.rms_current(600);  // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    tmc2130.en_pwm_mode(1);
    if(!tmc2130.isEnabled()) 
    {   
        Serial.println("TMC 2130 communication not established.");
    }

    Serial.println("Setup complete");
}

void loop()
{   Serial.println(tmc2130.isEnabled());

    if(!tmc2130.isEnabled()) 
    {
        
        Serial.println("TMC 2130 communication not established.");
    }
    delay(1000);
}
