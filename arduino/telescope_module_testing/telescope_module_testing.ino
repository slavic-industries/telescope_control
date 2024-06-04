
#include <Wire.h>
#include <Adafruit_SI5351.h>
#include <TMCStepper.h>
#include <TMC429.h>

#define SERIAL_BAUD_RATE        115200

#define SPI_MOSI_PIN            16
#define SPI_MISO_PIN            14
#define SPI_CLK_PIN             15

#define TMC429_CS_PIN           6
#define TMC429_CLK_FREQ         21
#define TMC429_MODE             0

#define TMC2130_CS_PIN          5
#define TMC2130_R_SENSE         0.11f


Adafruit_SI5351 clockgen = Adafruit_SI5351();

TMC429 tmc429;
TMC2130Stepper tmc2130 = TMC2130Stepper(TMC2130_CS_PIN, TMC2130_R_SENSE, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN); 



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
    // Setup PLLB to fractional mode @616.66667MHz (XTAL * 24 + 2/3)
    // Setup Output 2 to 21 MHz
    clockgen.setupPLL(SI5351_PLL_B, 24, 2, 3);
    clockgen.setupMultisynth(2, SI5351_PLL_B, 900, 0, 1);
    clockgen.setupRdiv(2, SI5351_R_DIV_32);
    clockgen.enableOutputs(true);
    delay(100);

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

    tmc2130.begin();
    tmc2130.rms_current(600);  // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    tmc2130.en_pwm_mode(1);

    Serial.println(tmc2130.DRV_STATUS(), BIN);
    if(!tmc2130.isEnabled()) 
    {   
        Serial.println("TMC 2130 communication not established.");
    }
    else
    {
            Serial.println("TMC 2130 communication established.");
    }

    Serial.println("Setup complete");
}

void loop()
{   
    // Serial.println(tmc2130.isEnabled());

    // if(!tmc2130.isEnabled()) 
    // {
        
    //     Serial.println("TMC 2130 communication not established.");
    // }
    // delay(1000);
}
