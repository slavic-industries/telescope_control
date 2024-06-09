/*
 * @file Adafruit_SI5351.h
 */

#ifndef _SI5351_H_
#define _SI5351_H_

#include <wiringPiI2C.h>
#include <cassert>
#include <iostream>
#include <cmath>


#define SI5351_ADDRESS (0x60) // Assumes ADDR pin = low
#define SI5351_READBIT (0x01)

/* Test setup from SI5351 ClockBuilder
 * -----------------------------------
 * XTAL:      25     MHz
 * Channel 0: 120.00 MHz
 * Channel 1: 12.00  MHz
 * Channel 2: 13.56  MHz
 */
static const uint8_t m_si5351_regs_15to92_149to170[100][2] = {
    {15, 0x00}, /* Input source = crystal for PLLA and PLLB */
    {16, 0x4F}, /* CLK0 Control: 8mA drive, Multisynth 0 as CLK0 source, Clock
                   not inverted, Source = PLLA, Multisynth 0 in integer mode,
                   clock powered up */
    {17, 0x4F}, /* CLK1 Control: 8mA drive, Multisynth 1 as CLK1 source, Clock
                   not inverted, Source = PLLA, Multisynth 1 in integer mode,
                   clock powered up */
    {18, 0x6F}, /* CLK2 Control: 8mA drive, Multisynth 2 as CLK2 source, Clock
                   not inverted, Source = PLLB, Multisynth 2 in integer mode,
                   clock powered up */
    {19, 0x80}, /* CLK3 Control: Not used ... clock powered down */
    {20, 0x80}, /* CLK4 Control: Not used ... clock powered down */
    {21, 0x80}, /* CLK5 Control: Not used ... clock powered down */
    {22, 0x80}, /* CLK6 Control: Not used ... clock powered down */
    {23, 0x80}, /* CLK7 Control: Not used ... clock powered down */
    {24, 0x00}, /* Clock disable state 0..3 (low when disabled) */
    {25, 0x00}, /* Clock disable state 4..7 (low when disabled) */
    /* PLL_A Setup */
    {26, 0x00},
    {27, 0x05},
    {28, 0x00},
    {29, 0x0C},
    {30, 0x66},
    {31, 0x00},
    {32, 0x00},
    {33, 0x02},
    /* PLL_B Setup */
    {34, 0x02},
    {35, 0x71},
    {36, 0x00},
    {37, 0x0C},
    {38, 0x1A},
    {39, 0x00},
    {40, 0x00},
    {41, 0x86},
    /* Multisynth Setup */
    {42, 0x00},
    {43, 0x01},
    {44, 0x00},
    {45, 0x01},
    {46, 0x00},
    {47, 0x00},
    {48, 0x00},
    {49, 0x00},
    {50, 0x00},
    {51, 0x01},
    {52, 0x00},
    {53, 0x1C},
    {54, 0x00},
    {55, 0x00},
    {56, 0x00},
    {57, 0x00},
    {58, 0x00},
    {59, 0x01},
    {60, 0x00},
    {61, 0x18},
    {62, 0x00},
    {63, 0x00},
    {64, 0x00},
    {65, 0x00},
    {66, 0x00},
    {67, 0x00},
    {68, 0x00},
    {69, 0x00},
    {70, 0x00},
    {71, 0x00},
    {72, 0x00},
    {73, 0x00},
    {74, 0x00},
    {75, 0x00},
    {76, 0x00},
    {77, 0x00},
    {78, 0x00},
    {79, 0x00},
    {80, 0x00},
    {81, 0x00},
    {82, 0x00},
    {83, 0x00},
    {84, 0x00},
    {85, 0x00},
    {86, 0x00},
    {87, 0x00},
    {88, 0x00},
    {89, 0x00},
    {90, 0x00},
    {91, 0x00},
    {92, 0x00},
    /* Misc Config Register */
    {149, 0x00},
    {150, 0x00},
    {151, 0x00},
    {152, 0x00},
    {153, 0x00},
    {154, 0x00},
    {155, 0x00},
    {156, 0x00},
    {157, 0x00},
    {158, 0x00},
    {159, 0x00},
    {160, 0x00},
    {161, 0x00},
    {162, 0x00},
    {163, 0x00},
    {164, 0x00},
    {165, 0x00},
    {166, 0x00},
    {167, 0x00},
    {168, 0x00},
    {169, 0x00},
    {170, 0x00}};

/* See http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf for
 * registers 26..41 */
enum {
  REG_0_DEVICE_STATUS = 0,
  REG_1_INTERRUPT_STATUS_STICKY = 1,
  REG_2_INTERRUPT_STATUS_MASK = 2,
  REG_3_OUTPUT_ENABLE_CONTROL = 3,
  REG_9_OEB_PIN_ENABLE_CONTROL = 9,
  REG_15_PLL_INPUT_SOURCE = 15,
  REG_16_CLK0_CONTROL = 16,
  REG_17_CLK1_CONTROL = 17,
  REG_18_CLK2_CONTROL = 18,
  REG_19_CLK3_CONTROL = 19,
  REG_20_CLK4_CONTROL = 20,
  REG_21_CLK5_CONTROL = 21,
  REG_22_CLK6_CONTROL = 22,
  REG_23_CLK7_CONTROL = 23,
  REG_24_CLK3_0_DISABLE_STATE = 24,
  REG_25_CLK7_4_DISABLE_STATE = 25,
  REG_42_MULTISYNTH0_PARAMETERS_1 = 42,
  REG_43_MULTISYNTH0_PARAMETERS_2 = 43,
  REG_44_MULTISYNTH0_PARAMETERS_3 = 44,
  REG_45_MULTISYNTH0_PARAMETERS_4 = 45,
  REG_46_MULTISYNTH0_PARAMETERS_5 = 46,
  REG_47_MULTISYNTH0_PARAMETERS_6 = 47,
  REG_48_MULTISYNTH0_PARAMETERS_7 = 48,
  REG_49_MULTISYNTH0_PARAMETERS_8 = 49,
  REG_50_MULTISYNTH1_PARAMETERS_1 = 50,
  REG_51_MULTISYNTH1_PARAMETERS_2 = 51,
  REG_52_MULTISYNTH1_PARAMETERS_3 = 52,
  REG_53_MULTISYNTH1_PARAMETERS_4 = 53,
  REG_54_MULTISYNTH1_PARAMETERS_5 = 54,
  REG_55_MULTISYNTH1_PARAMETERS_6 = 55,
  REG_56_MULTISYNTH1_PARAMETERS_7 = 56,
  REG_57_MULTISYNTH1_PARAMETERS_8 = 57,
  REG_58_MULTISYNTH2_PARAMETERS_1 = 58,
  REG_59_MULTISYNTH2_PARAMETERS_2 = 59,
  REG_60_MULTISYNTH2_PARAMETERS_3 = 60,
  REG_61_MULTISYNTH2_PARAMETERS_4 = 61,
  REG_62_MULTISYNTH2_PARAMETERS_5 = 62,
  REG_63_MULTISYNTH2_PARAMETERS_6 = 63,
  REG_64_MULTISYNTH2_PARAMETERS_7 = 64,
  REG_65_MULTISYNTH2_PARAMETERS_8 = 65,
  REG_66_MULTISYNTH3_PARAMETERS_1 = 66,
  REG_67_MULTISYNTH3_PARAMETERS_2 = 67,
  REG_68_MULTISYNTH3_PARAMETERS_3 = 68,
  REG_69_MULTISYNTH3_PARAMETERS_4 = 69,
  REG_70_MULTISYNTH3_PARAMETERS_5 = 70,
  REG_71_MULTISYNTH3_PARAMETERS_6 = 71,
  REG_72_MULTISYNTH3_PARAMETERS_7 = 72,
  REG_73_MULTISYNTH3_PARAMETERS_8 = 73,
  REG_74_MULTISYNTH4_PARAMETERS_1 = 74,
  REG_75_MULTISYNTH4_PARAMETERS_2 = 75,
  REG_76_MULTISYNTH4_PARAMETERS_3 = 76,
  REG_77_MULTISYNTH4_PARAMETERS_4 = 77,
  REG_78_MULTISYNTH4_PARAMETERS_5 = 78,
  REG_79_MULTISYNTH4_PARAMETERS_6 = 79,
  REG_80_MULTISYNTH4_PARAMETERS_7 = 80,
  REG_81_MULTISYNTH4_PARAMETERS_8 = 81,
  REG_82_MULTISYNTH5_PARAMETERS_1 = 82,
  REG_83_MULTISYNTH5_PARAMETERS_2 = 83,
  REG_84_MULTISYNTH5_PARAMETERS_3 = 84,
  REG_85_MULTISYNTH5_PARAMETERS_4 = 85,
  REG_86_MULTISYNTH5_PARAMETERS_5 = 86,
  REG_87_MULTISYNTH5_PARAMETERS_6 = 87,
  REG_88_MULTISYNTH5_PARAMETERS_7 = 88,
  REG_89_MULTISYNTH5_PARAMETERS_8 = 89,
  REG_90_MULTISYNTH6_PARAMETERS = 90,
  REG_91_MULTISYNTH7_PARAMETERS = 91,
  REG_092_CLOCK_6_7_OUTPUT_DIVIDER = 92,
  REG_149_SPREAD_SPECTRUM_PARAMETERS = 149,
  REG_165_CLK0_INITIAL_PHASE_OFFSET = 165,
  REG_166_CLK1_INITIAL_PHASE_OFFSET = 166,
  REG_167_CLK2_INITIAL_PHASE_OFFSET = 167,
  REG_168_CLK3_INITIAL_PHASE_OFFSET = 168,
  REG_169_CLK4_INITIAL_PHASE_OFFSET = 169,
  REG_170_CLK5_INITIAL_PHASE_OFFSET = 170,
  REG_177_PLL_RESET = 177,
  REG_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
};

typedef enum {
  SI5351_PLL_A = 0,
  SI5351_PLL_B,
} si5351PLL_t;

typedef enum {
  SI5351_CRYSTAL_LOAD_6PF = (1 << 6),
  SI5351_CRYSTAL_LOAD_8PF = (2 << 6),
  SI5351_CRYSTAL_LOAD_10PF = (3 << 6)
} si5351CrystalLoad_t;

typedef enum {
  SI5351_CRYSTAL_FREQ_25MHZ = (25000000),
  SI5351_CRYSTAL_FREQ_27MHZ = (27000000)
} si5351CrystalFreq_t;

typedef enum {
  SI5351_MULTISYNTH_DIV_4 = 4,
  SI5351_MULTISYNTH_DIV_6 = 6,
  SI5351_MULTISYNTH_DIV_8 = 8
} si5351MultisynthDiv_t;

typedef enum {
  SI5351_R_DIV_1 = 0,
  SI5351_R_DIV_2 = 1,
  SI5351_R_DIV_4 = 2,
  SI5351_R_DIV_8 = 3,
  SI5351_R_DIV_16 = 4,
  SI5351_R_DIV_32 = 5,
  SI5351_R_DIV_64 = 6,
  SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

/*!
 * @brief SI5351 constructor
 */
typedef struct {
  bool initialised;                 //!< Initialization status of SI5351
  si5351CrystalFreq_t crystalFreq;  //!< Crystal frequency
  si5351CrystalLoad_t crystalLoad;  //!< Crystal load capacitors
  uint32_t crystalPPM;              //!< Frequency synthesis
  bool plla_configured;             //!< Phase-locked loop A configured
  uint32_t plla_freq;               //!< Phase-locked loop A frequency
  bool pllb_configured;             //!< Phase-locked loop B configured
  uint32_t pllb_freq;               //!< Phase-locked loop B frequency
  uint8_t i2c_addr;                 //!< I2C Address for the SI5351 
  uint16_t i2c_td;                       //!< I2C Target device
} si5351Config_t;

/*!
 * @brief SI5351 class
 */
class SI5351 {
public:
  SI5351(uint8_t i2c_addr); //!< SI5351 object

  uint8_t begin();
  uint8_t setClockBuilderData(void);

  uint8_t setupPLL(si5351PLL_t pll, uint8_t mult, uint32_t num, uint32_t denom);                 
  uint8_t setupPLLInt(si5351PLL_t pll, uint8_t mult); 

  uint8_t setupMultisynth(uint8_t output, si5351PLL_t pllSource, uint32_t div, uint32_t num, uint32_t denom); 
  uint8_t setupMultisynthInt(uint8_t output, si5351PLL_t pllSource, si5351MultisynthDiv_t div);
  
  uint8_t enableSpreadSpectrum(bool enabled);
  uint8_t enableOutputs(bool enabled);
  /*!
   * @param output Enables or disables output
   * @param div Set of output divider values (2^n, n=1..7)
   */
  uint8_t setupRdiv(uint8_t output, si5351RDiv_t div); //!< @return ERROR_NONE

private:
  si5351Config_t m_si5351Config;

  

  uint8_t write8(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);

  uint8_t lastRdivValue[3];
};

#endif
