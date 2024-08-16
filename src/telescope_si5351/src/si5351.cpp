#include "telescope_si5351/si5351.h"

SI5351::SI5351(const char* i2c_device, uint8_t i2c_addr)
{
  m_si5351Config.initialised = false;
  m_si5351Config.crystalFreq = SI5351_CRYSTAL_FREQ_25MHZ;
  m_si5351Config.crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
  m_si5351Config.crystalPPM = 30;
  m_si5351Config.plla_configured = false;
  m_si5351Config.plla_freq = 0;
  m_si5351Config.pllb_configured = false;
  m_si5351Config.pllb_freq = 0;
  m_si5351Config.i2c_addr = i2c_addr;
  m_si5351Config.i2c_device = i2c_device;

  

  for (uint8_t i = 0; i < 3; i++) {
    lastRdivValue[i] = 0;
  }
}

uint8_t SI5351::begin()
{
  // if (gpioInitialise() < 0) {
  //   std::cerr << "pigpio initialization failed" << std::endl;
  //   return 1;
  // }

  m_si5351Config.i2c_handle = wiringPiI2CSetupInterface(m_si5351Config.i2c_device, m_si5351Config.i2c_addr);
  if (m_si5351Config.i2c_handle < 0) {
      // std::cerr << "Failed to initialize I2C communication. Error: " << ret_val << std::endl;
      return -1;
  }
  
  // Disable all outputs before setting registers
  enableOutputs(false);
  

  // Power down all output drivers
  write8(REG_16_CLK0_CONTROL, 0x80);
  write8(REG_17_CLK1_CONTROL, 0x80);
  write8(REG_18_CLK2_CONTROL, 0x80);
  write8(REG_19_CLK3_CONTROL, 0x80);
  write8(REG_20_CLK4_CONTROL, 0x80);
  write8(REG_21_CLK5_CONTROL, 0x80);
  write8(REG_22_CLK6_CONTROL, 0x80);
  write8(REG_23_CLK7_CONTROL, 0x80);

  
  write8(REG_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE, m_si5351Config.crystalLoad);
  enableSpreadSpectrum(false);

  // Reset the PLL config fields just in case we call init again
  m_si5351Config.plla_configured = false;
  m_si5351Config.plla_freq = 0;
  m_si5351Config.pllb_configured = false;
  m_si5351Config.pllb_freq = 0;

  // All done! 
  m_si5351Config.initialised = true;
  return 0;
}

uint8_t SI5351::setupPLL(si5351PLL_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{

  uint32_t P1; /* PLL config register P1 */
  uint32_t P2; /* PLL config register P2 */
  uint32_t P3; /* PLL config register P3 */

  /* Basic validation */
  assert(m_si5351Config.initialised);
  assert((mult > 14) && (mult < 91));                   /* mult = 15..90 */
  assert(denom > 0);        /* Avoid divide by zero */
  assert(num <= 0xFFFFF);   /* 20-bit limit */
  assert(denom <= 0xFFFFF); /* 20-bit limit */

  /* Feedback Multisynth Divider Equation
   *
   * where: a = mult, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = denom
   */

  /* Set the main PLL config registers */
  if (num == 0) {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = num;
    P3 = denom;
  } else {
    /* Fractional mode */
    P1 =
        (uint32_t)(128 * mult + floor(128 * ((float)num / (float)denom)) - 512);
    P2 = (uint32_t)(128 * num -
                    denom * floor(128 * ((float)num / (float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  write8(baseaddr,     (P3 & 0x0000FF00) >> 8);
  write8(baseaddr + 1, (P3 & 0x000000FF));
  write8(baseaddr + 2, (P1 & 0x00030000) >> 16);
  write8(baseaddr + 3, (P1 & 0x0000FF00) >> 8);
  write8(baseaddr + 4, (P1 & 0x000000FF));
  write8(baseaddr + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  write8(baseaddr + 6, (P2 & 0x0000FF00) >> 8);
  write8(baseaddr + 7, (P2 & 0x000000FF));

  /* Reset both PLLs */
  write8(REG_177_PLL_RESET, (1 << 7) | (1 << 5));

  /* Store the frequency settings for use with the Multisynth helper */
  if (pll == SI5351_PLL_A) {
    float fvco =
        m_si5351Config.crystalFreq * (mult + ((float)num / (float)denom));
    m_si5351Config.plla_configured = true;
    m_si5351Config.plla_freq = (uint32_t)floor(fvco);
  } else {
    float fvco =
        m_si5351Config.crystalFreq * (mult + ((float)num / (float)denom));
    m_si5351Config.pllb_configured = true;
    m_si5351Config.pllb_freq = (uint32_t)floor(fvco);
  }

  return 0;
}

uint8_t SI5351::setupPLLInt(si5351PLL_t pll, uint8_t mult)
{
  return setupPLL(pll, mult, 0, 1);
}

uint8_t SI5351::setupMultisynth(uint8_t output, si5351PLL_t pllSource, uint32_t div, uint32_t num, uint32_t denom)
{
  uint32_t P1; /* Multisynth config register P1 */
  uint32_t P2; /* Multisynth config register P2 */
  uint32_t P3; /* Multisynth config register P3 */

  /* Basic validation */
  assert(m_si5351Config.initialised);
  assert(output < 3);       /* Channel range */
  assert(div > 3);          /* Divider integer value */
  assert(div < 2049);       /* Divider integer value */
  assert(denom > 0);        /* Avoid divide by zero */
  assert(num <= 0xFFFFF);   /* 20-bit limit */
  assert(denom <= 0xFFFFF); /* 20-bit limit */

  /* Make sure the requested PLL has been initialised */
  if (pllSource == SI5351_PLL_A) {
    assert(m_si5351Config.plla_configured);
  } else {
    assert(m_si5351Config.pllb_configured);
  }

  /* Output Multisynth Divider Equations
   *
   * where: a = div, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = c
   */

  /* Set the main PLL config registers */
  if (num == 0) {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = 0;
    P3 = denom;
  } else if (denom == 1) {
    /* Fractional mode, simplified calculations */
    P1 = 128 * div + 128 * num - 512;
    P2 = 128 * num - 128;
    P3 = 1;
  } else {
    /* Fractional mode */
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num / (float)denom)) - 512);
    P2 = (uint32_t)(128 * num -
                    denom * floor(128 * ((float)num / (float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = 0;
  switch (output) {
  case 0:
    baseaddr = REG_42_MULTISYNTH0_PARAMETERS_1;
    break;
  case 1:
    baseaddr = REG_50_MULTISYNTH1_PARAMETERS_1;
    break;
  case 2:
    baseaddr = REG_58_MULTISYNTH2_PARAMETERS_1;
    break;
  }

  // Set the MSx config registers
  write8(baseaddr,        (P3 & 0xFF00) >> 8);
  write8(baseaddr + 1,    P3 & 0xFF);
  write8(baseaddr + 2,    ((P1 & 0x30000) >> 16) | 0);
  write8(baseaddr + 3,    (P1 & 0xFF00) >> 8);
  write8(baseaddr + 4,    P1 & 0xFF);
  write8(baseaddr + 5,    ((P3 & 0xF0000) >> 12) | ((P2 & 0xF0000) >> 16));
  write8(baseaddr + 6,    (P2 & 0xFF00) >> 8);
  write8(baseaddr + 7,    P2 & 0xFF);

  /* Configure the clk control and enable the output */
  /* TODO: Check if the clk control byte needs to be updated. */
  uint8_t clkControlReg = 0x0F; /* 8mA drive strength, MS0 as CLK0 source, Clock
                                   not inverted, powered up */
  if (pllSource == SI5351_PLL_B)
    clkControlReg |= (1 << 5); /* Uses PLLB */
  if (num == 0)
    clkControlReg |= (1 << 6); /* Integer mode */
  switch (output) {
  case 0:
    write8(REG_16_CLK0_CONTROL, clkControlReg);
    break;
  case 1:
    write8(REG_17_CLK1_CONTROL, clkControlReg);
    break;
  case 2:
    write8(REG_18_CLK2_CONTROL, clkControlReg);
    break;
  }

  return 0;
}

uint8_t SI5351::setupMultisynthInt(uint8_t output, si5351PLL_t pllSource, si5351MultisynthDiv_t div)
{
  return setupMultisynth(output, pllSource, div, 0, 1);
}

uint8_t SI5351::setupRdiv(uint8_t output, si5351RDiv_t div)
{
  assert(output < 3); /* Channel range */

  uint8_t Rreg, regval;

  if (output == 0)
    Rreg = REG_44_MULTISYNTH0_PARAMETERS_3;
  if (output == 1)
    Rreg = REG_52_MULTISYNTH1_PARAMETERS_3;
  if (output == 2)
    Rreg = REG_60_MULTISYNTH2_PARAMETERS_3;

  regval = read8(Rreg);

  regval &= 0x0F;
  uint8_t divider = div;
  divider &= 0x07;
  divider <<= 4;
  regval |= divider;
  lastRdivValue[output] = divider;
  return write8(Rreg, regval);
}

uint8_t SI5351::enableSpreadSpectrum(bool enabled)
{
  uint8_t regval;
  regval = read8(REG_149_SPREAD_SPECTRUM_PARAMETERS);

  if (enabled) {
    regval |= 0x80;
  } else {
    regval &= ~0x80;
  }
  return write8(REG_149_SPREAD_SPECTRUM_PARAMETERS, regval);

}

uint8_t SI5351::enableOutputs(bool enabled)
{
    return write8(REG_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00 : 0xFF);
}

uint8_t SI5351::read8(uint8_t reg)
{
  int8_t ret_val;
  // uint16_t i2c_handle = i2cOpen(m_si5351Config.i2c_device, m_si5351Config.i2c_addr, 0);
  // if (i2c_handle) {
  //     std::cerr << "Failed to initialize I2C communication. Error: " << ret_val << std::endl;
  //     return -1;
  // }
  ret_val = wiringPiI2CReadReg8(m_si5351Config.i2c_handle, reg);
  if (ret_val < 0) {
      std::cerr << "Failed to read data with I2C. Error: " << ret_val << std::endl;
      return -1;
  }
  // ret_val = i2cClose(i2c_handle);
  // if (ret_val) {
  //     std::cerr << "Failed to deinitialize I2C communication. Error: " << ret_val << std::endl;
  //     return -1;
  // }
  return 0;
}

uint8_t SI5351::write8(uint8_t reg, uint8_t value)
{
  int8_t ret_val;
  // uint16_t i2c_handle = i2cOpen(m_si5351Config.i2c_device, m_si5351Config.i2c_addr, 0);
  
  ret_val = wiringPiI2CWriteReg8(m_si5351Config.i2c_handle, reg, value);
  if (ret_val < 0) {
      std::cerr << "Failed to send data with I2C. Error: " << ret_val << std::endl;
      return -1;
  }
  // ret_val = i2cClose(i2c_handle);
  // if (ret_val) {
  //     std::cerr << "Failed to deinitialize I2C communication. Error: " << ret_val << std::endl;
  //     return -1;
  // }
  return 0;
}