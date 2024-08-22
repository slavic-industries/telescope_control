// ----------------------------------------------------------------------------
// TMC429.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
//
// Modified to work with pigpio library
// ----------------------------------------------------------------------------
#include "telescope_drive/tmc429.h"


TMC429::TMC429()
{
    // wiringPiSetup();
  // std::cerr << "TMC429 object created" << std::endl;
}

TMC429::~TMC429()
{ 
  stopAll();
  if(initialized)
  {
    // wiringPiSPIClose(spi_channel);
    initialized = false;
  }
  std::cerr << "TMC429 object destroyed" << std::endl;
}

uint8_t TMC429::setup(size_t chip_select_pin, uint8_t spi_device)
{
  this->chip_select_pin_ = chip_select_pin;
  this->spi_channel = spi_device;
  this->spi_channel = 1;

  std::cout << "spi_channel = " << int(this->spi_channel) << std::endl;

  if (wiringPiSetup() != 0) {
    std::cerr << "pigpio initialization failed" << std::endl;
    return 1;
  }

  pinMode(this->chip_select_pin_, OUTPUT);
  digitalWrite(this->chip_select_pin_, HIGH);

  // spi_handle = spiOpen(spi_channel, SPI_CLOCK, 0x03);
  spi_handle = wiringPiSPISetup(int(this->spi_channel), 1000000);
  std::cout << "spi_channel = " << int(this->spi_channel) << std::endl;
  std::cout << "spi_handle = " << int(spi_handle) << std::endl;
  if (spi_handle == -1) {
    std::cout << "Failed to initialize SPI!" << std::endl;
    return 2;
  }
  // std::cout << "TMC429 SPI Handle (setup): " << std::hex << spi_handle << std::endl;

  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    pulse_div_[motor] = 0;
    ramp_div_[motor] = 0;
  }

  // setStepDiv(STEP_DIV_MAX);
  // setStepDirOutput();

  // stopAll();


  // initialize();

  // printSettingsMotor0();

  initialized = true;

  return 0;
}

void TMC429::setPMul(size_t motor, uint8_t pmul)
{
  uint32_t rd = readRegister(motor, ADDRESS_PROP_FACTOR);
  uint32_t mask = (pmul << 8);
  // Clear pmul bits
  rd &= 0xFFFF00FF;
  // Add new pmul
  rd |= mask;
  writeRegister(motor, ADDRESS_PROP_FACTOR, rd);
}
void TMC429::setPDiv(size_t motor, uint8_t pdiv)
{
  uint32_t rd = readRegister(motor, ADDRESS_PROP_FACTOR);
  uint32_t mask = pdiv;
  // Clear pdiv bits
  rd &= 0xFFFFFF00;
  // Add new pmul
  rd |= mask;
  writeRegister(motor, ADDRESS_PROP_FACTOR, rd);
}

void TMC429::setPulseDiv(size_t motor, uint8_t pulsediv)
{
  uint32_t rd = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  uint32_t mask = pulsediv << 12;
  // Clear pdiv bits
  rd &= 0xFFFF0FFF;
  // Add new pmul
  rd |= mask;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, rd);
}

void TMC429::setRampDiv(size_t motor, uint8_t rampdiv)
{
  uint32_t rd = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  uint32_t mask = rampdiv << 8;
  // Clear pdiv bits
  rd &= 0xFFFFF0FF;
  // Add new pmul
  rd |= mask;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, rd);
}

void TMC429::printSettingsMotor0()
{ 
  uint32_t rd;
  size_t motor = 0;
  Status status_;
  PropFactor propfactor_;
  RefConfMode ref_conf_mode_;
  IfConf if_conf_;
  SwState sw_state_;
  GlobalParameters global_parameters_;
  ClkConfig clk_config_;

  status_ = getStatus();
  propfactor_.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  ref_conf_mode_.ref_conf = getReferenceConfiguration(motor);
  if_conf_.if_conf = getInterfaceConfiguration();
  sw_state_.switch_state = getSwitchState();
  global_parameters_.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  clk_config_.clk_config = getClockConfiguration(motor);

  uint32_t x_target = getTargetPosition(motor);
  uint32_t x_actual = getActualPosition(motor);
  uint16_t v_min = getVelocityMin(motor);
  uint16_t v_max = getVelocityMax(motor);
  uint16_t v_target = getTargetVelocity(motor);
  uint16_t v_actual = getActualVelocity(motor);
  uint16_t a_max = getAccelerationMax(motor);
  uint16_t a_actual = getActualAcceleration(motor);
  uint16_t a_threash;      // ADDRESS_A_THRESHOLD
  rd = readRegister(motor, ADDRESS_A_THRESHOLD);
  a_threash = rd & 0xFFF;
  uint16_t dx_ref_tol;    // ADDRESS_DX_REF_TOLERANCE
  rd = readRegister(motor, ADDRESS_DX_REF_TOLERANCE);
  dx_ref_tol = rd & 0xFFF;
  uint32_t x_latched = getLatchPosition(motor);
  uint8_t ustep_count_429;   // ADDRESS_USTEP_COUNT_429
  rd = readRegister(motor, ADDRESS_USTEP_COUNT_429);
  ustep_count_429 = rd & 0xFF;

  std::cout << "Motor index:\t" << std::dec << motor << std::endl;
  std::cout << "\tX_TARGET:\t" << std::dec << x_target << std::endl;
  std::cout << "\tX_TARGET:\t" << std::bitset<32>(x_target) << std::endl;
  std::cout << "\tX_ACTUAL:\t" << std::dec << x_actual << std::endl;
  std::cout << "\tX_ACTUAL:\t" << std::bitset<32>(x_actual) << std::endl;
  std::cout << "\tV_MIN:\t\t" << std::dec << v_min << std::endl;
  std::cout << "\tV_MIN:\t\t" << std::bitset<16>(v_min) << std::endl;
  std::cout << "\tV_MAX:\t\t" << std::dec << v_max << std::endl;
  std::cout << "\tV_MAX:\t\t" << std::bitset<16>(v_max) << std::endl;
  std::cout << "\tV_TARGET:\t" << std::dec << v_target << std::endl;
  std::cout << "\tV_TARGET:\t" << std::bitset<16>(v_target) << std::endl;
  std::cout << "\tV_ACTUAL:\t" << std::dec << v_actual << std::endl;
  std::cout << "\tA_MAX:\t\t" << std::dec << a_max << std::endl;
  std::cout << "\tA_MAX:\t\t" << std::bitset<16>(a_max) << std::endl;
  std::cout << "\tA_ACTUAL:\t" << std::dec << a_actual << std::endl;
  std::cout << "\tA_THREASH:\t" << std::dec << a_threash << std::endl;
  std::cout << "\tDX Ref Tol:\t" << std::dec << dx_ref_tol << std::endl;
  std::cout << "\tX_Latch:\t" << std::dec << x_latched << std::endl;
  std::cout << "\tuStep Count:\t" << std::bitset<8>(ustep_count_429) << std::endl;
  
  std::cout << "Datagrams:"<< std::endl;
  // std::cout << "\tStatus:\t" << std::endl;
  // std::cout << "\t  at_target_0\t\t" << std::bitset<1>(status_.at_target_position_0) << std::endl;
  // std::cout << "\t  switch_l_0\t\t" << std::bitset<1>(status_.switch_left_0) << std::endl;
  // std::cout << "\t  at_target_1\t\t" << std::bitset<1>(status_.at_target_position_1) << std::endl;
  // std::cout << "\t  switch_l_1\t\t" << std::bitset<1>(status_.switch_left_1) << std::endl;
  // std::cout << "\t  at_target_2\t\t" << std::bitset<1>(status_.at_target_position_2) << std::endl;
  // std::cout << "\t  switch_l_2\t\t" << std::bitset<1>(status_.switch_left_2) << std::endl;
  // std::cout << "\t  datagram_wait\t\t" << std::bitset<1>(status_.cover_datagram_waiting) << std::endl;
  // std::cout << "\t  interrupt\t\t" << std::bitset<1>(status_.interrupt) << std::endl;

  std::cout << "\tProp Factor:\t" << std::endl; // std::bitset<32>(propfactor_.bytes) << std::endl;
  std::cout << "\t  pdiv\t\t" << std::bitset<4>(propfactor_.pdiv) << std::endl;
  std::cout << "\t  pdiv\t\t" << std::dec << propfactor_.pdiv << std::endl;
  std::cout << "\t  pmul\t\t" << std::bitset<8>(propfactor_.pmul) << std::endl;
  std::cout << "\t  pmul\t\t" << std::dec << propfactor_.pmul << std::endl;

  // std::cout << "\tRef Conf:\t" << std::endl; // std::bitset<32>(ref_conf_mode_.bytes) << std::endl;
  // std::cout << "\t  mode\t\t" << std::bitset<2>(ref_conf_mode_.mode) << std::endl;
  // std::cout << "\t  dis_stop_l\t" << std::bitset<1>(ref_conf_mode_.ref_conf.disable_stop_l) << std::endl;
  // std::cout << "\t  dis_stop_r\t" << std::bitset<1>(ref_conf_mode_.ref_conf.disable_stop_r) << std::endl;
  // std::cout << "\t  soft stop\t" << std::bitset<1>(ref_conf_mode_.ref_conf.soft_stop) << std::endl;
  // std::cout << "\t  ref_RnL\t" << std::bitset<1>( ref_conf_mode_.ref_conf.ref_rnl) << std::endl;
  // std::cout << "\t  lp\t\t" << std::bitset<1>(ref_conf_mode_.lp) << std::endl;
  
  std::cout << "\tIf Conf:\t" << std::endl; // std::bitset<32>(if_conf_.bytes) << std::endl;
  // std::cout << "\t  inv_ref\t" << std::bitset<1>(if_conf_.if_conf.inv_ref) << std::endl;
  // std::cout << "\t  sdo_int\t" << std::bitset<1>(if_conf_.if_conf.sdo_int) << std::endl;
  // std::cout << "\t  step_half\t" << std::bitset<1>(if_conf_.if_conf.step_half) << std::endl;
  // std::cout << "\t  inv_step\t" << std::bitset<1>(if_conf_.if_conf.inv_stp) << std::endl;
  // std::cout << "\t  inv_dir\t" << std::bitset<1>(if_conf_.if_conf.inv_dir) << std::endl;
  std::cout << "\t  en_sd\t\t" << std::bitset<1>(if_conf_.if_conf.en_sd) << std::endl;
  // std::cout << "\t  pos_comp_sel\t" << std::bitset<1>(if_conf_.if_conf.pos_comp_sel) << std::endl;
  // std::cout << "\t  en_refr\t" << std::bitset<1>(if_conf_.if_conf.en_refr) << std::endl;
  
  // std::cout << "\tSW State:\t" << std::endl; // std::bitset<32>(sw_state_.bytes) << std::endl;
  // std::cout << "\t  r0\t\t" << std::bitset<1>(sw_state_.switch_state.r0) << std::endl;
  // std::cout << "\t  l0\t\t" << std::bitset<1>(sw_state_.switch_state.l0) << std::endl;
  // std::cout << "\t  r1\t\t" << std::bitset<1>(sw_state_.switch_state.r1) << std::endl;
  // std::cout << "\t  l1\t\t" << std::bitset<1>(sw_state_.switch_state.l1) << std::endl;
  // std::cout << "\t  r2\t\t" << std::bitset<1>(sw_state_.switch_state.r2) << std::endl;
  // std::cout << "\t  l2\t\t" << std::bitset<1>(sw_state_.switch_state.l2) << std::endl;
  
  // std::cout << "\tGlobal Param:\t" << std::endl; // std::bitset<32>(global_parameters_.bytes) << std::endl;
  // std::cout << "\t  lsmod\t\t" << std::bitset<2>(global_parameters_.lsmd) << std::endl;
  // std::cout << "\t  nscs_s\t" << std::bitset<1>(global_parameters_.nscs_s) << std::endl;
  // std::cout << "\t  sck_s\t\t" << std::bitset<1>(global_parameters_.sck_s) << std::endl;
  // std::cout << "\t  ph_ab\t\t" << std::bitset<1>(global_parameters_.ph_ab) << std::endl;
  // std::cout << "\t  fd_ab\t\t" << std::bitset<1>(global_parameters_.fd_ab) << std::endl;
  // std::cout << "\t  dac_ab\t" << std::bitset<1>(global_parameters_.dac_ab) << std::endl;
  // std::cout << "\t  cs_com_ind\t" << std::bitset<1>(global_parameters_.cs_com_ind) << std::endl;
  // std::cout << "\t  clk2_div\t" << std::bitset<8>(global_parameters_.clk2_div) << std::endl;
  // std::cout << "\t  cont_upsdate\t" << std::bitset<1>(global_parameters_.cont_update) << std::endl;
  // std::cout << "\t  ref_mux\t" << std::bitset<1>(global_parameters_.ref_mux) << std::endl;
  // std::cout << "\t  mot1r\t\t" << std::bitset<1>(global_parameters_.mot1r) << std::endl;
  
  std::cout << "\tClock Conf:\t" << std::endl; // std::bitset<32>(clk_config_.bytes) << std::endl;
  std::cout << "\t  usrs\t\t" << std::bitset<3>(clk_config_.clk_config.usrs) << std::endl;
  std::cout << "\t  ramp_div\t" << std::bitset<4>(clk_config_.clk_config.ramp_div) << std::endl;
  std::cout << "\t  ramp_div\t" << std::dec << clk_config_.clk_config.ramp_div << std::endl;
  std::cout << "\t  pulse_div\t" << std::bitset<4>(clk_config_.clk_config.pulse_div) << std::endl;
  std::cout << "\t  pulse_div\t" << std::dec << clk_config_.clk_config.pulse_div << std::endl;
  
}

bool TMC429::communicating()
{
  return (getVersion() == VERSION);
}

uint32_t TMC429::getVersion()
{
  return readRegister(SMDA_COMMON, ADDRESS_TYPE_VERSION_429);
}

void TMC429::setRampMode(size_t motor)
{
  setMode(motor, RAMP_MODE);
}

void TMC429::setSoftMode(size_t motor)
{
  setMode(motor, SOFT_MODE);
}

void TMC429::setHoldMode(size_t motor)
{
  setMode(motor, HOLD_MODE);
}

void TMC429::setVelocityMode(size_t motor)
{
  setMode(motor, VELOCITY_MODE);
}

void TMC429::setLimitsInHz(size_t motor,
  uint32_t velocity_min_hz,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  setOptimalStepDivHz(velocity_max_hz);

  setOptimalPulseDivHz(motor, velocity_max_hz);

  setVelocityMinInHz(motor, velocity_min_hz);

  setVelocityMaxInHz(motor, velocity_max_hz);

  setOptimalRampDivHz(motor, velocity_max_hz, acceleration_max_hz_per_s);

  uint32_t a_max = setAccelerationMaxInHzPerS(motor, velocity_max_hz, acceleration_max_hz_per_s);

  setOptimalPropFactor(motor, a_max);
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz()
{
  return convertVelocityToHz(0, VELOCITY_REGISTER_MAX);
}

uint32_t TMC429::getVelocityMinInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getVelocityMin(motor));
}

uint32_t TMC429::getVelocityMaxInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getVelocityMax(motor));
}

int32_t TMC429::getTargetVelocityInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getTargetVelocity(motor));
}

void TMC429::setTargetVelocityInHz(size_t motor,
  int32_t velocity_hz)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setTargetVelocity(motor, convertVelocityFromHz(pulse_div_[motor], velocity_hz));
}

int16_t TMC429::getTargetVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t velocity_unsigned = readRegister(motor, ADDRESS_V_TARGET);
  return unsignedToSigned(velocity_unsigned, V_BIT_COUNT);
}

void TMC429::setTargetVelocity(size_t motor,
  int16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_V_TARGET, velocity);
}

bool TMC429::atTargetVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return true;
  }
  int16_t actual_velocity = getActualVelocity(motor);
  int16_t target_velocity = getTargetVelocity(motor);
  return (actual_velocity == target_velocity);
}

int32_t TMC429::getActualVelocityInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertVelocityToHz(pulse_div_[motor], getActualVelocity(motor));
}

int16_t TMC429::getActualVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t velocity_unsigned = readRegister(motor, ADDRESS_V_ACTUAL);
  return unsignedToSigned(velocity_unsigned, V_BIT_COUNT);
}

void TMC429::setHoldVelocityMaxInHz(size_t motor,
  uint32_t velocity_max_hz)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  setOptimalStepDivHz(velocity_max_hz);

  setOptimalPulseDivHz(motor, velocity_max_hz);

  setVelocityMaxInHz(motor, velocity_max_hz);
}

void TMC429::setHoldVelocityInHz(size_t motor,
  int32_t velocity_hz)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setHoldVelocity(motor, convertVelocityFromHz(pulse_div_[motor], velocity_hz));
}

void TMC429::setHoldVelocity(size_t motor,
  int16_t velocity)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_V_ACTUAL, velocity);
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div;
  if (pulse_div > 0)
  {
    ramp_div = pulse_div - 1;
  }
  else
  {
    ramp_div = RAMP_DIV_MIN;
  }
  return getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
}

uint32_t TMC429::getAccelerationMaxLowerLimitInHzPerS(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = RAMP_DIV_MAX;
  return getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
}

uint32_t TMC429::getAccelerationMaxInHzPerS(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertAccelerationToHzPerS(pulse_div_[motor], ramp_div_[motor], getAccelerationMax(motor));
}

uint32_t TMC429::getActualAccelerationInHzPerS(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  return convertAccelerationToHzPerS(pulse_div_[motor], ramp_div_[motor], getActualAcceleration(motor));
}

int32_t TMC429::getTargetPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_TARGET);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setTargetPosition(size_t motor,
  int32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_X_TARGET, position);
}

bool TMC429::atTargetPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return true;
  }
  int32_t actual_position = getActualPosition(motor);
  int32_t target_position = getTargetPosition(motor);
  return (actual_position == target_position);
}

int32_t TMC429::getActualPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_ACTUAL);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setActualPosition(size_t motor,
  int32_t position)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_X_ACTUAL, position);
}

void TMC429::stop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  setMode(motor, VELOCITY_MODE);
  setTargetVelocity(motor, 0);
}

void TMC429::stopAll()
{
  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    stop(motor);
  }
}

void TMC429::enableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::enableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::setSwitchesActiveLow()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::setSwitchesActiveHigh()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::enableLeftSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableLeftSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::leftSwitchStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_l;
}

bool TMC429::leftSwitchActive(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  SwitchState switch_state = getSwitchState();
  switch (motor)
  {
    case 0:
    {
      return switch_state.l0;
      break;
    }
    case 1:
    {
      return switch_state.l1;
      break;
    }
    case 2:
    {
      return switch_state.l2;
      break;
    }
  }
  return false;
}

void TMC429::enableRightSwitches()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableRightSwitches()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

bool TMC429::rightSwitchesEnabled()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf.en_refr;
}

void TMC429::enableRightSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableRightSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::rightSwitchStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_r;
}

bool TMC429::rightSwitchActive(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  SwitchState switch_state = getSwitchState();
  switch (motor)
  {
    case 0:
    {
      return switch_state.r0;
      break;
    }
    case 1:
    {
      return switch_state.r1;
      break;
    }
    case 2:
    {
      return switch_state.r2;
      break;
    }
  }
  return false;
}

void TMC429::enableSwitchSoftStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableSwitchSoftStop(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::switchSoftStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return false;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return ref_conf_mode.ref_conf.soft_stop;
}

void TMC429::setReferenceSwitchToLeft(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::setReferenceSwitchToRight(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::startLatchPositionWaiting(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  writeRegister(motor, ADDRESS_X_LATCHED, 0);
}

bool TMC429::latchPositionWaiting(size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.lp = false;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.lp;
}

int32_t TMC429::getLatchPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0;
  }
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_LATCHED);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setPositionCompareMotor(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.pos_comp_sel = motor;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

TMC429::Status TMC429::getStatus()
{
  getVersion();
  return status_;
}

// private
void TMC429::initialize()
{
  setStepDirOutput();
}

void TMC429::setStepDirOutput()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_sd = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

// void TMC429::setSpiOutput()
// {
//   IfConf if_conf;
//   if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
//   if_conf.if_conf.en_sd = 0;
//   writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
// }

uint32_t TMC429::readRegister(uint8_t smda,
  uint8_t address)
{ 
  // std::cout << "TMC429 - Reading Addr: " << std::bitset<8>(address) << std::endl;
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_READ;
  mosi_datagram.data = 0;
  // std::cout << "TMC429 - Sending  Datagram: " << std::bitset<32>(mosi_datagram.bytes) << std::endl;
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  // std::cout << "TMC429 - Received Datagram: " << std::bitset<32>(miso_datagram.bytes) << std::endl << std::endl;
  
  return miso_datagram.data;
}

void TMC429::writeRegister(uint8_t smda,
  uint8_t address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.data = data;
  // std::cout << "TMC429 - Sending  Datagram: " << std::bitset<32>(mosi_datagram.bytes) << std::endl << std::endl;
  writeRead(mosi_datagram);
}

TMC429::MisoDatagram TMC429::writeRead(MosiDatagram mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;
  // beginTransaction();
  digitalWrite(chip_select_pin_, LOW);
  // usleep(1);
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    // char byte_write[1];
    // char byte_read[1];
    // byte_write[0] = (mosi_datagram.bytes >> (8*i)) & 0xff;
    // uint8_t spi_status = spiXfer(spi_handle, byte_write, byte_read, 1);
    // miso_datagram.bytes |= ((uint32_t)byte_read[0]) << (8*i);
    
    unsigned char byte_transfer[1];
    byte_transfer[0] = (mosi_datagram.bytes >> (8*i)) & 0xff;
    // wiringPiSPIDataRW(spi_handle, byte_transfer, 1);
    wiringPiSPIDataRW(spi_channel, byte_transfer, 1);
    miso_datagram.bytes |= ((uint32_t)byte_transfer[0]) << (8*i);

  }
  // endTransaction();
  // usleep(1);
  digitalWrite(chip_select_pin_, HIGH);
  status_ = miso_datagram.status;
  return miso_datagram;
}

int32_t TMC429::unsignedToSigned(uint32_t input_value,
  uint8_t num_bits)
{
  uint32_t mask = 1 << (num_bits - 1);
  return -(input_value & mask) + (input_value & ~mask);
}

void TMC429::specifyClockFrequencyInMHz(uint8_t clock_frequency)
{
  if (clock_frequency <= CLOCK_FREQUENCY_MAX)
  {
    clock_frequency_ = clock_frequency;
  }
  else
  {
    clock_frequency_ = CLOCK_FREQUENCY_MAX;
  }
}

void TMC429::setOptimalStepDivHz(uint32_t velocity_max_hz)
{
  int step_div = getStepDiv();

  double step_time = stepDivToStepTime(step_div);

  uint32_t velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);

  while ((velocity_max_upper_limit < velocity_max_hz) && (step_div >= 1))
  {
    --step_div;
    step_time = stepDivToStepTime(step_div);
    velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);
  }

  setStepDiv(step_div);
}

uint8_t TMC429::getStepDiv()
{
  GlobalParameters global_parameters;
  global_parameters.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  return global_parameters.clk2_div & STEP_DIV_MASK;
}

void TMC429::setStepDiv(uint8_t step_div)
{
  GlobalParameters global_parameters;
  global_parameters.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  global_parameters.clk2_div = step_div & STEP_DIV_MASK;
  writeRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS, global_parameters.bytes);
}

double TMC429::stepDivToStepTime(uint8_t step_div)
{
  double step_time = (double)(16*(1 + step_div))/(double)clock_frequency_;
  return step_time;
}

int32_t TMC429::convertVelocityToHz(uint8_t pulse_div,
  int16_t velocity)
{
  // (clock_frequency_*MHZ_PER_HZ*velocity)/((1 << pulse_div)*VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = (x*(double)velocity)/((double)(1 << pulse_div));
  return y;
}

int16_t TMC429::convertVelocityFromHz(uint8_t pulse_div,
  int32_t velocity)
{
  // (velocity*(1 << pulse_div)*VELOCITY_CONSTANT)/(clock_frequency_*MHZ_PER_HZ);
  double x = ((double)velocity*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double y = x*(double)VELOCITY_CONSTANT;
  return y;
}

uint8_t TMC429::findOptimalPulseDivHz(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = PULSE_DIV_MAX + 1;
  uint32_t velocity_max_upper_limit = 0;
  while ((velocity_max_upper_limit < velocity_max_hz) && (pulse_div >= 1))
  {
    --pulse_div;
    velocity_max_upper_limit = getVelocityMaxUpperLimitInHz(pulse_div);
  }
  return pulse_div;
}

void TMC429::setOptimalPulseDivHz(size_t motor,
  uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  ClkConfig clk_config;
  clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.pulse_div = pulse_div;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  pulse_div_[motor] = pulse_div;
}

TMC429::Mode TMC429::getMode(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return RAMP_MODE;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  switch (ref_conf_mode.mode)
  {
    case RAMP_MODE:
      return RAMP_MODE;
      break;
    case SOFT_MODE:
      return SOFT_MODE;
      break;
    case VELOCITY_MODE:
      return VELOCITY_MODE;
      break;
    case HOLD_MODE:
      return HOLD_MODE;
      break;
  }
  return RAMP_MODE;
}

void TMC429::setMode(size_t motor,
  Mode mode)
{
  if (motor >= MOTOR_COUNT)
  {
    std::cout << "MOTOR_INDEX bigger than MOTOR_COUNT" << std::endl;
    return;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.mode = (uint8_t)mode;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

TMC429::ReferenceConfiguration TMC429::getReferenceConfiguration(size_t motor)
{
  RefConfMode ref_conf_mode;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.ref_conf;
}

TMC429::InterfaceConfiguration TMC429::getInterfaceConfiguration()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf;
}

TMC429::SwitchState TMC429::getSwitchState()
{
  SwState switch_state;
  switch_state.bytes = readRegister(SMDA_COMMON, ADDRESS_SWITCHES);
  return switch_state.switch_state;
}

TMC429::ClockConfiguration TMC429::getClockConfiguration(size_t motor)
{
  ClkConfig clk_config;
  if (motor < MOTOR_COUNT)
  {
    clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  }
  return clk_config.clk_config;
}

double TMC429::getProportionalityFactor(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0.0;
  }
  PropFactor prop_factor;
  prop_factor.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  int pm = prop_factor.pmul;
  int pd = prop_factor.pdiv;
  return ((double)(pm)) / ((double)(1 << (pd + 3)));
}

double TMC429::getStepTimeInMicroS()
{
  uint8_t step_div = getStepDiv();
  return stepDivToStepTime(step_div);
}

uint16_t TMC429::getVelocityMin(size_t motor)
{
  return readRegister(motor, ADDRESS_V_MIN);
}

void TMC429::setVelocityMinInHz(size_t motor,
  uint32_t velocity_min_hz)
{
  uint32_t velocity_min = convertVelocityFromHz(pulse_div_[motor], velocity_min_hz);
  if (velocity_min < VELOCITY_MIN_MIN)
  {
    velocity_min = VELOCITY_MIN_MIN;
  }
  writeRegister(motor, ADDRESS_V_MIN, velocity_min);
}

void TMC429::setVelocityMin(size_t motor,
  uint32_t velocity_min)
{
  writeRegister(motor, ADDRESS_V_MIN, velocity_min);
}

uint16_t TMC429::getVelocityMax(size_t motor)
{
  return readRegister(motor, ADDRESS_V_MAX);
}

void TMC429::setVelocityMaxInHz(size_t motor,
  uint32_t velocity_max_hz)
{
  uint32_t velocity_max = convertVelocityFromHz(pulse_div_[motor], velocity_max_hz);
  uint32_t velocity_max_upper_limit = getVelocityMaxUpperLimitInHz();
  if (velocity_max > velocity_max_upper_limit)
  {
    velocity_max = velocity_max_upper_limit;
  }
  writeRegister(motor, ADDRESS_V_MAX, velocity_max);
}

void TMC429::setVelocityMax(size_t motor, uint32_t velocity_max)
{
  writeRegister(motor, ADDRESS_V_MAX, velocity_max);
}

uint32_t TMC429::convertAccelerationToHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t acceleration)
{
  // (clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ*acceleration)/((1 << pulse_div)*(1 << ramp_div)*ACCELERATION_CONSTANT);
  double a = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)ACCELERATION_CONSTANT;
  double b = a*(double)clock_frequency_*(double)MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div));
  double d = c/((double)(1 << ramp_div));
  uint32_t e = round(d*(double)acceleration);
  return e;
}

uint32_t TMC429::convertAccelerationFromHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t acceleration)
{
  // (acceleration*(1 << pulse_div)*(1 << ramp_div)*ACCELERATION_CONSTANT)/(clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ);
  double a = ((double)acceleration*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double b = a*(double)ACCELERATION_CONSTANT;
  double c = b/((double)clock_frequency_*(double)MHZ_PER_HZ);
  uint32_t d = round(c*(1 << ramp_div));
  return d;
}

uint8_t TMC429::findOptimalRampDivHz(uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = RAMP_DIV_MAX;
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);;
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);

  while ((acceleration_max_upper_limit < acceleration_max_hz_per_s) &&
    (acceleration_max_lower_limit < acceleration_max_hz_per_s) &&
    (ramp_div >= 1) &&
    (ramp_div >= pulse_div))
  {
    --ramp_div;
    acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
    acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
  }
  return ramp_div;
}

void TMC429::setOptimalRampDivHz(size_t motor,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint8_t ramp_div = findOptimalRampDivHz(velocity_max_hz, acceleration_max_hz_per_s);
  ClkConfig clk_config;
  clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.ramp_div = ramp_div;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  ramp_div_[motor] = ramp_div;
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz(uint8_t pulse_div)
{
  return convertVelocityToHz(pulse_div, VELOCITY_REGISTER_MAX);
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div,
  uint8_t ramp_div)
{
  uint32_t a_max_upper_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div + 1) >= 0)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  else if (((int8_t)ramp_div - (int8_t)pulse_div + 12) < 1)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_upper_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div + 12)) - 1;
  }
  if (a_max_upper_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_upper_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_upper_limit);
}

uint32_t TMC429::getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t velocity_max)
{
  uint32_t a_max_lower_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div - 1) <= 0)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_lower_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div - 1));
    if (convertVelocityFromHz(pulse_div, velocity_max) <= (int16_t)VELOCITY_REGISTER_THRESHOLD)
    {
      a_max_lower_limit /= 2;
    }
  }
  if (a_max_lower_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_lower_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_lower_limit);
}

uint32_t TMC429::getAccelerationMax(size_t motor)
{
  return readRegister(motor, ADDRESS_A_MAX);
}

uint32_t TMC429::setAccelerationMaxInHzPerS(size_t motor,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div_[motor], ramp_div_[motor]);
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div_[motor], ramp_div_[motor], velocity_max_hz);
  if (acceleration_max_hz_per_s > acceleration_max_upper_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_upper_limit;
  }
  if (acceleration_max_hz_per_s < acceleration_max_lower_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_lower_limit;
  }
  uint32_t acceleration_max = convertAccelerationFromHzPerS(pulse_div_[motor], ramp_div_[motor], acceleration_max_hz_per_s);
  if (acceleration_max > ACCELERATION_REGISTER_MAX)
  {
    acceleration_max = ACCELERATION_REGISTER_MAX;
  }
  if (acceleration_max < ACCELERATION_REGISTER_MIN)
  {
    acceleration_max = ACCELERATION_REGISTER_MIN;
  }
  writeRegister(motor, ADDRESS_A_MAX, acceleration_max);
  return acceleration_max;
}

void TMC429::setAccelerationMaxInStepPerSS(size_t motor, uint32_t acceleration_max)
{
  writeRegister(motor, ADDRESS_A_MAX, acceleration_max);
}

int16_t TMC429::getActualAcceleration(size_t motor)
{
  uint32_t acceleration_unsigned = readRegister(motor, ADDRESS_A_ACTUAL);
  return unsignedToSigned(acceleration_unsigned, A_BIT_COUNT);
}

void TMC429::setOptimalPropFactor(size_t motor,
  uint32_t acceleration_max)
{
  // int pdiv, pmul, pm, pd ;
  // double p_ideal, p_best, p, p_reduced;

  // pm=-1; pd=-1; // -1 indicates : no valid pair found

  // p_ideal = a_max / (pow(2, ramp_div-pulse_div)*128.0);
  // p = a_max / ( 128.0 * pow(2, ramp_div-pulse_div) );
  // p_reduced = p * ( 1.0 – p_reduction );
  // for (pdiv=0; pdiv<=13; pdiv++)
  // {
  //   pmul = (int)(p_reduced * 8.0 * pow(2, pdiv)) – 128;
  //   if ( (0 <= pmul) && (pmul <= 127) )
  //   {
  //     pm = pmul + 128;
  //     pd = pdiv;
  //   }
  //   *p_mul = pm;
  //   *p_div = pd;
  //   p_best = ((double)(pm)) / ((double)pow(2, pd+3));
  // }
  // *PIdeal = p_ideal;
  // *PBest = p_best;
  // *PRedu = p_reduced;

  int pdiv, pmul, pm, pd ;
  double p_ideal, p_reduced;

  pm=-1; pd=-1; // -1 indicates : no valid pair found
  p_ideal = acceleration_max/(128.0*(1 << (ramp_div_[motor] - pulse_div_[motor])));
  p_reduced = p_ideal*0.99;
  for (pdiv=0; pdiv<=13; ++pdiv)
  {
    pmul = (int)(p_reduced*8.0*(1 << pdiv)) - 128;
    if ((0 <= pmul) && (pmul <= 127))
    {
      pm = pmul + 128;
      pd = pdiv;
    }
  }
  if ((pm == -1) || (pd == -1))
  {
    return;
  }
  PropFactor prop_factor;
  prop_factor.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  prop_factor.pmul = pm;
  prop_factor.pdiv = pd;
  writeRegister(motor, ADDRESS_PROP_FACTOR, prop_factor.bytes);
}

void TMC429::enableChipSelect()
{
  digitalWrite(chip_select_pin_, LOW);
}

void TMC429::disableChipSelect()
{
  digitalWrite(chip_select_pin_, HIGH);
}

void TMC429::beginTransaction()
{
  // std::cout << "TMC429 SPI Handle: " << std::hex << spi_handle << std::endl;
  enableChipSelect();
  usleep(1);
}

void TMC429::endTransaction()
{
  //spiClose(spi_handle);
  
  disableChipSelect();
  // usleep(10);
}