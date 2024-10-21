// ----------------------------------------------------------------------------
// TMC2130.h
//
// <Original Author:
// Peter Polidoro peter@polidoro.io
// 
// Modified to work with pigpio
// ----------------------------------------------------------------------------
#include "telescope_drive/tmc2130.h"
#include <bitset>

TMC2130::~TMC2130()
{
  if(initialized)
  {
    initialized = false;
  }
}
uint8_t TMC2130::setup(size_t chip_select_pin, uint8_t spi_device)
{
  this->chip_select_pin_ = chip_select_pin;
  this->enable_pin_ = -1;
  this->spi_channel = spi_device;

  if (wiringPiSetup() != 0) {
    return 1;
  }

  pinMode(this->chip_select_pin_, OUTPUT);
  digitalWrite(this->chip_select_pin_, HIGH);

  spi_handle = wiringPiSPISetupMode(this->spi_channel, 500000, 3);
  if (spi_handle == -1) {
    return 2;
  }
  // std::cout << "TMC2130 SPI Handle (setup): " << std::hex << spi_handle << std::endl;


  global_config_.uint32 = 0;

  driver_current_.uint32 = 0;

  chopper_config_.uint32 = 0;
  chopper_config_.fields.toff = TOFF_DEFAULT;
  chopper_config_.fields.hstrt = HSTRT_DEFAULT;
  chopper_config_.fields.hend = HEND_DEFAULT;
  chopper_config_.fields.chm = CHM_DEFAULT;
  chopper_config_.fields.tbl = TBL_DEFAULT;
  microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;

  pwm_config_.uint32 = 0;
  pwm_config_.fields.pwm_ampl = PWM_AMPL_DEFAULT;
  pwm_config_.fields.pwm_grad = PWM_GRAD_DEFAULT;
  pwm_config_.fields.pwm_freq = PWM_FREQ_DEFAULT;
  pwm_config_.fields.pwm_autoscale = PWM_AUTOSCALE_DEFAULT;

//   reset_driver();

  // ChopperConfig chopper_config;
  // uint32_t rd = read(ADDRESS_CHOPCONF);
  // std::cout << "Chopper confg read: " << std::bitset<32>(rd) << std::endl;
  // chopper_config.uint32 = rd;
  // std::cout << "Chopper confg: " << std::bitset<32>(chopper_config.uint32) << std::endl;
  // std::cout << "Chopper toff: " << std::bitset<4>(chopper_config.fields.toff) << std::endl;
  // chopper_config.fields.toff = 8;
  // chopper_config.fields.tbl = 1;
  // std::cout << "Chopper confg: " << std::bitset<32>(chopper_config.uint32) << std::endl;
  // write(ADDRESS_CHOPCONF, chopper_config.uint32);
  // rd = read(ADDRESS_CHOPCONF);
  // std::cout << "Chopper confg read: " << std::bitset<32>(rd) << std::endl;

  // GlobalConfig gc;
  // rd = read(ADDRESS_GCONF);
  // gc.uint32 = rd;
  // std::cout << "General confg read: " << std::bitset<32>(rd) << std::endl;
  // gc.fields.enc_commutation = 0;
  // gc.fields.shaft = 0;
  // gc.fields.diag0_error = 0;
  // gc.fields.diag0_otpw = 0;
  // write(ADDRESS_GCONF, gc.uint32);
  // rd = read(ADDRESS_GCONF);
  // std::cout << "General con mo read: " << std::bitset<32>(rd) << std::endl;

  initialized = true;
  return 0;
}

void TMC2130::printSettings()
{
  GlobalConfig global_config_;
  GlobalStatus global_status_;
  InputPinStatus input_pin_status_;
  DriverCurrent driver_current_;
  PowerDownDelay power_down_;
  ChopperConfig chopper_config_;
  CoolConfig cool_config_;
  DriveStatus drive_status_;
  PwmConfig pwm_cingfig_;


  global_config_.uint32 = read(ADDRESS_GCONF);
  global_status_.uint32 = read(ADDRESS_GSTAT);
  input_pin_status_.uint32 = read(ADDRESS_IOIN);
  driver_current_.uint32 = read(ADDRESS_IHOLD_IRUN);
  power_down_.uint32 = read(ADDRESS_TPOWERDOWN);
  chopper_config_.uint32 = read(ADDRESS_CHOPCONF);
  cool_config_.uint32 = read(ADDRESS_COOLCONF);
  drive_status_.uint32 = read(ADDRESS_DRV_STATUS);
  pwm_config_.uint32 = read(ADDRESS_PWMCONF);
  
  std::cout << "Driver Settings" << std::endl;
  std::cout << "Global Config:   \t" << std::bitset<32>(global_config_.uint32) << std::endl;
  std::cout << "\ti_scale_analog:" << std::bitset<1>(global_config_.fields.i_scale_analog) << std::endl;
  std::cout << "\tinternal_rsense:" << std::bitset<1>(global_config_.fields.internal_rsense) << std::endl;
  std::cout << "\ten_pwm_mode:" << std::bitset<1>(global_config_.fields.en_pwm_mode) << std::endl;
  std::cout << "\tenc_communication:" << std::bitset<1>(global_config_.fields.enc_commutation) << std::endl;
  std::cout << "\tshaft:" << std::bitset<1>(global_config_.fields.shaft) << std::endl;
  std::cout << "\tdiag0_error:" << std::bitset<1>(global_config_.fields.diag0_error) << std::endl;
  // std::cout << "\tdiag0_otpw:" << std::bitset<1>(global_config_.fields.diag0_otpw) << std::endl;
  // std::cout << "\tdiag0_stall:" << std::bitset<1>(global_config_.fields.diag0_stall) << std::endl;
  // std::cout << "\tdiag1_index:" << std::bitset<1>(global_config_.fields.diag1_index) << std::endl;
  // std::cout << "\tdiag1_onstate:" << std::bitset<1>(global_config_.fields.diag1_onstate) << std::endl;
  // std::cout << "\tdiag1_steps_skipped:" << std::bitset<1>(global_config_.fields.diag1_steps_skipped) << std::endl;
  // std::cout << "\tdiag1_int_pushpull:" << std::bitset<1>(global_config_.fields.diag1_int_pushpull) << std::endl;
  // std::cout << "\tdiag1_pushpull:" << std::bitset<1>(global_config_.fields.diag1_pushpull) << std::endl;
  std::cout << "\tsmall_hysteresis:" << std::bitset<1>(global_config_.fields.small_hysteresis) << std::endl;
  std::cout << "\tstop_enable:" << std::bitset<1>(global_config_.fields.stop_enable) << std::endl;
  std::cout << "\tdirect_mode:" << std::bitset<1>(global_config_.fields.direct_mode) << std::endl;
  std::cout << "\ttest_mode:" << std::bitset<1>(global_config_.fields.test_mode) << std::endl;
  
  // std::cout << "Global Status:   \t" << std::bitset<32>(global_status_.uint32) << std::endl;
  // std::cout << "\treset:" << std::bitset<1>(global_status_.fields.reset) << std::endl;
  // std::cout << "\tdrv_err:" << std::bitset<1>(global_status_.fields.drv_err) << std::endl;
  // std::cout << "\tuv_cp:" << std::bitset<1>(global_status_.fields.uv_cp) << std::endl;
  
  // std::cout << "Input Pin Status:\t" << std::bitset<32>(input_pin_status_.uint32) << std::endl;
  // std::cout << "\tstep:" << std::bitset<1>(input_pin_status_.fields.step) << std::endl;
  // std::cout << "\tdir:" << std::bitset<1>(input_pin_status_.fields.dir) << std::endl;
  // std::cout << "\tdcen_cfg4:" << std::bitset<1>(input_pin_status_.fields.dcen_cfg4) << std::endl;
  // std::cout << "\tdcin_cfg5:" << std::bitset<1>(input_pin_status_.fields.dcin_cfg5) << std::endl;
  // std::cout << "\tdrv_enn_cfg6:" << std::bitset<1>(input_pin_status_.fields.drv_enn_cfg6) << std::endl;
  // std::cout << "\tdco:" << std::bitset<1>(input_pin_status_.fields.dco) << std::endl;
  // std::cout << "\tone:" << std::bitset<1>(input_pin_status_.fields.one) << std::endl;
  // std::cout << "\tversion:" << std::bitset<8>(input_pin_status_.fields.version) << std::endl;
  
  std::cout << "Driver Current:  \t" << std::bitset<32>(driver_current_.uint32) << std::endl;
  std::cout << "\tihold:" << std::bitset<5>(driver_current_.fields.ihold) << std::endl;
  std::cout << "\tirun:" << std::bitset<5>(driver_current_.fields.irun) << std::endl;
  std::cout << "\tiholddelay:" << std::bitset<4>(driver_current_.fields.iholddelay) << std::endl;
  
  // std::cout << "Power Down:      \t" << std::bitset<32>(power_down_.uint32) << std::endl;
  // std::cout << "\tvalue:" << std::bitset<8>(power_down_.fields.value) << std::endl;
  
  std::cout << "Chopper Config:  \t" << std::bitset<32>(chopper_config_.uint32) << std::endl;
  std::cout << "\ttoff:" << std::bitset<4>(chopper_config_.fields.toff) << std::endl;
  std::cout << "\thstrt:" << std::bitset<3>(chopper_config_.fields.hstrt) << std::endl;
  std::cout << "\thend:" << std::bitset<4>(chopper_config_.fields.hend) << std::endl;
  std::cout << "\tfd3:" << std::bitset<1>(chopper_config_.fields.fd3) << std::endl;
  std::cout << "\tdisfdcc:" << std::bitset<1>(chopper_config_.fields.disfdcc) << std::endl;
  std::cout << "\trndtf:" << std::bitset<1>(chopper_config_.fields.rndtf) << std::endl;
  std::cout << "\tchm:" << std::bitset<1>(chopper_config_.fields.chm) << std::endl;
  std::cout << "\ttbl:" << std::bitset<2>(chopper_config_.fields.tbl) << std::endl;
  std::cout << "\tvsense:" << std::bitset<1>(chopper_config_.fields.vsense) << std::endl;
  std::cout << "\tvhighfs:" << std::bitset<1>(chopper_config_.fields.vhighfs) << std::endl;
  std::cout << "\tvhighchm:" << std::bitset<1>(chopper_config_.fields.vhighchm) << std::endl;
  std::cout << "\tsync:" << std::bitset<4>(chopper_config_.fields.sync) << std::endl;
  std::cout << "\tmres:" << std::bitset<4>(chopper_config_.fields.mres) << std::endl;
  std::cout << "\tintpol:" << std::bitset<1>(chopper_config_.fields.intpol) << std::endl;
  std::cout << "\tdedge:" << std::bitset<1>(chopper_config_.fields.dedge) << std::endl;
  std::cout << "\tdiss2g:" << std::bitset<1>(chopper_config_.fields.diss2g) << std::endl;
  
  std::cout << "Cool Config:     \t" << std::bitset<32>(cool_config_.uint32) << std::endl;
  std::cout << "\tsemin:" << std::bitset<4>(cool_config_.fields.semin) << std::endl;
  std::cout << "\tseup:" << std::bitset<2>(cool_config_.fields.seup) << std::endl;
  std::cout << "\tsemax:" << std::bitset<4>(cool_config_.fields.semax) << std::endl;
  std::cout << "\tsedn:" << std::bitset<2>(cool_config_.fields.sedn) << std::endl;
  std::cout << "\tseimin:" << std::bitset<1>(cool_config_.fields.seimin) << std::endl;
  std::cout << "\tsgt:" << std::bitset<7>(cool_config_.fields.sgt) << std::endl;
  std::cout << "\tsfilt:" << std::bitset<1>(cool_config_.fields.sfilt) << std::endl;
  
  std::cout << "Drive Status:    \t" << std::bitset<32>(drive_status_.uint32) << std::endl;
  std::cout << "\tload:" << std::bitset<1>(drive_status_.fields.status.load) << std::endl;
  std::cout << "\tfull_step_active:" << std::bitset<1>(drive_status_.fields.status.full_step_active) << std::endl;
  std::cout << "\tcurrent_scaling:" << std::bitset<1>(drive_status_.fields.status.current_scaling) << std::endl;
  std::cout << "\tstall:" << std::bitset<1>(drive_status_.fields.status.stall) << std::endl;
  std::cout << "\tover_temperature_shutdown:" << std::bitset<1>(drive_status_.fields.status.over_temperature_shutdown) << std::endl;
  std::cout << "\tover_temperature_warning:" << std::bitset<1>(drive_status_.fields.status.over_temperature_warning) << std::endl;
  std::cout << "\tshort_to_ground_a:" << std::bitset<1>(drive_status_.fields.status.short_to_ground_a) << std::endl;
  std::cout << "\tshort_to_ground_b:" << std::bitset<1>(drive_status_.fields.status.short_to_ground_b) << std::endl;
  std::cout << "\topen_load_a:" << std::bitset<1>(drive_status_.fields.status.open_load_a) << std::endl;
  std::cout << "\topen_load_b:" << std::bitset<1>(drive_status_.fields.status.open_load_b) << std::endl;
  std::cout << "\tstandstill:" << std::bitset<1>(drive_status_.fields.status.standstill) << std::endl;
  
  // std::cout << "PWM Config:      \t" << std::bitset<32>(pwm_cingfig_.uint32) << std::endl;
  // std::cout << "\tpwm_ampl:" << std::bitset<8>(pwm_cingfig_.fields.pwm_ampl) << std::endl;
  // std::cout << "\tpwm_grad:" << std::bitset<8>(pwm_cingfig_.fields.pwm_grad) << std::endl;
  // std::cout << "\t:pwm_freq" << std::bitset<2>(pwm_cingfig_.fields.pwm_freq) << std::endl;
  // std::cout << "\tpwm_autoscale:" << std::bitset<1>(pwm_cingfig_.fields.pwm_autoscale) << std::endl;
  // std::cout << "\tpwm_symmetric:" << std::bitset<1>(pwm_cingfig_.fields.pwm_symmetric) << std::endl;
  // std::cout << "\tfreewheel:" << std::bitset<2>(pwm_cingfig_.fields.freewheel) << std::endl;
  
}

uint8_t TMC2130::setup(size_t chip_select_pin,
  size_t enable_pin, uint8_t spi_device)
{
  if(!setup(chip_select_pin, spi_device)) return -1;
  setEnablePin(enable_pin);
  return 0;
}

bool TMC2130::communicating()
{
  return (getVersion() == VERSION);
}

uint8_t TMC2130::getVersion()
{
  uint32_t data = read(ADDRESS_IOIN);

  InputPinStatus input_pin_status;
  input_pin_status.uint32 = data;

  // std::cout << "TMC2130 - Version: " << std::bitset<8>(input_pin_status.fields.version)<< std::endl;

  return input_pin_status.fields.version;
}


void TMC2130::reset_driver()
{
  write(ADDRESS_GCONF, 0);
  write(ADDRESS_IHOLD_IRUN, 0x00000000);
  write(ADDRESS_TPOWERDOWN, 0x00000000);
  write(ADDRESS_TPWMTHRS, 0x00000000);
  write(ADDRESS_TCOOLTHRS, 0x00000000);
  write(ADDRESS_THIGH, 0x00000000);
  write(ADDRESS_XDIRECT, 0x00000000);
  write(ADDRESS_VDCMIN, 0x00000000);
  write(ADDRESS_CHOPCONF, 0x00000000);
  write(ADDRESS_COOLCONF, 0x00000000);
  write(ADDRESS_PWMCONF, 0x00000000);
  write(ADDRESS_ENCM_CTRL, 0x00000000);
}

void TMC2130::setup_driver()
{
  setGlobalConfig();
  setDriverCurrent();
  setChopperConfig();
  setPwmConfig();

  setMicrostepsPerStep(32);
  enableStealthChop();
  // tmc2130.enableAutomaticCurrentScaling();
  setPwmThreshold(TPWMTHRS_DEFAULT);
}




void TMC2130::initialize()
{
  // setMicrostepsPerStep(256);
  // enableStealthChop();
  setPwmThreshold(TPWMTHRS_DEFAULT);
  setPwmConfig();
}

void TMC2130::enable()
{
  if (enable_pin_ >= 0)
  {
    digitalWrite(enable_pin_, LOW);
  }
}

void TMC2130::disable()
{
  if (enable_pin_ >= 0)
  {
    digitalWrite(enable_pin_, HIGH);
  }
}

void TMC2130::setMicrostepsPerStep(size_t microsteps_per_step)
{
  size_t microsteps_per_step_shifted = constrain(microsteps_per_step,
    MICROSTEPS_PER_STEP_MIN,
    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step >> 1;
  size_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  setMicrostepsPerStepPowerOfTwo(exponent);
}

size_t TMC2130::getMicrostepsPerStep()
{
  return 1 << microsteps_per_step_exponent_;
}

void TMC2130::setRunCurrent(uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);

  driver_current_.fields.irun = run_current;
  setDriverCurrent();
}

void TMC2130::setHoldCurrent(uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);

  driver_current_.fields.ihold = hold_current;
  setDriverCurrent();
}

void TMC2130::setHoldDelay(uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);

  driver_current_.fields.iholddelay = hold_delay;
  setDriverCurrent();
}

void TMC2130::setAllCurrentValues(uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  driver_current_.fields.irun = run_current;
  driver_current_.fields.ihold = hold_current;
  driver_current_.fields.iholddelay = hold_delay;
  setDriverCurrent();
}

TMC2130::Status TMC2130::getStatus()
{
  DriveStatus drive_status;
  drive_status.uint32 = read(ADDRESS_DRV_STATUS);
  return drive_status.fields.status;
}

void TMC2130::enableAnalogInputCurrentScaling()
{
  global_config_.fields.i_scale_analog = 1;
  setGlobalConfig();
}

void TMC2130::disableAnalogInputCurrentScaling()
{
  global_config_.fields.i_scale_analog = 0;
  setGlobalConfig();
}

void TMC2130::enableInverseMotorDirection()
{
  global_config_.fields.shaft = 1;
  setGlobalConfig();
}

void TMC2130::disableInverseMotorDirection()
{
  global_config_.fields.shaft = 0;
  setGlobalConfig();
}

void TMC2130::enableStealthChop()
{
  global_config_.fields.en_pwm_mode = 1;
  setGlobalConfig();
}

void TMC2130::disableStealthChop()
{
  global_config_.fields.en_pwm_mode = 0;
  setGlobalConfig();
}

void TMC2130::enableAutomaticCurrentScaling()
{
  pwm_config_.fields.pwm_autoscale = PWM_AUTOSCALE_ENABLED;
  pwm_config_.fields.pwm_ampl = PWM_AMPL_DEFAULT;
  pwm_config_.fields.pwm_grad = PWM_GRAD_DEFAULT;
  setPwmConfig();
}

void TMC2130::disableAutomaticCurrentScaling()
{
  pwm_config_.fields.pwm_autoscale = PWM_AUTOSCALE_DISABLED;
  pwm_config_.fields.pwm_ampl = PWM_AMPL_MIN;
  pwm_config_.fields.pwm_grad = PWM_GRAD_MIN;
  setPwmConfig();
}

void TMC2130::setZeroHoldCurrentMode(TMC2130::ZeroHoldCurrentMode mode)
{
  pwm_config_.fields.freewheel = mode;
  setPwmConfig();
}

void TMC2130::setPwmOffset(uint8_t pwm_amplitude)
{
  uint8_t pwm_ampl = pwmAmplitudeToPwmAmpl(pwm_amplitude);
  pwm_config_.fields.pwm_ampl = pwm_ampl;
  setPwmConfig();
}

void TMC2130::setPwmGradient(uint8_t pwm_amplitude)
{
  uint8_t pwm_grad = pwmAmplitudeToPwmGrad(pwm_amplitude);
  pwm_config_.fields.pwm_grad = pwm_grad;
  setPwmConfig();
}

uint8_t TMC2130::getPwmScale()
{
  return read(ADDRESS_PWM_SCALE);
}

TMC2130::Settings TMC2130::getSettings()
{
  Settings settings;
  settings.stealth_chop_enabled = global_config_.fields.en_pwm_mode;
  settings.automatic_current_scaling_enabled = pwm_config_.fields.pwm_autoscale;
  settings.zero_hold_current_mode = pwm_config_.fields.freewheel;
  settings.pwm_offset = pwm_config_.fields.pwm_ampl;
  settings.pwm_gradient = pwm_config_.fields.pwm_grad;
  settings.irun = driver_current_.fields.irun;
  settings.ihold = driver_current_.fields.ihold;
  settings.iholddelay = driver_current_.fields.iholddelay;

  return settings;
}

// private
void TMC2130::setEnablePin(size_t enable_pin)
{
  enable_pin_ = enable_pin;

  pinMode(enable_pin_, OUTPUT);
  disable();
}

// void TMC2130::setStepDirInput()
// {
// }

// void TMC2130::setSpiInput()
// {
// }

void TMC2130::setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
{
  microsteps_per_step_exponent_ = exponent;

  switch (exponent)
  {
    case 0:
    {
      chopper_config_.fields.mres = MRES_001;
      break;
    }
    case 1:
    {
      chopper_config_.fields.mres = MRES_002;
      break;
    }
    case 2:
    {
      chopper_config_.fields.mres = MRES_004;
      break;
    }
    case 3:
    {
      chopper_config_.fields.mres = MRES_008;
      break;
    }
    case 4:
    {
      chopper_config_.fields.mres = MRES_016;
      break;
    }
    case 5:
    {
      chopper_config_.fields.mres = MRES_032;
      break;
    }
    case 6:
    {
      chopper_config_.fields.mres = MRES_064;
      break;
    }
    case 7:
    {
      chopper_config_.fields.mres = MRES_128;
      break;
    }
    case 8:
    default:
    {
      microsteps_per_step_exponent_ = MICROSTEPS_PER_STEP_EXPONENT_MAX;
      chopper_config_.fields.mres = MRES_256;
      break;
    }
  }
  setChopperConfig();
}

uint8_t TMC2130::percentToCurrentSetting(uint8_t percent)
{
  uint8_t current_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map(current_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t TMC2130::percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t hold_delay_percent = constrain(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t hold_delay = map(hold_delay_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX);
  return hold_delay;
}

uint8_t TMC2130::pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude)
{
  uint8_t pwm_ampl = pwm_amplitude;
  if (pwm_config_.fields.pwm_autoscale)
  {
    pwm_ampl = constrain(pwm_ampl,
      PWM_AMPL_AUTOSCALE_MIN,
      PWM_AMPL_AUTOSCALE_MAX);
  }
  return pwm_ampl;
}

uint8_t TMC2130::pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude)
{
  uint8_t pwm_grad = pwm_amplitude;
  if (pwm_config_.fields.pwm_autoscale)
  {
    pwm_grad = constrain(pwm_grad,
      PWM_GRAD_AUTOSCALE_MIN,
      PWM_GRAD_AUTOSCALE_MAX);
  }
  return pwm_grad;
}

void TMC2130::setGlobalConfig()
{
  write(ADDRESS_GCONF,global_config_.uint32);
}

void TMC2130::setDriverCurrent()
{
  write(ADDRESS_IHOLD_IRUN,driver_current_.uint32);
}

void TMC2130::setChopperConfig()
{
  write(ADDRESS_CHOPCONF,chopper_config_.uint32);
}

void TMC2130::setPwmThreshold(uint32_t value)
{
  write(ADDRESS_TPWMTHRS,value);
}

void TMC2130::setPwmConfig()
{
  write(ADDRESS_PWMCONF,pwm_config_.uint32);
}

void TMC2130::enableClockSelect()
{
  digitalWrite(chip_select_pin_, LOW);
}

void TMC2130::disableClockSelect()
{
  digitalWrite(chip_select_pin_, HIGH);
}
void TMC2130::spiBeginTransaction()
{
  // std::cout << "TMC2130 SPI Handle: " << std::hex << spi_handle << std::endl;
  enableClockSelect();
}

void TMC2130::spiEndTransaction()
{
  disableClockSelect();
}

template <typename T>
T TMC2130::constrain(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

template <typename T>
T TMC2130::map(T value, T fromLow, T fromHigh, T toLow, T toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

uint32_t TMC2130::write(uint8_t address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint64 = 0;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.address = address;
  mosi_datagram.fields.data = data;
  // std::cout << "TMC2130 - Sending  Datagram: " << std::bitset<32>(mosi_datagram.uint64) << std::endl << std::endl;

  return sendReceivePrevious(mosi_datagram);
}

uint32_t TMC2130::read(uint8_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint64 = 0;
  mosi_datagram.fields.rw = RW_READ;
  mosi_datagram.fields.address = address;

  // must read twice to get value at address
  // std::cout << "TMC2130 - Sending  Datagram : " << std::bitset<40>(mosi_datagram.uint64) << std::endl;
  // std::cout << "TMC2130 - Address           : " << std::bitset<7>(mosi_datagram.fields.address) << std::endl;
  // std::cout << "TMC2130 - RW                : " << std::bitset<1>(mosi_datagram.fields.rw) << std::endl;
  // std::cout << "TMC2130 - Data              : " << std::bitset<32>(mosi_datagram.fields.data) << std::endl;
  //  sendReceivePrevious(mosi_datagram);
  uint32_t data = sendReceivePrevious(mosi_datagram);
  data = sendReceivePrevious(mosi_datagram);

  return data;
}

uint32_t TMC2130::sendReceivePrevious(TMC2130::MosiDatagram & mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.uint64 = 0;

  // std::cout << "TMC2130 - Sending  Datagram : " << std::bitset<40>(mosi_datagram.uint64) << std::endl;

  digitalWrite(chip_select_pin_, LOW);
  usleep(1);
  // for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
//   {
//     unsigned char byte_transfer[1];
//     byte_transfer[0] = (mosi_datagram.uint64 >> (8*i)) & 0xff;
//     // std::cout << "TMC2130 - Sent  Byte: " << std::bitset<8>(byte_transfer[0]) << std::endl;
//     wiringPiSPIDataRW(spi_channel, byte_transfer, 1);
//     // std::cout << "TMC2130 - Received  Byte: " << std::bitset<8>(byte_transfer[0]) << std::endl;
// // 
//     miso_datagram.uint64 |= byte_transfer[0] << (8*i);
//   }

  unsigned char byte_transfer[5];
  byte_transfer[0] = (mosi_datagram.uint64 >> (8*4)) & 0xff;
  byte_transfer[1] = (mosi_datagram.uint64 >> (8*3)) & 0xff;
  byte_transfer[2] = (mosi_datagram.uint64 >> (8*2)) & 0xff;
  byte_transfer[3] = (mosi_datagram.uint64 >> (8*1)) & 0xff;
  byte_transfer[4] = (mosi_datagram.uint64 ) & 0xff;

  // std::cout << "TMC2130 - Sent Byte: " << std::bitset<8>(byte_transfer[0])<< std::endl;
  // std::cout << "TMC2130 - Sent Byte: " << std::bitset<8>(byte_transfer[1])<< std::endl;
  // std::cout << "TMC2130 - Sent Byte: " << std::bitset<8>(byte_transfer[2])<< std::endl;
  // std::cout << "TMC2130 - Sent Byte: " << std::bitset<8>(byte_transfer[3])<< std::endl;
  // std::cout << "TMC2130 - Sent Byte: " << std::bitset<8>(byte_transfer[4])<< std::endl;

  wiringPiSPIDataRW(spi_channel, byte_transfer, 5);

  miso_datagram.uint64 |= byte_transfer[0] << (8*4);
  miso_datagram.uint64 |= byte_transfer[1] << (8*3);
  miso_datagram.uint64 |= byte_transfer[2] << (8*2);
  miso_datagram.uint64 |= byte_transfer[3] << (8*1);
  miso_datagram.uint64 |= byte_transfer[4] << (8*0);


  usleep(1);
  digitalWrite(chip_select_pin_, HIGH);

  spi_status_ = miso_datagram.fields.spi_status;

  // std::cout << "TMC2130 - Received  Datagram: " << std::bitset<40>(miso_datagram.uint64)<< std::endl;

  return miso_datagram.fields.data;
}
