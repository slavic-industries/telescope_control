/*
 * @file tmc2130.h
 */

#ifndef _TMC2130_H_
#define _TMC2130_H_

#include <pigpio.h>
#include <iostream>

const static uint8_t VERSION = 0x11;


enum REGISTER{
    GCONF = 0x00,
    GSTAT = 0x01,
    IOIN = 0x04,
    IHOLD_IRUN = 0x10,
    TPOWER_DOWN = 0x11,
    TSTEP = 0x12,
    TPWMTHRS = 0x13,
    TCOOLTHRS = 0x14,
    THIGH = 0x15,
    XDIRECT = 0x2D,
    VDCMIN = 0X33,
    MSLUT_0 = 0X60,
    MSLUT_1 = 0X61,
    MSLUT_2 = 0X62,
    MSLUT_3 = 0X63,
    MSLUT_4 = 0X64,
    MSLUT_5 = 0X65,
    MSLUT_6 = 0X66,
    MSLUT_7 = 0X67,
    MSLUTSEL = 0X68,
    MSLUTSTART = 0X69,
    MSCNT = 0X6A,
    MSCURACT = 0x6B,
    CHOPCONF = 0x6C,
    COOLCONF = 0x6D,
    DCCTRL = 0x6E,
    DRV_STATUS = 0x6F,
    PWMCONF = 0x70,
    PWMSCALE = 0x71,
    ENCM_CTRL = 0x72,
    LOST_STEPS = 0x73,
};

enum {
    DRV_STATUS_NO_CONNECTION,
    DRV_STATUS_STANDSTILL,
    DRV_STATUS_OPEN_LOAD_PHASE_B,
    DRV_STATUS_OPEN_LOAD_PHASE_A,
    DRV_STATUS_SHORT_TO_GND_PHASE_B,
    DRV_STATUS_SHORT_TO_GND_PHASE_A,
    DRV_STATUS_OVER_TEMP_PREWARNING,
    DRV_STATUS_OVER_TEMP,
    STALL_DETECTION
};



typedef struct {
    bool initialized;
    uint8_t cs_pin;
    uint8_t spi_channel;
    uint32_t spi_speed;
}tmc2130Config_t;

class TMC2130{
    public:
        TMC2130(int16_t cs_pin, uint8_t spi_channel, uint32_t spi_speed);
        uint8_t begin();
        uint32_t check_drv_status();
        void set_rms_current();

    private:

        tmc2130Config_t _tmc2130_config;

        void select_device();
        void deselect_device();

        uint8_t write_spi(uint8_t addr, uint8_t *data);
        uint32_t read_spi(uint8_t addr);
        


};

#endif
