#include "tmc2130.h"


TMC2130::TMC2130(int16_t cs_pin, uint8_t spi_channel, uint32_t spi_speed)
{
    _tmc2130_config.cs_pin = cs_pin;
    _tmc2130_config.spi_channel = spi_channel;
    _tmc2130_config.spi_speed;
}

uint8_t TMC2130::begin()
{
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed" << std::endl;
        return -1;
    }

    gpioSetMode(_tmc2130_config.cs_pin , PI_OUTPUT);  // Set CS pin as output
    gpioWrite(_tmc2130_config.cs_pin , PI_HIGH);      // Set CS pin high (inactive)

    


    _tmc2130_config.initialized = true;
    return 0;
}

uint8_t TMC2130::test_connection() {
	uint32_t drv_status = read_spi(DRV_STATUS);
	switch (drv_status) {
	    case 0xFFFFFFFF: return 1;
	    case 0: return 2;
	    default: return 0;
	}
}



void TMC2130::select_device() 
{
    gpioWrite(_tmc2130_config.cs_pin, PI_LOW);  // Set CS pin low (active)
}

void TMC2130::deselect_device() 
{
    gpioWrite(_tmc2130_config.cs_pin, PI_HIGH);  // Set CS pin high (inactive)
}

uint8_t TMC2130::write_spi(uint8_t addr, uint8_t *data)
{   
    uint8_t ret_val;
    char tx_datagram[5] = {0};
    
    // Set write bit
    tx_datagram[0] |= 0x80;

    //Set address
    tx_datagram[0] |= addr;

    //Transfer data to datagram
    tx_datagram[1] = data[0];
    tx_datagram[2] = data[1];
    tx_datagram[3] = data[2];
    tx_datagram[4] = data[3];

    // Open the SPI channel
    uint8_t spi_handle = spiOpen(_tmc2130_config.spi_channel, _tmc2130_config.spi_speed, 0);  // mode 0
    if (spi_handle < 0) {
        std::cerr << "spiOpen failed" << std::endl;
        gpioTerminate();
        return -1;
    }

    select_device();
    ret_val = spiWrite(spi_handle, tx_datagram, 5);
    if (ret_val < 0) 
    {
        std::cerr << "spiXfer failed. Error: " << ret_val << std::endl;
        deselect_device();
        spiClose(spi_handle);
        gpioTerminate();
        return -1;
    }

    deselect_device();

    spiClose(spi_handle);

    return 0;
}

uint32_t TMC2130::read_spi(uint8_t addr)
{   
    uint8_t ret_val;
    char tx_datagram[5] = {0};
    char rx_datagram[5];

    //Set address
    tx_datagram[0] |= addr;

    // Open the SPI channel
    uint8_t spi_handle = spiOpen(_tmc2130_config.spi_channel, _tmc2130_config.spi_speed, 0);  // mode 0
    if (spi_handle < 0) {
        std::cerr << "spiOpen failed" << std::endl;
        gpioTerminate();
        return -1;
    }

    // Double send empty datagram to read desired address.
    select_device();
    ret_val = spiXfer(spi_handle, tx_datagram, rx_datagram, 5);
    deselect_device();

    select_device();
    ret_val = spiXfer(spi_handle, tx_datagram, rx_datagram, 5);
    if (ret_val < 0) 
    {
        std::cerr << "spiXfer failed. Error: " << ret_val << std::endl;
        deselect_device();
        spiClose(spi_handle);
        gpioTerminate();
        return -1;
    }

    deselect_device();

    spiClose(spi_handle);

    uint32_t register_value = (rx_datagram[1] << 24) | (rx_datagram[2] << 16) | (rx_datagram[3] << 8) | rx_datagram[4];

    return register_value;
}