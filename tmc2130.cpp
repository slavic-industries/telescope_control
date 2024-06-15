#include "tmc2130.h"
#include <bitset>


TMC2130::TMC2130(int16_t cs_pin, uint8_t spi_channel, uint32_t spi_speed)
{   
    std::cout << "TMC2130 Configuration Start"<< std::endl;

    _tmc2130_config.cs_pin = cs_pin;
    _tmc2130_config.spi_channel = spi_channel;
    _tmc2130_config.spi_speed = spi_speed;

    
    std::cout << "TMC2130 CS Pin: " << _tmc2130_config.cs_pin << std::endl;
    std::cout << "TMC2130 SPI Channel: " << _tmc2130_config.spi_channel << std::endl;
    std::cout << "TMC2130 SPI Speed: " << _tmc2130_config.spi_speed << std::endl;

    std::cout << "TMC2130 Configuration End"<< std::endl;
    
}

uint8_t TMC2130::begin()
{   
    std::cout << "TMC2130 Initialization Start"<< std::endl;
    
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed" << std::endl;
        return -1;
    }

    gpioSetMode(_tmc2130_config.cs_pin , PI_OUTPUT);  // Set CS pin as output
    gpioWrite(_tmc2130_config.cs_pin , PI_HIGH);      // Set CS pin high (inactive)

    _tmc2130_config.initialized = true;

    uint8_t status = check_drv_status();
    if(status == DRV_STATUS_STANDSTILL)
    {
        std::cerr << "TMC2130 Error: " << status << std::endl;
        return -1;
    }
    
    std::cout << "TMC2130 Initialization End"<< std::endl;
    return 0;
}

uint32_t TMC2130::check_drv_status() {
	uint32_t drv_status = read_spi(DRV_STATUS);	
    switch (drv_status) 
    {
	    case 0x00000000: return DRV_STATUS_NO_CONNECTION;
        case 0x80000000: return DRV_STATUS_STANDSTILL;
        case 0x40000000: return DRV_STATUS_OPEN_LOAD_PHASE_B;
        case 0x20000000: return DRV_STATUS_OPEN_LOAD_PHASE_A;
        case 0x10000000: return DRV_STATUS_SHORT_TO_GND_PHASE_B;
        case 0x08000000: return DRV_STATUS_SHORT_TO_GND_PHASE_A;
        case 0x04000000: return DRV_STATUS_OVER_TEMP_PREWARNING;
        case 0x02000000: return DRV_STATUS_OVER_TEMP;
        case 0x01000000: return STALL_DETECTION;
        
	    default: return drv_status;
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

    // std::cout << "TX Byte 0: "<< std::bitset<8>(tx_datagram[0]) << std::endl;
    // std::cout << "TX Byte 1: "<< std::bitset<8>(tx_datagram[1]) << std::endl;
    // std::cout << "TX Byte 2: "<< std::bitset<8>(tx_datagram[2]) << std::endl;
    // std::cout << "TX Byte 3: "<< std::bitset<8>(tx_datagram[3]) << std::endl;
    // std::cout << "TX Byte 4: "<< std::bitset<8>(tx_datagram[4]) << std::endl;

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
    int spi_handle = spiOpen(unsigned(_tmc2130_config.spi_channel), 
                             unsigned(_tmc2130_config.spi_speed), 
                             0);
    if (spi_handle < 0) {
        std::cerr << "spiOpen failed. Error: " << spi_handle << std::endl;
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

    // std::cout << "TX Byte 0: "<< std::bitset<8>(tx_datagram[0]) << std::endl;
    // std::cout << "TX Byte 1: "<< std::bitset<8>(tx_datagram[1]) << std::endl;
    // std::cout << "TX Byte 2: "<< std::bitset<8>(tx_datagram[2]) << std::endl;
    // std::cout << "TX Byte 3: "<< std::bitset<8>(tx_datagram[3]) << std::endl;
    // std::cout << "TX Byte 4: "<< std::bitset<8>(tx_datagram[4]) << std::endl;

    // std::cout << "RX Byte 0: "<< std::bitset<8>(rx_datagram[0]) << std::endl;
    // std::cout << "RX Byte 1: "<< std::bitset<8>(rx_datagram[1]) << std::endl;
    // std::cout << "RX Byte 2: "<< std::bitset<8>(rx_datagram[2]) << std::endl;
    // std::cout << "RX Byte 3: "<< std::bitset<8>(rx_datagram[3]) << std::endl;
    // std::cout << "RX Byte 4: "<< std::bitset<8>(rx_datagram[4]) << std::endl;

    return register_value;
}