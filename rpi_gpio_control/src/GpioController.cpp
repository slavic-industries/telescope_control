#include "rpi_gpio_control/GpioController.h"
#include <pigpio.h>
#include <iostream>

namespace rpi_gpio_control
{
    GpioController::GpioController()
    {
        if (gpioInitialise() < 0)
        {
            std::cerr << "Failed to initialize pigpio" << std::endl;
        }
        else
        {
            std::cout << "pigpio initialized successfully" << std::endl;
        }
    }

    void GpioController::set_gpio_high(int gpio)
    {
        gpioSetMode(gpio, PI_OUTPUT);
        gpioWrite(gpio, 1);
        std::cout << "GPIO " << gpio << " set to HIGH" << std::endl;
    }

    void GpioController::set_gpio_low(int gpio)
    {
        gpioSetMode(gpio, PI_OUTPUT);
        gpioWrite(gpio, 0);
        std::cout << "GPIO " << gpio << " set to LOW" << std::endl;
    }

    int GpioController::read_gpio(int gpio)
    {
        gpioSetMode(gpio, PI_INPUT);
        int value = gpioRead(gpio);
        std::cout << "GPIO " << gpio << " read value: " << value << std::endl;
        return value;
    }

    GpioController::~GpioController()
    {
        gpioTerminate();
        std::cout << "pigpio terminated" << std::endl;
    }
} // namespace rpi_gpio_control
