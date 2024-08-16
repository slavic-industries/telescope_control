#ifndef RPI_GPIO_CONTROL__GPIO_CONTROLLER_H_
#define RPI_GPIO_CONTROL__GPIO_CONTROLLER_H_

namespace rpi_gpio_control
{
    class GpioController
    {
    public:
        GpioController();
        ~GpioController();
        void set_gpio_high(int gpio);
        void set_gpio_low(int gpio);
        int read_gpio(int gpio);
    };
} // namespace rpi_gpio_control

#endif // RPI_GPIO_CONTROL__GPIO_CONTROLLER_H_
