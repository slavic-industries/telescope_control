# Save this content in generate_16mhz.py
import pigpio
import time

# Connect to pigpio daemon
pi = pigpio.pi()

if not pi.connected:
    exit(0)

# Define the GPIO pin
GPIO_PIN = 4

# Set the GPIO mode
pi.set_mode(GPIO_PIN, pigpio.OUTPUT)

# Generate a 16 MHz signal
pi.hardware_clock(GPIO_PIN, 8000000)

try:
    # Keep the script running to maintain the signal
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Clean up on exit
    pi.hardware_clock(GPIO_PIN, 0)
    pi.stop()
