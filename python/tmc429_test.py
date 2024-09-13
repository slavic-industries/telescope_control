import spidev
import pigpio
import time

TMC429_CS_PIN = 5   # GPIO 5
TMC2130_CS_PIN = 8  # GPIO 8

spi_datagram = {"address" : 0x00,
                "rw" : 0b0,
                "data" : 0x000000
                }



# Function to write to a register on the TMC429
def write_register(spid, address, data):
    # The first byte contains the write flag (0x80) and the register address
    first_byte = 0x80 | (address & 0x7F)
    # Convert the data to a list of bytes (assuming 3 bytes)
    data_bytes = [(data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF]
    # Send the command and data
    spid.xfer2([first_byte] + data_bytes)

# Function to read from a register on the TMC429
def write_read_register(gpiod, addr, rw, data, cs_pin):

  byte_4 = addr << 1 | rw
  byte_3 = (data >> 16) & 0xFF
  byte_2 = (data >> 8)  & 0xFF
  byte_1 = data         & 0xFF
  datagram = [byte_4, byte_3, byte_2, byte_1]

  gpiod.spi_open(0, 500000, 0b11000011)
  gpiod.write(cs_pin, pigpio.LOW)
  count, rx_data = gpiod.spi_xfer(0, datagram) #
  gpiod.write(cs_pin, pigpio.HIGH)
  gpiod.spi_close(0)

  data = rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3]
  return data

def main():


  gpiod = pigpio.pi()

  # gpiod.spi_close(0)

  gpiod.set_mode(TMC429_CS_PIN, pigpio.OUTPUT)
  gpiod.write(TMC429_CS_PIN, pigpio.HIGH)
  try:
    # gpiod.spi_open(0, 1000000, 0b11000011)



    # byte_4 = 0b00011010
    # byte_3 = 0b00000000
    # byte_2 = 0b00000001
    # byte_1 = 0b00100011
    # print(f"Sent addr:\t {(byte_4):08b}")
    # print(f"Sent data:\t {(byte_3 << 16 | byte_2 << 8 | byte_1):0024b}")
    # datagram = [byte_4, byte_3, byte_2, byte_1]


    # gpiod.write(TMC429_CS_PIN, pigpio.LOW)
    # count, rx_data = gpiod.spi_xfer(0, datagram) #
    # gpiod.write(TMC429_CS_PIN, pigpio.HIGH)

    # tmc429_status = rx_data[0]
    # rx_data = rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3]

    # print(f"Status byte:\t {tmc429_status:08b}")
    # print(f"Data bytes:\t{rx_data: 0024b}")
    

    # # print("Read addr")
    # # byte_4 = 0b00011011
    # # byte_3 = 0b00000000
    # # byte_2 = 0b00000000
    # # byte_1 = 0b00000000
    # # print(f"Sent addr:\t {(byte_4):08b}")
    # # print(f"Sent data:\t {(byte_3 << 16 | byte_2 << 8 | byte_1):0024b}")
    # # datagram = [byte_4, byte_3, byte_2, byte_1]

    # # gpiod.write(TMC429_CS_PIN, pigpio.LOW)
    # # count, rx_data = gpiod.spi_xfer(0, datagram) #
    # # gpiod.write(TMC429_CS_PIN, pigpio.HIGH)

    # # tmc429_status = rx_data[0]
    # # rx_data = rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3]

    # # print(f"Status byte:\t {tmc429_status:08b}")
    # # print(f"Data bytes:\t{rx_data: 0024b}")


    # print("Version Datagram")
    # byte_4 = 0b01110011
    # byte_3 = 0b00000000
    # byte_2 = 0b00000000
    # byte_1 = 0b00000000
    # print(f"Sent addr:\t {(byte_4):08b}")
    # print(f"Sent data:\t {(byte_3 << 16 | byte_2 << 8 | byte_1):0024b}")
    # datagram = [byte_4, byte_3, byte_2, byte_1]

    # gpiod.write(TMC429_CS_PIN, pigpio.LOW)
    # count, rx_data = gpiod.spi_xfer(0, datagram) #
    # gpiod.write(TMC429_CS_PIN, pigpio.HIGH)

    # tmc429_status = rx_data[0]
    # rx_data = rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3]

    # print(f"Status byte:\t {tmc429_status:08b}")
    # print(f"Data bytes:\t{rx_data: 0024b}")
    # print(hex(rx_data))


    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))
    rx_data = write_read_register(gpiod, 0b0111001, 0b1, 0x000000, TMC429_CS_PIN)
    print(hex(rx_data))




  except Exception as e:
    print(f"Exception: {e}")
 #  gpiod.spi_close(0)
  

  return 0

if __name__ == "__main__":
  main()
