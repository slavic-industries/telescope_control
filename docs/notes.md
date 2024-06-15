#### Used Libraries

- `pigpio` library - https://abyz.me.uk/rpi/pigpio/index.html

#### Add user to gpio group

##### If `gpio` group does not exist create group:

1. Create the group
```
sudo groupadd gpio
```
2. Set udev rules for gpio. Open:
```
sudo nano /etc/udev/rules.d/99-gpio.rules
```
Add the following lines:
```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
SUBSYSTEM=="gpio", KERNEL=="gpio*", MODE="0660", GROUP="gpio"
```
Save and exit `nano`
3. Reload udev rules:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```


**Run the following command:**

```
sudo usermod -aG gpio <username>
```
