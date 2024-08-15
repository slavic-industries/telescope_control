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

### ROS2 Nodes

#### target_data_node

- Description: Provides data about celestial targets.
- Subscribers:
- Publishers:
- Service Servers:
    - `/request_target_data [telescope_interfaces/srv/RequestTargetData]`
    - `/set_observer [telescope_interfaces/srv/SetObserver]`
- Service Clients:


### ROS2 Services

#### Set Observer

```
float64 lat
float64 lon
float64 alt
float64 press
float64 temp
float64 rel_humid
string time_zone
---
bool success
```

- Server: `target_data_node`
- Description: Set the observer position on the Earth to get corect target coordinates

#### Set Target

```
string target_name
---
bool success
string message
```

- Server: `telescope_target_data_node`
- Description: Get target data.