# A9 V720 Naxclow Camera 

![Camera itself](https://raw.githubusercontent.com/intx82/a9-v720/master/img/0.jpg =480x)

**This app has been tested only with camera version 202212011602 and only which uses Chinese V720 APP. See pictures of camera PCB below**

## How to use script and short brief

To use this script, the camera should have inserted SD card. All data are reads from SD card.

Then, turn on the camera and attach to their Wifi-AP. AP name should(could) starts from prefix `Nax` (as i suppose it means name of manufacturer: Naxclow) For example, i have: `Nax_210000211234`

Camera should give IP in range 192.168.169.100 / 255.255.255.0. After, you can try to connect with this script. The camera's Wi-Fi is very poor, and i am able to have a good connection only after connecting UART tool, which connects the camera's GND to my laptop.

By default, camera has IP 192.168.169.1 and didn't have any Web-page to configure it. Camera can be configured only via mobile-application.

Also, camera networking could be configured from UART via `ifconfig` and `wifi` commands.

**I would not recommend connecting camera to the internet, due to it downloads from Chinese server update package**

- List of all recorded videos inside camera:
    ```
    python3 src/a9_naxclow.py -l
    ```

    Camera filenames have datetime format with full date/hour/minute information. 
    
    For example: `202301312016.avi`. 
    
    But on SD card will be saved in different folders by date, `[date]/[hour]/[minute].avi`. (/20230131/20/16.avi)

- Downloading selected video file:
    
    ```
    python3 src/a9_naxclow.py -d 20230131-20-16 -o out.avi
    ```

    Where syntax are:
        src/a9_naxclow.py -d [date-hour-minute] -o [output-file]
    
    Date/Hour/Minute could be taken from first command (ie `-l` )
    if -o arg is not provided, the script will save the file as `out.avi`

    While downloading file from camera, recording is not going.

## Network options

After camera has been attached, camera provide an IP via DHCP in 192.168.169.* network. Camera itself will have ip 192.168.169.1 and have only one opened TCP port - 6123.

Camera have mixed binary/json/xml protocol, where most of all commands sends in JSON. Check <src/prot_udp.py> (which works over TCP) <src/prot_json_udp.py>, etc

**In AP mode there are no live video broadcasting** (or i didn't found it)

## Original source

Disassmbled original app (V720) sources could be found in orig-app folder. Very very early version of a9 script could be found in a9_old.py

## UART logs

Found some manufacturer passwords inside UART logs

```
com.naxclow.Camera

[WLAN_easyfReadId - 136]-Debue:  SET wifi = (NAXCLOW)  (34974033A) 
_wifi_easyjoin: ssid:NAXCLOW bssid:00:00:00:00:00:00 key:34974033A

_wifi_easyjoin: ssid:[ap0] bssid:00:00:00:00:00:00 key:1213aCBVDiop@  <--- Here ssid from most powerful AP in range

```
Full logs could be found in docs dir. <docs/uart.log>

## Camera photos

IP-camera has been built around BL7252 MCU. MCU SDK could be found at github

![PCB photo](https://raw.githubusercontent.com/intx82/a9-v720/master/img/4.jpg =480x)

All photos in <img> folder

## Where to buy this camera

On aliexpress, could be found by name `A9 V720 camera` and on 2023-01-31 costs 5.2 euro

