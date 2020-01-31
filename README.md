# Brave

## Connect to Brave (via Ethernet)
 ```shell
 $ ssh brave@10.42.0.1
 #brave's password : brave
 ```
 
 ## Ublox
  ```shell
 $ roslaunch ublox ublox.launch
 ```
 Ublox correspond to the Furuno Weather Station.
 ### Topic
Publisher : /ublox/GPRMC<br/>
            /ublox/HCHDG
            /ublox/WIMDA
            /ublox/WIMWV
            /ublox/raw_GPRMC
            /ublox/raw_HCHDG
            /ublox/raw_WIMDA
            /ublox/raw_WIMWV
            
 ## Maestro
  ```shell
 $ roslaunch maestro_node maestro.launch
 ```
 Control the servo 
 ### Topic
 Subscriber : /Command
