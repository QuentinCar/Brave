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
Publisher :<br/>
            /ublox/GPRMC<br/>
            /ublox/HCHDG<br/>
            /ublox/WIMDA<br/>
            /ublox/WIMWV<br/>
            /ublox/raw_GPRMC<br/>
            /ublox/raw_HCHDG<br/>
            /ublox/raw_WIMDA<br/>
            /ublox/raw_WIMWV<br/>
            
 ## Maestro
  ```shell
 $ roslaunch maestro_node maestro.launch
 ```
 Control the servo 
 ### Topic
 Subscriber : /Command
