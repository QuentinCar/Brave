# Brave

## Connect to Brave 

### via Ethernet
 ```shell
 $ ssh brave@10.42.0.1
 #brave's password : brave
 ```
 ### via SSH
 ```shell
 $ ssh brave@192.168.0.55
 #brave's password : brave
 ```

 ## Launch the line following
  ```shell
 $ roslaunch controller controller.launch    
 ```
 ### Topic
 **Publisher**: /Command

 **Subscriber**:/ublox/GPRMC <br/>
                /ublox/HCHDG <br/>
                /ublox/WIMDA <br/>
                /ublox/WIMWV <br/>

 
 ## Ublox
  ```shell
 $ roslaunch ublox ublox.launch
 ```
 Ublox correspond to the Furuno Weather Station.
 ### Topic
**Publisher**:  /ublox/GPRMC<br/>
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
 **Subscriber** : /Command
 
 ## GoPro
  ```shell
 $ roslaunch gopro_node gopro.launch
 ```
 Control the gopro and publish the image 
 ### Topic
 **Publisher** : /gopro/image
