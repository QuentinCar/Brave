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

 ## Launch the line following (main mission)
  ```shell
 $ roslaunch controller controller.launch
 ```
 This file will launch the maestro, ublox and line following nodes.
 ### Topic
 **Publisher**: /Command

 **Subscriber**:/ublox/GPRMC <br/>
                /ublox/HCHDG <br/>
                /ublox/WIMDA <br/>
                /ublox/WIMWV <br/>
 


 
