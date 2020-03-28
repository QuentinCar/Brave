 ## GoPro
  ```shell
 $ roslaunch gopro_node gopro.launch
 ```
 Control the gopro and publish images

 ### Topic
 **Publisher** : /gopro/raw_image<br/>
 				/gopro/undistort_image

### Launch Sequence
	
	1. Switch on the Gopro
	2. The Intel Compute Stick wifi interface should be connected to the GoPro's wifi
	3. roslaunch gopro_node gopro.launch
	
#### Common Mistake
	4. You will have several warnings for a few seconds (~30 sec) just ignore them
	5. Sometimes the programm will crash (only when launch) just try to re-launch it (generaly 2-3 time max)