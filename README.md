# Road Trip

Authors: John Culley

Date: 2023-04-04

### Solution Design

**Cruise Control:**

To effectively create a cruise control for the buggy, the wheel speed and the front distance need to be obtained. To get the wheel speed, a photo sensor was connected to the ESP32 with an ADC port. Within the ESP32 code, the function “wheel_read()” obtains the raw data value from the photo sensor and within the function converts the raw data to voltage by multiplying by five and dividing by 1023 which are the adjustment values. With a function to effectively return the voltage from the photo sensor, it is called within another function “PulseCount()”. “PulseCount()” reads the current voltage of the photo sensor and stores it in a variable “voltage”. This current voltage is compared to the previous reading from the sensor which is stored in “prev_volt”. If the difference between the two values is greater than 8, the pulse count value increases by 1. Now with a successfully obtained pulse count, the function “getSpeed()” is used to return the speed. Within “getSpeed()”, the speed is calculated by doing a conversion to meters per second using the pulse count and the sample time variables. This speed is then converted to centimeters per second for an easier viewing number and displayed on the alpha numeric display which is attached to the buggy. 

To obtain the front distance from the buggy a Lidar V3 is used through an ADC port and its reading is obtained and converted to meters. 

Combining the use of these two metrics, the cruise control algorithm is developed. The buggy uses a PID controller to control the buggy’s speed and keep it as close as possible to .3 m/s by modifying the speed whether the obtained speed value is above or below the desired value. Secondly, the cruise control needs to recognize obstructions in the road and come to a complete stop within 20 cm of the object. To do this, our front Lidar reads the current and previous distance and looks for sudden differences outside of the expected range value and if it senses that an obstruction is in front of the vehicle it will start to slow down the wheel speed at 50 cm so that it can come to a complete stop within 20cm. 
</p>
<img width="514" alt="Screen Shot 2023-04-04 at 10 55 50 PM" src="https://user-images.githubusercontent.com/91488781/229970937-c2058340-23eb-4239-9d59-1cde34f58e10.png">




**Lane Assistance:**

To effectively create lane assistance, there are two Lidars, one on the front right of the buggy and one on the back right of the buggy. Both of the Lidars receive their readings through the ADC port and convert their readings into meters. All three of the Lidars on the buggy are able to operate at the same time because their addresses are manually being assigned because otherwise they would all have the same ADC address and would not be able to operate in conjunction. The code cycle operates in .2 seconds, so the lane assistance PID checks the distance of both Lidars and there are three circumstances: the values are within .02 meters of each other meaning the buggy is moving straight, the front Lidar is more than .02 meters larger than the back Lidar meaning that the buggy is turning left, and the back Lidar is more than .02 larger than the front Lidar meaning that the buggy is turning right. 

Front Lidar = Back Lidar: wheel angle does not change  
Front Lidar > Back Lidar: wheel angle is turned to the right for one timer cycle  
Back Lidar > Front Lidar: wheel angle is turned to the left for one timer cycle  
<img width="296" alt="Screen Shot 2023-04-04 at 10 55 46 PM" src="https://user-images.githubusercontent.com/91488781/229969517-98d86730-c26d-4fce-9a23-d56ede57c865.png">

<img width="227" alt="Screen Shot 2023-04-04 at 10 58 53 PM" src="https://user-images.githubusercontent.com/91488781/229969905-3864a76e-bbe5-41d3-b209-99259d3553df.png">

**Left Turn:**

To turn to the left, there are three states of the turn: 

1. Recognize that a turn is approaching  
2. Turn the wheels and begin the turn  
3. Recognize that the turn is complete and return to straight  


Recognize that a turn is approaching-  
The front Lidar on the buggy will be detecting the distance as the buggy approaches the wall, and as long as the buggy didn’t register an obstruction, the buggy will begin the turn when it is 1.5 meters away from the wall. 

Turn the wheels and begin turn-  
When the logical block is entered after the buggy is 1.5 meters away from the wall, the speed will be slowed down to .1 m/s and the wheel turn will be maximized to the left. 

Recognize that the turn is complete and return to straight-  
When the turn is started, a check will be performed on the two side Lidars comparing their values. Once the two values are within .05 meters of each other, the car will return back to going straight. 


**Remote Control:**

To create a remote control function for the buggy there were three components that needed to be created together: the user-facing HTML page, the node.js server, and the UDP client. 

The HTML page created a button that checks for user input. When the user presses the button, it uses AJAX to initiate an HTTP Request with the node server. When the node server receives this variable from the HTML page, it then uses UDP to send the packet to the ESP32 Client. This variable “run” will either be set to 0 for off or 1 for run. The code to run the buggy can only be accessed when “run” is set to 1, so the user can control when the buggy is on and off. 


### Sketches/Diagrams
<p align="center">
<img width="765" alt="Screen Shot 2023-04-04 at 10 50 40 PM" src="https://user-images.githubusercontent.com/91488781/229969990-761b50f6-91ca-4629-9fef-481978c200e8.png">
<img width="397" alt="Screen Shot 2023-04-04 at 10 50 29 PM" src="https://user-images.githubusercontent.com/91488781/229970012-05f10da1-c9ed-442f-a608-663e618b30f0.png">
<img width="670" alt="Screen Shot 2023-04-04 at 10 50 44 PM" src="https://user-images.githubusercontent.com/91488781/229970023-7c5b687f-4db5-495c-8f79-c6c0f26efe76.png">

</p>
<p align="center">
Images of the Buggy
</p>
<img width="518" alt="Screen Shot 2023-04-04 at 10 55 41 PM" src="https://user-images.githubusercontent.com/91488781/229970049-4272aaed-a88b-4558-bb72-9d60d64a7069.png">
</p>
Simplified circuit diagram with all of the sensors used on the buggy 


### Supporting Artifacts
- [Link to video demo](https://drive.google.com/file/d/1X4F4XvbshXq4FzzXS3Gg4VRr5AfhFGAZ/view?usp=sharing). Not to exceed 120s


### Modules, Tools, Source Used Including Attribution
ESP32, ESP-IDF, Visual Studio Code, Lidar V3, i2c display, battery pack, buggy, H Bridge, photo sensor, Node.js, AJAX, HTTP Request

### References


