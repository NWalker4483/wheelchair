
# April 23th - April 25th 2021 Wheelchair Update

by: Nile Walker

For this Baltimore visit I shifted my focus towards driving the wheelchair with the Arduino. It hasn't been successful so far but I've learned a lot about the wheelchair.

## **Software**

### **Joystick**
By using an Arduino mega Ive been able to sniff the I2C traffic from the wheelchair to the joystick and use that to determine which register I needed to read to get the X and Y values of the stick. There's also some sort of set up byte that needs to be sent before reading data from the sensor or it will not respond to the 0x46 read attempt.

The first and sixth bit do seem to change when I'm sniffing traffic between devices but don't change at all when I'm reading directly from the joystick to the Arduino.  I can't imagine offhand way two unnecessary bytes would be sent back but it's possible that these changes are due to issues with trying to listen to and display the data with the arduino. Since they don't change when I'm using the more tested Wire library I'm going to assume that they are not meant to change for now.

<img src="sniff.png" width="500px"/>

The third and fifth bytes seem to smoothly follow the X and Y of the joystick when rotated as shown below.

<img src="plot.png" width="500px"> 

The second and fourth bytes also seem to follow the motion of the axis but significantly less smoothly this leads me to think that this is part of a larger number and these two values are combined to get greater precision. I tried some methods that I saw on stack overflow but none of them produced reasonable results so I'm going to need to look into this a bit more before I can do anything significant.

## **Hardware**

### **Arduino**

I've yet to be able to get the wheelchair to turn on properly with just an arduino connected. I setup the arduino uno to respond to the first 0x36 byte with 0x21 just like the joystick but not to the 0x46 byte. So this is very likely part of that issue but I have to focus on an essay due monday.

### **Joystick**

While the sensor is powered by 3.3 V like initially assumed, it's data lines operate at 2.7 V. So I connected the arduinos 5 V line to voltage divider in order to provide a reference for the logic level shifter.

Also somehow the sensor can still respond and send out readings when VCC is not connected I'm not sure if somehow the clock line is powering the sensor but that's a bit out of my wheelhouse and I just wanted to mention it in the update if anyone had any ideas.