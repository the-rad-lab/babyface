# BabyFace

The code for the machine-augmented costume used in the "Baby Face" performance

An Arduino Uno Board is used as the microcontroller to control the components. The wings in the suit can either be controlled tactilely using two buttons (one each for slow and fast movement of the wings) or somatically using the FSR sensor as a breath sensor. The system parameters in the code are tunable to the performers liking. 

### Input Components
1. Force-Sensitive Resistor (FSR) (optional)
2. Left Flex Sensor
3. Right Flex Sensor
4. Two Push-Buttons for Manual mode (slow and fast)

### Output Components
1. Left Wing
2. Right Wing
3. 8x8 NeoPixel Matrix (optional)
4. NeoPixel LED Strip

### Calibration
Calibration of the flex sensors (and FSR) is needed before each performance. This ensures that the trigger values correspond to the artists intention. The accompanying calibration code file can be run to find the optimal values. Only a single trigger value is needed for the flex sensors. The figure below illustrates the relationship between the flex sensor and the triggering of the traveling lights on the LED strip. Similarly, the FSR sensor (if it is used) is calibrated to find the lowest and highest digital value between 0-1024. The wings operate within that range and the value is mapped to the servo angle. The Serial Monitor or the Serial Plotter in the Arduino IDE can be used to read these values.

![Flex Sensor and Body Flow](/images/flexsensorimage.png) 

### Parts List
![Parts](/images/parts.png)

- [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3)
- Standard Sized [Servo Motors](https://www.amazon.com/ANNIMOS-Digital-Waterproof-DS3218MG-Control/dp/B076CNKQX4/) x 2
- [Flex Sensor](https://www.adafruit.com/product/182) x 2
- [Push-Buttons](https://www.digikey.com/product-detail/en/e-switch/RP3502ARED/EG1930-ND/280448)
- [NeoPixel LED Strip](https://www.adafruit.com/product/1138?length=1)
- [FSR](https://www.adafruit.com/product/1071)
- [8x8 NeoPixel LED Matrix](https://www.adafruit.com/product/1071)

#### TODO
- Add wings drawings and corresponding parts needed to make one
- Make code more object oriented

