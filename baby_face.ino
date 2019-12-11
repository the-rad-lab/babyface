#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>

#define MODE_BUTTON       12 // used to change mode from auto to manual
#define RED_LED           13 // indicates manual mode if on

// Input Pins
#define FSR_PIN           A0 // force sensitive resistor around chest
#define FLEX_L_PIN        A2 // left Flex sensor
#define FLEX_R_PIN        A5 // right Flex sensor
#define BUTTON_SLOW_PIN   4  // slow controller for wings (in manual mode)
#define BUTTON_FAST_PIN   2  // fast controller for wings (in manual mode)

// Output Pins
#define SERVO_L_PIN       9  // left servo
#define SERVO_R_PIN       10 // right servo
#define LED_MATRIX_PIN    6  // led matrix option
#define LED_STRIP_PIN     5  // led strip around neck

// System Parameters
#define BRIGHTNESS 32 //Sets the brightness of the LED's
#define MANUAL_SWITCH_INTERVAL 1000
#define UPPER 95 // Angle Limit for Servo
#define LOWER 50 // Angle Limit for Servo
#define SLOW 80 // Servo control slow (in manual mode)
#define FAST 20 // Servo control fast (in manual mode)
#define FSR_LOWER 720 // lower constraining value of FSR readings
#define FSR_UPPER 800 // upper constraining value of FSR readings
#define STRIP_UPDATE_INTERVAL 20
#define CART_UPDATE_INTERVAL 200
#define MATRIX_INTERVAL 5000 // updates led matrix every 5s

// Define LED matrix width and height.
#define mw 8
#define mh 8

#define N_LEDS 60 // Number of LEDs in the strip

bool manual;
unsigned long manualLastUpdate = 0;

bool slow_button = false;
unsigned long manualSlowPrevMillis = 0;
unsigned long manualFastPrevMillis = 0;

// servo object initializations
Adafruit_TiCoServo servo_R;
Adafruit_TiCoServo servo_L;
int angle1;
int angle2;

// variables used in smoothing fsr outputs
unsigned long autoServoLastUpdate = 0;
const int numReadings = 6;
int fsrReadings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int fsr_average = 0;

Adafruit_NeoMatrix *matrix = new Adafruit_NeoMatrix(mw, mh, LED_MATRIX_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

// led strip initializations
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800); 
const int numCarts = 5;
int rail_forward[numCarts];
int rail_backward[numCarts];
int activeCarts_forward;
int activeCarts_backward;
uint32_t stripColor_forward;
uint32_t stripColor_backward;

int stripUpdateInterval;  
int cartUpdateInterval;      // interval between updates
unsigned long stripLastUpdate = 0;
unsigned long cartLastUpdate = 0;

unsigned long matrixLastUpdate = 0;
int matrix_count = 0;

// led matrix emojis 
static const uint16_t PROGMEM
  RGB_bmp[][64] = {
    //00: Happy Face
    {
      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
      0x0000, 0xFFE0, 0xFFE0, 0x0000, 0x0000, 0xFFE0, 0xFFE0, 0x0000,   // 0x0010 (16) pixels
      0x0000, 0xFFE0, 0xFFE0, 0x0000, 0x0000, 0xFFE0, 0xFFE0, 0x0000, 
      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0020 (32) pixels
      0xFFE0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFE0, 
      0xFFE0, 0xFFE0, 0x0000, 0x0000, 0x0000, 0x0000, 0xFFE0, 0xFFE0,   // 0x0030 (48) pixels
      0x0000, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0, 0x0000, 
      0x0000, 0x0000, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0, 0x0000, 0x0000,   // 0x0040 (64) pixels
    },
    //01: Heart
    {
      0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
      0x0000, 0xF800, 0xF800, 0x0000, 0x0000, 0xF800, 0xF800, 0x0000,   // 0x0010 (16) pixels
      0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 
      0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800,   // 0x0020 (32) pixels
      0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 
      0x0000, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0xF800, 0x0000,   // 0x0030 (48) pixels
      0x0000, 0x0000, 0xF800, 0xF800, 0xF800, 0xF800, 0x0000, 0x0000, 
      0x0000, 0x0000, 0x0000, 0xF800, 0xF800, 0x0000, 0x0000, 0x0000, 
    }
  };

// Convert a BGR 4/4/4 bitmap to RGB 5/6/5 used by Adafruit_GFX
void fixdrawRGBBitmap(int16_t x, int16_t y, const uint16_t *bitmap, int16_t w, int16_t h) {    
    matrix->drawRGBBitmap(x, y, bitmap, w, h);
}

void display_rgbBitmap(uint8_t bmp_num) { 
    static uint16_t bmx,bmy;

    fixdrawRGBBitmap(bmx, bmy, RGB_bmp[bmp_num], 8, 8);
    bmx += 8;
    if (bmx >= mw) bmx = 0;
    if (!bmx) bmy += 8;
    if (bmy >= mh) bmy = 0;
    matrix->show();
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);

  // Assigning manual push_button and Red LED to their respective pins
  pinMode(MODE_BUTTON, INPUT);
  pinMode(RED_LED, OUTPUT);
  manual = false;
  // Assigning the manual mode buttons to their respective pins
  pinMode(BUTTON_SLOW_PIN, INPUT);
  pinMode(BUTTON_FAST_PIN, INPUT);


  angle1 = UPPER;
  angle2 = LOWER;
  servo_R.write(SERVO_R_PIN);
  servo_L.write(SERVO_L_PIN);
  servo_R.attach(SERVO_R_PIN); //attaches the servo object to its respective pin
  servo_L.attach(SERVO_L_PIN);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      fsrReadings[thisReading] = 0;
  }
  
  matrix->begin();
  matrix->setBrightness(BRIGHTNESS);

  strip.begin();
  for (int thisCart = 0; thisCart < numCarts; thisCart++) {
      rail_forward[thisCart] = -1;
      rail_backward[thisCart] = -1;
  }
  activeCarts_forward = 0;
  activeCarts_backward = 0;
  stripColor_forward = strip.Color(0, 255, 0);
  stripColor_backward = strip.Color(255, 0, 255);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // led matrix update block
  if((unsigned long)(millis() - matrixLastUpdate) >= MATRIX_INTERVAL) {
    matrixLastUpdate = millis();
    int emoji = matrix_count%2;
    display_rgbBitmap(emoji);
    matrix_count += 1;
  }

  // code block to check if the servos are being controlled manually
  if((unsigned long)(millis() - manualLastUpdate) >= MANUAL_SWITCH_INTERVAL) {
    manualLastUpdate = millis();
    int buttonState = digitalRead(MODE_BUTTON);
    if(buttonState == HIGH) {
      if (manual == false) {
        manual = true;
        digitalWrite(RED_LED, HIGH);
      } else {
        manual = false;
        digitalWrite(RED_LED, LOW);
      }
    }
  }

  // Servo Controls
  if (manual) {
    int buttonState1 = digitalRead(BUTTON_SLOW_PIN);
    int buttonState2 = digitalRead(BUTTON_FAST_PIN);
    if (buttonState1 == HIGH) { 
      slow_button = true;
      if((unsigned long)(millis() - manualSlowPrevMillis) >= SLOW) {
        manualSlowPrevMillis = millis();
        angle1 = angle1 - 1;
        if (angle1 <= LOWER) {angle1 = LOWER;}
        angle2 = angle2 + 1;
        if (angle2 >= UPPER) {angle2 = UPPER;}
      
        servo_R.write(angle1);
        servo_L.write(angle2);
      }
    } else if (buttonState2 == HIGH) { 
      slow_button = false;
      if((unsigned long)(millis() - manualFastPrevMillis) >= FAST) {
        manualFastPrevMillis = millis();
        angle1 = angle1 - 1;
        if (angle1 <= LOWER) {angle1 = LOWER;}
        angle2 = angle2 + 1;
        if (angle2 >= UPPER) {angle2 = UPPER;}
    
        servo_R.write(angle1);
        servo_L.write(angle2);
      }
    } else if (buttonState1 == LOW && slow_button) {
      if((unsigned long)(millis() - manualSlowPrevMillis) >= SLOW) {
        manualSlowPrevMillis = millis();
        angle1 = angle1 + 1;
        if (angle1 >= UPPER) {angle1 = UPPER;}
        angle2 = angle2 - 1;
        if (angle2 <= LOWER) {angle2 = LOWER;}
    
        servo_R.write(angle1);
        servo_L.write(angle2);
      }
    } else if (buttonState2 == LOW && !slow_button) {
      if((unsigned long)(millis() - manualFastPrevMillis) >= FAST) {
        manualFastPrevMillis = millis();
        angle1 = angle1 + 1;
        if (angle1 >= UPPER) {angle1 = UPPER;}
        angle2 = angle2 - 1;
        if (angle2 <= LOWER) {angle2 = LOWER;}
    
        servo_R.write(angle1);
        servo_L.write(angle2);
      }
    }
  } else { // block runs if manual is false. uses fsr readings to control servos
    if((unsigned long)(millis() - autoServoLastUpdate) >= 50) {
      autoServoLastUpdate = millis();
      // subtract the last reading:
      total = total - fsrReadings[readIndex];
      // read from the sensor:
      fsrReadings[readIndex] = analogRead(FSR_PIN);
      //Serial.println(fsrReadings[readIndex]);
      total = total + fsrReadings[readIndex]; // add the reading to the total
      readIndex = readIndex + 1; // advance to the next position in the array
      if (readIndex >= numReadings) { // if we're at the end of the array...
        readIndex = 0; // ...wrap around to the beginning:
      }
      
      fsr_average = total / numReadings; // calculate the average
      Serial.println(fsr_average);
      fsr_average = constrain(fsr_average, FSR_LOWER, FSR_UPPER); // constrains the average

      int angle1 = map(fsr_average, FSR_LOWER, FSR_UPPER, UPPER, LOWER);
      int angle2 = map(fsr_average, FSR_LOWER, FSR_UPPER, LOWER, UPPER);
  
      servo_R.write(angle1);
      servo_L.write(angle2);
    }
  }

  //Flex Sensor and LED strip code blocks
  int flexReadingR = analogRead(FLEX_R_PIN);
  int flexReadingL = analogRead(FLEX_L_PIN);

  if((unsigned long)(millis() - cartLastUpdate) >= CART_UPDATE_INTERVAL) {
    cartLastUpdate = millis();
    if (flexReadingR < 53 && activeCarts_forward < numCarts) {
      activeCarts_forward += 1;
      for (int thisCart = 0; thisCart < numCarts; thisCart++) {
        if(rail_forward[thisCart] == -1){
          rail_forward[thisCart] = 0;
          break;
        }
      }
    }
    if (flexReadingL < 64 && activeCarts_backward < numCarts) {
      activeCarts_backward += 1;
      for (int thisCart = 0; thisCart < numCarts; thisCart++) {
        if(rail_backward[thisCart] == -1){
          rail_backward[thisCart] = N_LEDS - 1;
          break;
        }
      }
    }
  }
  if((unsigned long)(millis() - stripLastUpdate) > STRIP_UPDATE_INTERVAL) {
    stripLastUpdate = millis();
    for (int thisCart = 0; thisCart < numCarts; thisCart++) {
      if(rail_forward[thisCart] != -1) {
         rail_forward[thisCart] += 1;
         strip.setPixelColor(rail_forward[thisCart] , stripColor_forward);
         strip.setPixelColor(rail_forward[thisCart]-2, 0);
         strip.show();
         if (rail_forward[thisCart] > N_LEDS) {
           rail_forward[thisCart] = -1;
           activeCarts_forward -= 1;
         }
      }
    }
    for (int thisCart = 0; thisCart < numCarts; thisCart++) {
      if(rail_backward[thisCart] != -1) {
         rail_backward[thisCart] -= 1;
         strip.setPixelColor(rail_backward[thisCart], stripColor_backward);
         strip.setPixelColor(rail_backward[thisCart]+2, 0);
         strip.show();
         if (rail_backward[thisCart] < 0) {
           rail_backward[thisCart] = -1;
           activeCarts_backward -= 1;
         }
      }
    }
  }
  
}
