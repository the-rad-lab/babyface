#define FSR_PIN A0
#define FLEX_L_PIN A2
#define FLEX_R_PIN A5

#define UPDATE_INTERVAL 100

unsigned long lastUpdate = 0;

void setup() {
  // put your setup code here, to run once:

  // We'll send debugging information via the Serial monitor
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if((millis() - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = millis();
    int reading_FSR = analogRead(FSR_PIN);
    int reading_FLEX_L = analogRead(FLEX_L_PIN);
    int reading_FLEX_R = analogRead(FLEX_R_PIN);

    Serial.print(reading_FSR); //prints out the FSR reading first.
    Serial.print(",");
    Serial.print(reading_FLEX_L); //prints out the left Flex Sensor reading.
    Serial.print(",");
    Serial.println(reading_FLEX_R); //prints out the right Flex Sensor reading.
  }
}
