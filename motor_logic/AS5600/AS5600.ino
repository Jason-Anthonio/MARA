const int analogPin = A0; // Connect AS5600 OUT pin here

//DIR goes to GND for clockwise rotation or VCC for anticlockwise
//GPO always goes to VCC
//VCC can be 5V or 3.3V as long as its consistent

//when the magnet is shifted or distanced by a bit, the read became wavelike. It needs a failsafe.

void setup() {
  Serial.begin(115200);
}

void loop() {
  int sensorValue = analogRead(analogPin);
  
  // Convert analog reading (0-1023) to voltage (0.0 - 5.0V)
  float voltage = sensorValue * (5.0 / 1023.0);
  
  // Convert to approximate degrees (0 to 360)
  float degrees = sensorValue * (360.0 / 1023.0);

  Serial.print("Analog Value: ");
  Serial.print(sensorValue);
  Serial.print("\t Voltage: ");
  Serial.print(voltage);
  Serial.print("V \t Angle: ");
  Serial.print(degrees);
  Serial.println("°");
  
  delay(100);
}
