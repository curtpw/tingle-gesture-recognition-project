/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.print("A0: "); Serial.println(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A1);
  // print out the value you read:
  Serial.print("A1: "); Serial.print(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A2);
  // print out the value you read:
  Serial.print("A2: "); Serial.print(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A3);
  // print out the value you read:
  Serial.print("A3: "); Serial.print(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A4);
  // print out the value you read:
  Serial.print("A4: "); Serial.print(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A5);
  // print out the value you read:
  Serial.print("A5: "); Serial.print(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A6);
  // print out the value you read:
  Serial.print("A6: "); Serial.print(sensorValue); Serial.print("\t");

    sensorValue = analogRead(A7);
  // print out the value you read:
  Serial.print("A7: "); Serial.println(sensorValue);

  delay(10);        // delay in between reads for stability
}
