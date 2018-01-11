
//NOTE: Thermopile values are now smoothed   (((Xt-2...)/2 + Xt-1)/2 + X)/2    6/14/17
/* NN 1: mouth
   NN 2: fronthead
   NN 3: tophead
   NN 4: backhead
   NN 5: righthead
   NN 6: lefthead
 */
/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#define NRF52

#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include <Wire.h>
#include <KX126_SPI.h>        //accelerometer
#include <VL6180X.h>          //distance sensor


/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool    debug = true;        //turn serial on/off to get data or turn up sample rate
bool    debug_time = false;    //turn loop component time debug on/off

//native max loop speed is about 35 ms or 28Hz
float   speedLowpower  = 1000 / 2;  //2Hz default power saving speed
float   speedBluetooth = 1000 / 16; //16Hz while connected to 
float   speedBallpark  = 1000 / 8; //8Hz when NN approach target

float   speedMs = speedBallpark;

float   detect_objT_lowpass =    80;
float   detect_objT_highpass =   102;
int     tempScaleAdjust =        12;
int     limit_stopRepeatDetect = 200;
//int   read_heart_rate_interval = 20;

/********************************************************************************************************/
/************************ DEFINITIONS *******************************************************************/
/********************************************************************************************************/
//SCL = 12  SDA = 11   

//#define PIN_SERIAL_RX           14
//#define PIN_SERIAL_TX           13

//#define BLUE_LED_PIN            
#define GREEN_LED_PIN             15

#define BUTTON_PIN                29

//#define BUTTON_PIN_temp           4

//#define HEART_RATE_LED_PIN        4
//#define HEART_RATE_DETECTOR_PIN   29
//#define TOUCH_BUTTON_PIN          30
#define VIBRATE_PIN               8

//Accelerometer Pins
#define CS_PIN                    24

#define KX022_SDI 19
#define KX022_SDO 20
#define KX022_SCL 18
#define KX022_INT 23
//#define KX022_NCS 

#define PIN_SPI_MISO         (KX022_SDO)
#define PIN_SPI_MOSI         (KX022_SDI)
#define PIN_SPI_SCK          (KX022_SCL)

//Thermopile Addresses
#define MLX90615_I2CADDR          0x00
#define MLX90615_I2CADDR1         0x2A
#define MLX90615_I2CADDR2         0x2B 
#define MLX90615_I2CADDR3         0x2C
#define MLX90615_I2CADDR4         0x2D

// RAM
#define MLX90615_RAWIR1           0x04
#define MLX90615_RAWIR2           0x05
#define MLX90615_TA               0x26
#define MLX90615_TOBJ1            0x27
#define MLX90615_TOBJ2            0x28
// EEPROM
#define MLX90615_TOMAX            0x20
#define MLX90615_TOMIN            0x21
#define MLX90615_PWMCTRL          0x22
#define MLX90615_TARANGE          0x23
#define MLX90615_EMISS            0x24
#define MLX90615_CONFIG           0x25
#define MLX90615_ADDR             0x0E

//dummy LED pin for BLE
#define LED_PIN   3


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/

  //LED
    float   greenLED_timer = 0;
    int     LED_counter = 10;
    bool    greenLED_status = false;

  //Button
    int     buttonState = 0;         // variable for reading the pushbutton

  //MLP (Multi Layer Perceptron) LSTM Neural Net
  //NN weights
    int     nnLength = 500;
    float   F[500];
    int     transmittedCounter = 0;
    bool    flag_haveNeural = false;

  //Detection
    int     selectNN = 0;
    float   fiveInScore = 0;
    float   sevenInScore = 0;
    int     vibrate_counter = 0;
    bool    vibrate_status = false;
    bool    flag_detect = false;              //gesture detected!
    float   lastDetectTime = 0;
    bool    flag_stopRepeatDetect = false;

  //Timestamp
    float   clocktime = 0;
    
  //Bluetooth
    unsigned long microsPerReading, microsPrevious;
    float   accelScal;
    int     command_value = 99; //controlls how device and app talk to each other

  //System
    int     varState = 0; //variable state controlled in app and appended to data stream

  //MLX90615 Thermopiles
    float   TObj[4] = {0,0,0,0};
    float   TAmb[4] = {0,0,0,0};
    float   TAmbAv;

  //vl6180x Distance
    float   distance = 0;
 
  //KX126 Accelerometer
    // pins used for the connection with the sensor
    // the other you need are controlled by the SPI library):
    const int dataReadyPin = 6;
    const int chipSelectPin = 7;

    float     acc[3];
    double    pitch;
    double    roll;

/*  //APDS-9960 Gesture and Proximity
    uint8_t proximity_data = 0;
    int isr_flag = 0; 
*/

/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/

//Time of Flight LIDAR distance sensor
VL6180X vl6180x;

//KX022 Accelerometer
KX126_SPI kx126(CS_PIN);

//Bluetooth
// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral = BLEPeripheral();

// create service
//BLEService customService =    BLEService("FFFF");
BLEService customService =    BLEService("a000");

// create command i/o characteristics
BLECharCharacteristic    ReadOnlyArrayGattCharacteristic  = BLECharCharacteristic("a001", BLERead);
BLECharCharacteristic    WriteOnlyArrayGattCharacteristic = BLECharCharacteristic("a002", BLEWrite);

//create streaming data characteristic
BLECharacteristic        DataCharacteristic("a003", BLERead | BLENotify, 20);  //@param data - an Uint8Array.

//create streaming neural network i/o characteristic
//BLECharacteristic    ReadNeuralNetCharacteristic  = BLECharacteristic("a004", BLERead | BLENotify, 20); //@param data - an Uint8Array.
//BLECharacteristic    WriteNeuralNetCharacteristic  = BLECharacteristic("a005", BLEWrite, 20); //@param data - an Uint8Array.




/********************************************************************************************************/
/************************ UTILITY FUNCTIONS *************************************************/
/********************************************************************************************************/
float differenceBetweenAngles(float firstAngle, float secondAngle)
  {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
 }


/********************************************************************************************************/
/************************ MLX90615 THERMOPILE FUNCTIONS *************************************************/
/********************************************************************************************************/
uint16_t read16(uint8_t a, int sensorNum) {
  uint8_t _addr = MLX90615_I2CADDR;

  if(sensorNum == 0) _addr = MLX90615_I2CADDR;   //custom addresses
  else if(sensorNum == 1)  _addr = MLX90615_I2CADDR1;   
  else if(sensorNum == 2)  _addr = MLX90615_I2CADDR2;
  else if(sensorNum == 3)  _addr = MLX90615_I2CADDR3;
  else if(sensorNum == 4)  _addr = MLX90615_I2CADDR4;
  
  uint16_t ret;
  Wire.beginTransmission(_addr);                  // start transmission to device 
  Wire.write(a); delay(1);                        // sends register address to read from
  Wire.endTransmission(false);                    // end transmission
  Wire.requestFrom(_addr, (uint8_t)3); delay(1);  // send data n-bytes read
  ret = Wire.read();// delay(1);                    // receive DATA
  ret |= Wire.read() << 8;// delay(1);              // receive DATA
  uint8_t pec = Wire.read(); delay(1);
  return ret;
}

float readTemp(uint8_t reg, int sensorNum) {
  float temp;
  temp = read16(reg, sensorNum);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

double readObjectTempF(int sensorNum) {
  return (readTemp(MLX90615_TOBJ1, sensorNum) * 9 / 5) + 32;
}

double readAmbientTempF(int sensorNum) {
  return (readTemp(MLX90615_TA, sensorNum) * 9 / 5) + 32;
}

double readObjectTempC(int sensorNum) {
  return readTemp(MLX90615_TOBJ1, sensorNum);
}

double readAmbientTempC(int sensorNum) {
  return readTemp(MLX90615_TA, sensorNum);
}


/********************************************************************************************************/
/************************ MX25L6445E FLASH MEMORY FUNCTIONS *********************************************/
/********************************************************************************************************/


/********************************************************************************************************/
/************************ BLUETOOTH BLE FUNCTIONS *************************************************/
/********************************************************************************************************/
void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler

  //increase speed while connected to Bluetooth
 // speedMs = speedBluetooth;
  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Connected event, central: "));
    Serial.println(central.address());
  }
  delay(5);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  
  //bring spped back down to low power default
//  speedMs = speedLowpower;
  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Disconnected event, central: "));
    Serial.println(central.address());
  }
  delay(5);
}

void blePeripheralServicesDiscoveredHandler(BLECentral& central) {
  // central  services discovered event handler
  if(debug){
    Serial.print(F(" services discovered event, central: "));
    Serial.println(central.address());
  }
/*
  if (ReadOnlyArrayGattCharacteristic.canRead()) {
    Serial.println(F("ReadOnlyArrayGattCharacteristic"));
    ReadOnlyArrayGattCharacteristic.read();
  }

  if (WriteOnlyArrayGattCharacteristic.canWrite()) {
    Serial.println(F("WriteOnlyArrayGattCharacteristic"));

   // unsigned long writeValue = 42;
    static uint8_t writeValue[10] = {0};
  //  writeValue[0] = 5;

    WriteOnlyArrayGattCharacteristic.write((const unsigned char*)&writeValue, sizeof(writeValue));
  } */
  delay(5);
  //delay(2000);
}

void bleCharacteristicValueUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  
    if(debug){ Serial.print(F(" Begin bleCharacteristicValueUpdatedHandle: ")); }
    
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
 // char char_buf[2]={0,0};
  //int command_value;
  
  String bleRawVal = "";
  for (byte i = 0; i < the_length; i++){ 
    bleRawVal += String(the_buffer[i], HEX); 
  }

  char *char_buf = const_cast<char*>(bleRawVal.c_str());
  
  command_value = (int)strtol(char_buf, NULL, 16);

//  bleRawVal.toCharArray(temp_char_buffer, the_length);
 // sscanf(temp_char_buffer, "%x", &command_value);

//  if(debug) Serial.print("Raw command: "); Serial.println( the_buffer );
//  if(debug) 
  selectNN = command_value;
  Serial.print("APP COMMAND: "); Serial.println( command_value );



  BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength());
 // if(debug) delay(1000);
  delay(5);
}


void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print(F("Characteristic event, written: "));

  if (ReadOnlyArrayGattCharacteristic.value()) {
    if(debug) Serial.println(F("LED on"));
 //   digitalWrite(LED_PIN, HIGH);
  } else {
    if(debug) Serial.println(F("LED off"));
 //   digitalWrite(LED_PIN, LOW);
  }
 // delay(2000);
 //delay(10);
}

/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup() 
{
    Serial.begin(115200);
    if(debug) Serial.print("STARTING\t");
    delay(50);

    // start the I2C library:
    Wire.begin();
    delay(50);

  /************ INIT VL53L0X DISTANCE SENSOR *****************************/
    Serial.println("VL6180X INIT");
    vl6180x.init();
    delay(500);
    vl6180x.configureDefault();
    delay(500);
    vl6180x.setTimeout(100);
    delay(100);
    vl6180x.setScaling(2); //resolution x0.5 , range x2
    delay(100);

  /************ INIT KX126 ACCELEROMETER *****************************/
    Serial.print("KX126 INIT RESPONSE WAS ");
    Serial.println(kx126.init());
    delay(200);

  /************ I/O BUTTON, LED, HAPTIC FEEDBACK *********************/
     //Configure display LED pins
    pinMode(GREEN_LED_PIN, OUTPUT); digitalWrite(GREEN_LED_PIN, 0);  

    //Configure Button Pin
    pinMode(BUTTON_PIN, INPUT);

    //Set HR LED pin high/off to conserve power
  //  pinMode(HEART_RATE_LED_PIN, OUTPUT);  digitalWrite(HEART_RATE_LED_PIN, 1);

   //configure haptic feedback pin
    pinMode(VIBRATE_PIN, OUTPUT);  digitalWrite(VIBRATE_PIN, 0);


  /************ INIT BLUETOOTH BLE instantiate BLE peripheral *********/
   // set advertised local name and service UUID
    blePeripheral.setLocalName("ChildMind");
    blePeripheral.setDeviceName("ChildMind");
    blePeripheral.setAdvertisedServiceUuid(customService.uuid());
    blePeripheral.setAppearance(0xFFFF);
  
    // add attributes (services, characteristics, descriptors) to peripheral
    blePeripheral.addAttribute(customService);
    
    blePeripheral.addAttribute(ReadOnlyArrayGattCharacteristic);
    blePeripheral.addAttribute(WriteOnlyArrayGattCharacteristic);
    
    blePeripheral.addAttribute(DataCharacteristic); //streaming data for app graph
    
    // assign event handlers for connected, disconnected to peripheral
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  //  blePeripheral.setEventHandler(BLEWritten, blePeripheralServicesDiscoveredHandler);

    // assign event handlers for characteristic
    ReadOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);
    WriteOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);

    // assign initial values
    char readValue[10] = {0,0,0,0,0,0,0,0,0,0};
    ReadOnlyArrayGattCharacteristic.setValue(0);
    char writeValue[10] = {0,0,0,0,0,0,0,0,0,0};
    WriteOnlyArrayGattCharacteristic.setValue(0);

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
  
    // begin initialization
    blePeripheral.begin();
  
    if(debug) Serial.println("BLE MOBILE APP PERIPHERAL");

    //fill NN weight array with zeros
    for(int i = 0; i < nnLength; i++){
      F[i] = 99.999;
    }

  delay(500);  
}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     
 
   /************************ LOOP SPEED CONTROL ***********************/
 if(clocktime + speedMs < millis()){
    /*************************** Timestamp ****************************/
    clocktime = millis();
    if(debug){
        Serial.println(" "); Serial.print("TIME: "); Serial.print( clocktime/1000 ); Serial.println(" s"); 
    }

    if(debug_time){ Serial.print("Time after init speed limit check: "); Serial.println(millis() - clocktime); }
    


   /******************* Bluetooth App Integration ********************/
    blePeripheral.poll(); 
   // delay(5);

    if(debug_time){ Serial.print("Time after BLE poll: "); Serial.println( (millis() - clocktime))/1000; }

   /************************* Button mgmt ****************************/
    // read the state of the pushbutton value:
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == 1 && LED_counter < 10 ) {
      // turn LED on:
      LED_counter = 80;
      digitalWrite(GREEN_LED_PIN, 1);
      greenLED_status = true;
    } 
    if (debug) { 
      Serial.print("BUTTON: ");
      Serial.println(buttonState);
    }
    

   /*************************** LED mgmt *****************************/
   //example blink program
   if(LED_counter > 0 && greenLED_status == false ){
      LED_counter--;
      if(LED_counter <= 0){
          LED_counter = 2;
          digitalWrite(GREEN_LED_PIN, 1);
          greenLED_status = true;
      }
   }
   else if(LED_counter > 0 && greenLED_status == true){
      LED_counter--;
      if(LED_counter <= 0){
          LED_counter = 30;
          digitalWrite(GREEN_LED_PIN, 0);
          greenLED_status = false;
      }
   }
  if(debug_time){ Serial.print("Time after button & LED: "); Serial.println( (millis() - clocktime))/1000; }

  /************************ Haptic Feedback mgmt *************************/
  if(vibrate_counter > 0){
      vibrate_counter--;
      if(vibrate_status == false){
          digitalWrite(VIBRATE_PIN, 1);
          vibrate_status == true;
      }
  } else if(vibrate_status == true || vibrate_counter <= 0) {
    vibrate_status == false;
    digitalWrite(VIBRATE_PIN, 0);
  }



   /******************* READ KX126 ACCELEROMETER *********************/
    //KX022 ACCELEROMETER I2C
    acc[0] = (float)(kx126.getAccel(0) * 10 );
    acc[1] = (float)(kx126.getAccel(1) * 10 );
    acc[2] = (float)(kx126.getAccel(2) * 10 );
    float eulerX, eulerY, eulerZ;
    
    eulerX = acc[0]; eulerY = acc[1]; eulerZ = acc[2]; 
    pitch = (180/3.141592) * ( atan2( eulerX, sqrt( eulerY * eulerY + eulerZ * eulerZ)) );
    roll = (180/3.141592) * ( atan2(-eulerY, -eulerZ) );

    //adjust for device upward = 180 to prevent crossover 
    pitch = pitch + 180;
    if(roll < -90 ){
      roll = 450 + roll;
      if(roll > 360){roll = roll - 360;}
    } else { roll = roll + 90; }

    if(debug_time){ Serial.print("Time after accelerometer read: "); Serial.println( (millis() - clocktime))/1000; }
    

   /************************ Thermopile mgmt **************************/
    //MLX90615 THERMOPILE SENSORS I2C CUSTOM ADDRESSES - NOT SMOOTHED!!!!
    for(int j = 0; j < 4; j++){
        TAmb[j] = readAmbientTempF(j+1); 
        TObj[j] = readObjectTempF(j+1);
    }

    TAmbAv = (TAmb[0] + TAmb[1] + TAmb[2] + TAmb[3]) / 4;

    if(debug_time){ Serial.print("Time after thermo read: "); Serial.println( (millis() - clocktime))/1000; }


        /*********************** VL6180X Distance READ **************************/    
    if(debug){ Serial.println("Reading VL6180X Distance... "); } 

    distance = 255 - (float)vl6180x.readRangeSingleMillimeters();
    if(distance < 0.1){ distance = 0; } // edge case
 //   delay(15);
    if (vl6180x.timeoutOccurred() && debug) { Serial.print(" TIMEOUT"); }
        
    if(debug_time){ Serial.print("Time after distance read: "); Serial.println( (millis() - clocktime))/1000; }

    

    if(debug){
    for(int k = 0; k < 4; k++){
      Serial.print("OT"); Serial.print(k+1); Serial.print(": ");
      Serial.print( TObj[k] );
      Serial.print("    AT"); Serial.print(k+1); Serial.print(": ");
      Serial.println( TAmb[k] );
    }

    Serial.print("TAave: "); Serial.print( TAmbAv ); Serial.println("F"); 
   
    Serial.print("ACC X: "); Serial.print( acc[0] ); Serial.println("F"); 
    Serial.print("ACC Y: "); Serial.print( acc[1] ); Serial.println("F"); 
    Serial.print("ACC Z: "); Serial.print( acc[2] ); Serial.println("F"); 
    
    Serial.print("Distance (mm): "); Serial.println(distance); 

    Serial.print("CMD: "); Serial.println(command_value);
    Serial.print("NN5: "); Serial.print(fiveInScore); Serial.print("  NN7: "); Serial.println(sevenInScore);
    }

    //Debug var state
/*    if(debug){
      Serial.print("**COMMAND: ");
      Serial.println(command_value);
    } */

/*********** Neural Network Gesture Recognition **********/
    if(selectNN != 0){
      int setting = 0; //dummy for now
      detectGesture(selectNN, setting, TObj[0], TObj[1], TObj[2], TObj[3], distance, pitch, roll);
    }
  
/*********** Bluetooth App Integration *******************/
    unsigned long microsNow;
    
    int roll_ble = roll;
    int pitch_ble = pitch;
    
    // check if it's time to read data and update the filter
    microsNow = micros();
    
    if(microsNow - microsPrevious >= microsPerReading){

          String strRoll = String(roll_ble);
          String strPitch = String(pitch_ble);
          
          if(debug){Serial.print("STRPITCH  STRROLL: "); Serial.print(strPitch); Serial.print(" "); Serial.println(strRoll);}
        
          BLECentral central = blePeripheral.central();
          
          if(central){ // if a central is connected to peripheral

              /*
               * reduce temperature readings to a range between 0 and 31, then multiply to use up the 256 max decimal value of an 8 bit integer
               * Object temperature floor: 70F
               * Object temperature ceiling: 101F
               */
              float TObj_compressed[4];
              for(int q=0; q < 4; q++){
                  TObj_compressed[q] = TObj[q];
                  if(TObj_compressed[q] < 70){ TObj_compressed[q] = 70;  }
                  else if(TObj_compressed[q] > 101){ TObj_compressed[q] = 101;  }
                  TObj_compressed[q] = (TObj_compressed[q] - 70)*8;
              }
              
              float TAmbAv_compressed;
              TAmbAv_compressed = TAmbAv;
              if(TAmbAv < 70){ TAmbAv_compressed = 70;  }
              else if(TAmbAv > 101){ TAmbAv_compressed = 101;  }
              TAmbAv_compressed = (TAmbAv_compressed - 70)*8;

              //OLD BACKWARDS COMPATABLE
              const unsigned char imuCharArray[20] = {
                  (uint8_t)(roll),  
                  (uint8_t)(pitch),
                  (uint8_t)(distance),
                  (uint8_t)(TObj_compressed[0]),  
                  (uint8_t)(TObj_compressed[1]),
                  (uint8_t)(TObj_compressed[2]),
                  (uint8_t)(TObj_compressed[3]),
                  (uint8_t)( (acc[0] + 1.00) * 100.00),               
                  (uint8_t)( (acc[1] + 1.00) * 100.00),
                  (uint8_t)( (acc[2] + 1.00) * 100.00),  
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,  
                  (uint8_t)0                   //empty
              }; 
            //  imuChar.setValue(imuCharArray, 12); //notify central with new data 
              //send data over bluetooth
              DataCharacteristic.setValue(imuCharArray,20);
              //time to send
              delay(8);
          }
  
          // increment previous time, so we keep proper pace
          microsPrevious = microsPrevious + microsPerReading;
        
     }

     if(debug_time){ Serial.print("Time after bluetooth send: "); Serial.println( (millis() - clocktime))/1000; }
  
  /*********** Bluetooth App Integration *******************/


  
    if(debug_time){ Serial.print("TIME LOOP: "); Serial.println(millis() - clocktime); }
  } //end loop speed
} //end infinate loop




int hex_to_int(char c){
  int first;
  int second;
  int value;
  
  if (c >= 97) {
    c -= 32;
  }
  first = c / 16 - 3;
  second = c % 16;
  value = first * 10 + second;
  if (value > 9) {
    value--;
  }
  return value;
}

int hex_to_ascii(char c, char d){
  int high = hex_to_int(c) * 16;
  int low = hex_to_int(d);
  return high+low;
}

void detectGesture(int selectNN, int setting, float t1, float t2, float t3, float t4, float distance, float pitch, float roll){
  float fivePrediction = 0; float sevenPrediction = 0;

  //normalize
  t1 = t1 / 101;
  t2 = t2 / 101;
  t3 = t3 / 101;
  t4 = t4 / 101;
  distance = distance / 250;
  pitch = pitch / 360;
  roll = roll / 360;
  
if(debug){ Serial.print("in detectGesture selectNN: "); Serial.print(selectNN); Serial.print("  "); Serial.print(t1); Serial.print("  "); Serial.print(distance); Serial.print("  "); Serial.println(pitch); }
  if(selectNN == 1){        //mouth --> left hand
      fivePrediction = nn_lefthand_mouth_5521(t1, t2, t3, t4, distance);
      if(fivePrediction > 50){ sevenPrediction = nn_lefthand_mouth_7521(t1, t2, t3, t4, distance, pitch, roll); }
  } else if(selectNN == 2){ //front head --> left hand
   
  } else if(selectNN == 3){ //top head --> left hand
    
  } else if(selectNN == 4){ //back head --> left hand
    
  } else if(selectNN == 5){ //right head --> left hand
    
  } else if(selectNN == 6){ //left head --> left hand
      fivePrediction = nn_lefthand_lefthead_5521(t1, t2, t3, t4, distance);
      if(fivePrediction > 50){ sevenPrediction = nn_lefthand_lefthead_7521(t1, t2, t3, t4, distance, pitch, roll); }
  } else if(selectNN == 7){
    
  }
  //for display
  fiveInScore = fivePrediction;
  sevenInScore = sevenPrediction;

  //detection algo
  if(fivePrediction > 95 && sevenPrediction > 50 || fivePrediction > 50 && sevenPrediction > 95){ flag_detect = true; } else {flag_detect = false;}
  //haptic feedback duration
  if(flag_detect){ vibrate_counter = 3; }
}


//LEFT HAND --> MOUTH
//LEFT HAND --> MOUTH
//LEFT HAND --> MOUTH
//LEFT HAND --> MOUTH
float nn_lefthand_mouth_5521(float t1, float t2, float t3, float  t4, float distance) {
  float F[500];
  /*  F[3]=input[0];
    F[5]=input[1];
    F[7]=input[2];
    F[9]=input[3];
    F[11]=input[4];
    F[13]=input[5];
    F[15]=input[6];
  */
F[3] = 0.8623762376237624;
F[5] = 0.8603960396039605;
F[7] = 0.8257425742574258;
F[9] = 0.8663366336633663;
F[11] = 0.172;
F[23] = 0.800543239992363;
F[0] = 1.3896122228097472;
F[1] = 1.389693076186595;
F[2] = 1.006891771999179;
F[4] = 0.042076599580362334;
F[6] = 0.10071046306522181;
F[8] = 0.11123700996285628;
F[10] = 0.08778965676602987;
F[12] = -0.006729002044971366;
F[13] = 0.9999939157331509;
F[14] = 0.06071444407878067;
F[15] = 0.9999952512926454;
F[16] = -0.00696508320896274;
F[17] = 0.9999958931748276;
F[18] = 0.013760984908196236;
F[19] = 0.9999946042289929;
F[20] = 0.05373476287710587;
F[21] = 0.99999849472181;
F[22] = -0.028131048175576566;
F[24] = 0;
F[190] = 12.005357527348389;
F[194] = 0.26194044414842377;
F[195] = 0.3121619420550426;
F[196] = 0.2482398329933371;
F[197] = 0.30307292513432305;
F[198] = 0.22131432178823066;
F[192] = 1;
F[125] = 1;
F[25] = 0.800543239992363;
F[26] = 0.800543239992363;
F[27] = 0.800543239992363;
F[28] = 0.800543239992363;
F[29] = 0.800543239992363;
F[43] = 0.7535608689372356;
F[30] = 1.1180150069846038;
F[31] = 1.1176947662245016;
F[32] = 1.0090871748866146;
F[33] = 0.03490161737292705;
F[34] = 0.01129653329071229;
F[35] = -0.05360293403411813;
F[36] = 0.053567796404050516;
F[37] = 0.02765145810839165;
F[38] = -0.04394797514641681;
F[39] = -0.031856519160253255;
F[40] = 0.003101845312346054;
F[41] = -0.004857541389540449;
F[42] = 0.1394481711170993;
F[44] = 0;
F[200] = 12.251914671835207;
F[204] = 0.2990452692056737;
F[205] = 0.3434787833098535;
F[206] = 0.29076301387588577;
F[207] = 0.2755312517354759;
F[208] = 0.1682038223490729;
F[202] = 1;
F[141] = 1;
F[45] = 0.7535608689372356;
F[46] = 0.7535608689372356;
F[47] = 0.7535608689372356;
F[48] = 0.7535608689372356;
F[49] = 0.7535608689372356;
F[63] = 0.7744503821658202;
F[50] = 1.2334041572214487;
F[51] = 1.2336134239810224;
F[52] = 1.0049666880182968;
F[53] = -0.0019931533436324325;
F[54] = 0.023979169266472956;
F[55] = 0.03186714309295249;
F[56] = -0.061038407569857865;
F[57] = -0.015297219538272708;
F[58] = 0.043928456530609956;
F[59] = 0.12291387277547208;
F[60] = -0.04478077566735683;
F[61] = -0.013625974315644464;
F[62] = 0.13049609211960236;
F[64] = 0;
F[210] = 12.40130464926073;
F[214] = 0.2503228191197298;
F[215] = 0.21591132399248433;
F[216] = 0.2123493741381435;
F[217] = 0.32390061075960686;
F[218] = 0.16043510957364956;
F[212] = 1;
F[157] = 1;
F[65] = 0.7744503821658202;
F[66] = 0.7744503821658202;
F[67] = 0.7744503821658202;
F[68] = 0.7744503821658202;
F[69] = 0.7744503821658202;
F[83] = 0.7615716145672538;
F[70] = 1.1629965286750445;
F[71] = 1.1613152174779096;
F[72] = 1.0051542647846032;
F[73] = 0.008952751188882549;
F[74] = 0.12962486032008017;
F[75] = -0.02569638971780705;
F[76] = 0.09733886320755855;
F[77] = 0.10778025723918325;
F[78] = 0.07534265965462161;
F[79] = 0.029715010285457998;
F[80] = -0.05900089608226091;
F[81] = -0.025774934599076257;
F[82] = -0.06501794562508313;
F[84] = 0;
F[220] = 12.12895492084727;
F[224] = 0.2582094769922371;
F[225] = 0.20284039906466617;
F[226] = 0.19367133757191152;
F[227] = 0.30698341848812405;
F[228] = 0.2425832591818956;
F[222] = 1;
F[173] = 1;
F[85] = 0.7615716145672538;
F[86] = 0.7615716145672538;
F[87] = 0.7615716145672538;
F[88] = 0.7615716145672538;
F[89] = 0.7615716145672538;
F[103] = 0.761177227066642;
F[90] = 1.159588824946637;
F[91] = 1.1591444768119497;
F[92] = 1.0116879380711703;
F[93] = -0.003908741064184865;
F[94] = -0.05003924289023334;
F[95] = 0.06032430816121952;
F[96] = -0.03405805141801654;
F[97] = 0.023555532067981083;
F[98] = 0.031532451203535365;
F[99] = 0.06679362310970433;
F[100] = 0.08547939153366196;
F[101] = 0.0314872471556316;
F[102] = -0.04576900209167118;
F[104] = 0;
F[230] = 13.396397467863395;
F[234] = 0.35061182744702135;
F[235] = 0.30121114765000123;
F[236] = 0.33680463707371383;
F[237] = 0.20143372849765473;
F[238] = 0.08541873635120809;
F[232] = 1;
F[189] = 1;
F[105] = 0.761177227066642;
F[106] = 0.761177227066642;
F[107] = 0.761177227066642;
F[108] = 0.761177227066642;
F[109] = 0.761177227066642;
F[123] = 0.9253699100601454;
F[110] = 2.520754967215745;
F[111] = 2.5176497840543424;
F[112] = 1.039114744243437;
F[113] = 0.19091889752602761;
F[114] = 0.27573277383898065;
F[115] = 0.18012015200592138;
F[116] = 0.13061173884164923;
F[117] = 0.20655830462313113;
F[118] = 0.12169928508123838;
F[119] = 0.2040034024580364;
F[120] = 0.13430400754632008;
F[121] = 0.18298751462990173;
F[122] = 0.13624634835617588;
F[124] = 0;
F[139] = 0.9341684289758098;
F[126] = 2.655027827347564;
F[127] = 2.6525572267184563;
F[128] = 1.0270162009140713;
F[129] = 0.24671582044312698;
F[130] = 0.2195390240963083;
F[131] = 0.21854141346229666;
F[132] = 0.1564163410073202;
F[133] = 0.17142508710179308;
F[134] = 0.1258110857207473;
F[135] = 0.21394213198854758;
F[136] = 0.17856144075345992;
F[137] = 0.20947571593271336;
F[138] = 0.15064889843555399;
F[140] = 0;
F[155] = 0.9322776151152227;
F[142] = 2.6252881523724;
F[143] = 2.6222138671360073;
F[144] = 1.0304871226428738;
F[145] = 0.08573543612394528;
F[146] = 0.12222159540951592;
F[147] = 0.1437057444945865;
F[148] = 0.24187500910317933;
F[149] = 0.1938633318523027;
F[150] = 0.22807170940960292;
F[151] = 0.09293422755506431;
F[152] = 0.2790138137073882;
F[153] = 0.2696874512761516;
F[154] = 0.18137545508547226;
F[156] = 0;
F[171] = 0.9319489551347803;
F[158] = 2.618816659682372;
F[159] = 2.617019960662404;
F[160] = 1.0359858387440053;
F[161] = 0.1710853383424964;
F[162] = 0.09562051824260674;
F[163] = 0.27292216249238727;
F[164] = 0.14842453481963652;
F[165] = 0.11658151537308907;
F[166] = 0.276081719349593;
F[167] = 0.29566972156196347;
F[168] = 0.11943210694527687;
F[169] = 0.1638943402976247;
F[170] = 0.12214836813321593;
F[172] = 0;
F[187] = 0.9330963551757303;
F[174] = 2.6360591232610457;
F[175] = 2.63525502275867;
F[176] = 1.0350972873834765;
F[177] = 0.1794081293757721;
F[178] = 0.17496181676761072;
F[179] = 0.10948902820935015;
F[180] = 0.14857718043060736;
F[181] = 0.06568743110407689;
F[182] = 0.2627792908201076;
F[183] = 0.11325903600300653;
F[184] = 0.20602096117627336;
F[185] = 0.26462672826988765;
F[186] = 0.21779686742101445;
F[188] = 0;
F[191] = 12.009798239456194;
F[193] = 0.09978989513961657;
F[199] = 0;
F[481] = 14.553191255770782;
F[255] = 0.9999999641164734;
F[248] = 7.9174991457008685;
F[264] = 3.856271897183459;
F[280] = 5.478426076295193;
F[296] = 4.453746636470086;
F[312] = 3.8348853902333406;
F[421] = 0.1897369922884211;
F[344] = 0.7592579444579974;
F[436] = 0.1456288954480528;
F[371] = 0.7641528707278367;
F[328] = 0.027625022515174676;
F[357] = -0.061527975387050175;
F[384] = 0.16357955935450227;
F[402] = 0.17661654594023254;
F[450] = 1.3780230808855622;
F[468] = 1.6002974453379883;
F[201] = 12.257633364109198;
F[203] = 0.012642934406829436;
F[209] = 0;
F[482] = 4.531524601904228;
F[271] = 0.7088656743041285;
F[249] = 8.099292868696537;
F[265] = 3.8569907012738214;
F[281] = 5.366718487851373;
F[297] = 4.485804365044307;
F[313] = 3.7086291592333662;
F[422] = 0.20581555882648872;
F[345] = 0.7592579444579974;
F[437] = 0.08078113333450725;
F[372] = 0.7641528707278367;
F[329] = 0.06346369986817618;
F[358] = 0.08704855944137928;
F[385] = 0.10460027753062125;
F[403] = 0.05921506704309888;
F[451] = 1.4054351967365901;
F[469] = 1.7121093189318126;
F[211] = 12.40285618528656;
F[213] = 0.15586217681058662;
F[219] = 0;
F[483] = 13.997650488843444;
F[287] = 0.0029305259945022794;
F[250] = 8.061415766866354;
F[266] = 3.8186200279895277;
F[282] = 5.3198900279542745;
F[298] = 4.478569009530704;
F[314] = 3.7114001583572236;
F[423] = 0.10675409378612512;
F[346] = 0.7592579444579974;
F[438] = 0.18113616032678334;
F[373] = 0.7641528707278367;
F[330] = -0.0060870033607904;
F[359] = 0.07491183515234578;
F[386] = 0.1516599785701996;
F[404] = 0.07934080685589973;
F[452] = 1.4883993637829074;
F[470] = 1.6534913834730618;
F[221] = 12.129889662280167;
F[223] = 0.16771965215204696;
F[229] = 0;
F[484] = 5.959953829622235;
F[303] = 0.9248604318053617;
F[251] = 8.030790281674562;
F[267] = 3.841941008946304;
F[283] = 5.423277671512031;
F[299] = 4.626618153668067;
F[315] = 3.799069316677931;
F[424] = 0.141161709325966;
F[347] = 0.7592579444579974;
F[439] = 0.08089718024484373;
F[374] = 0.7641528707278367;
F[331] = 0.08093306038695867;
F[360] = -0.036933759653623006;
F[387] = 0.14485683436950347;
F[405] = 0.1858944858227489;
F[453] = 1.431714793659721;
F[471] = 1.700215427562507;
F[231] = 13.40653132771111;
F[233] = 0.12327548227237752;
F[239] = 0;
F[485] = 4.476394627415866;
F[319] = 0.6895858071667015;
F[252] = 7.9510369961780105;
F[268] = 3.87499308112073;
F[284] = 5.312005136526212;
F[300] = 4.581816467906282;
F[316] = 3.8784971979372895;
F[425] = 0.06079937388917243;
F[348] = 0.7592579444579974;
F[440] = 0.07038080617864867;
F[375] = 0.7641528707278367;
F[332] = -0.040923802964865995;
F[361] = 0.05331818848980848;
F[388] = 0.1808198907580729;
F[406] = 0.1747254363611686;
F[454] = 1.4350187978851816;
F[472] = 1.6245420124530694;
F[253] = 0.9999999641164734;
F[240] = 16.9160037576339;
F[241] = 17.142987481504928;
F[242] = 42.21438639189773;
F[243] = -48.56038345928055;
F[244] = -17.25615368848191;
F[245] = -5.7047401435245515;
F[246] = -0.32678361643911413;
F[247] = -19.84337454169945;
F[254] = 0;
F[478] = -13.077145324021906;
F[269] = 0.7088656743041285;
F[256] = 0.8533902343224748;
F[257] = 0.8898812896232635;
F[258] = 19.79206928025723;
F[259] = -21.85958418526722;
F[260] = -15.57819816923536;
F[261] = 4.592191182629343;
F[262] = -10.136708512020693;
F[263] = -5.2702357469098455;
F[270] = 0;
F[285] = 0.0029305259945022794;
F[272] = -5.546238394380766;
F[273] = -5.82963852336367;
F[274] = 44.738001121321865;
F[275] = -30.012617827933784;
F[276] = -32.969913015241765;
F[277] = 1.2473290864511535;
F[278] = -30.753470867040416;
F[279] = 13.92091628339311;
F[286] = 0;
F[301] = 0.9248604318053617;
F[288] = 2.447693836768357;
F[289] = 2.510295548141844;
F[290] = 35.70821671927845;
F[291] = -30.544286735713424;
F[292] = -22.05156947192733;
F[293] = 4.749374182403583;
F[294] = -15.07343921798068;
F[295] = -7.986714577509054;
F[302] = 0;
F[317] = 0.6895858071667015;
F[304] = 0.7635736917021929;
F[305] = 0.7981836262850845;
F[306] = 19.448328187608272;
F[307] = -21.609186620839033;
F[308] = -15.453572567916222;
F[309] = 4.562478883049235;
F[310] = -9.85520203823745;
F[311] = -5.1198376136838695;
F[318] = 0;
F[337] = 0.7592579444579974;
F[320] = 1.148799619350778;
F[321] = 1.1486155160845541;
F[322] = 1.0038961008385794;
F[323] = 0.014007271458826574;
F[324] = 0.09520812418413382;
F[325] = -0.05169070308100931;
F[326] = -0.061049183057756404;
F[327] = 0.01643959816064021;
F[333] = 0.9999750002894632;
F[334] = -0.08066227118694204;
F[335] = 0.9999671007585722;
F[336] = 0.0991212967201327;
F[338] = 0;
F[412] = 10.59259776898992;
F[416] = 0.06833959807740991;
F[417] = 0.10544815269788087;
F[418] = 0.14746278100139035;
F[419] = 0.1872969867779849;
F[420] = 0.10210109535818578;
F[414] = 1;
F[393] = 1;
F[339] = 0.7592579444579974;
F[340] = 0.7592579444579974;
F[341] = 0.7592579444579974;
F[342] = 0.7592579444579974;
F[343] = 0.7592579444579974;
F[364] = 0.7641528707278367;
F[349] = 1.176675804299;
F[350] = 1.17558402390752;
F[351] = 1.0034950773252285;
F[352] = -0.05760042802146631;
F[353] = 0.056030042146254594;
F[354] = -0.027362938820588923;
F[355] = 0.09832565290813486;
F[356] = 0.06280046819338041;
F[362] = 0.02596292067958049;
F[363] = -0.0426161893133128;
F[365] = 0;
F[427] = 10.317499983974036;
F[431] = 0.22054326150389256;
F[432] = 0.12723808362214348;
F[433] = 0.21068857843725852;
F[434] = 0.0835899951958959;
F[435] = 0.18334934542280962;
F[429] = 1;
F[411] = 1;
F[366] = 0.7641528707278367;
F[367] = 0.7641528707278367;
F[368] = 0.7641528707278367;
F[369] = 0.7641528707278367;
F[370] = 0.7641528707278367;
F[391] = 0.9037553422879941;
F[376] = 2.2397982091762896;
F[377] = 2.239665217364238;
F[378] = 1.0216054528566236;
F[379] = 0.12408923194526031;
F[380] = 0.06300421940214339;
F[381] = 0.034544050070213786;
F[382] = 0.12750547026255263;
F[383] = 0.02153452948755665;
F[389] = 0.12379652639821627;
F[390] = 0.04484317789052631;
F[392] = 0;
F[409] = 0.9119741775198591;
F[394] = 2.3378554227527264;
F[395] = 2.33798146684228;
F[396] = 1.017475394534579;
F[397] = 0.17061018720156768;
F[398] = 0.16439121749095045;
F[399] = 0.15455376096305712;
F[400] = 0.04838910292514365;
F[401] = 0.0036767589084844023;
F[407] = 0.023012606240136475;
F[408] = 0.16296303571465293;
F[410] = 0;
F[413] = 10.596621311668432;
F[415] = 0.14616990095063162;
F[426] = 0;
F[486] = 2.9905350352022775;
F[459] = 0.13122159797816055;
F[455] = 1.3721022887703946;
F[473] = 1.6741361916293016;
F[428] = 10.322028057583742;
F[430] = 0.04433915898721529;
F[441] = 0;
F[487] = 3.1414019982094685;
F[477] = 0.16136583035630003;
F[456] = 1.4618476489477623;
F[474] = 1.7402606311062605;
F[457] = 0.13122159797816055;
F[442] = -1.8882060211696365;
F[443] = -1.8902006076215863;
F[444] = 8.780048561011641;
F[445] = -11.820689649338354;
F[446] = -8.122218407545521;
F[447] = 1.4821014774789671;
F[448] = -5.123121970332265;
F[449] = -1.4299797583310219;
F[458] = 0;
F[475] = 0.16136583035630003;
F[460] = -1.647654310617783;
F[461] = -1.648100554370955;
F[462] = 9.714077052115364;
F[463] = -13.293186112714833;
F[464] = -9.009457150316296;
F[465] = 1.7438551683053503;
F[466] = -5.765552774582809;
F[467] = -1.7251827368645438;
F[476] = 0;
F[493] = 0.0000014999919318759745;
F[479] = -13.410049328626869;
F[480] = 20.8352816247334;
F[488] = -49.6357921146332;
F[489] = -26.35575213659238;
F[490] = 14.735644101746335;
F[491] = -13.232031435937113;
F[492] = 18.759978631471807;
F[494] = 0;

  F[3] = t1;
  F[5] = t2;
  F[7] = t3;
  F[9] = t4;
  F[11] = distance;

F[0] = F[1];F[1] = F[2];F[1] += F[3] * F[4];F[1] += F[5] * F[6];F[1] += F[7] * F[8];F[1] += F[9] * F[10];F[1] += F[11] * F[12];F[1] += F[13] * F[14];F[1] += F[15] * F[16];F[1] += F[17] * F[18];F[1] += F[19] * F[20];F[1] += F[21] * F[22];F[23] = (1 / (1 + exp(-F[1])));F[24] = F[23] * (1 - F[23]);F[25] = F[23];F[26] = F[23];F[27] = F[23];F[28] = F[23];F[29] = F[23];
F[30] = F[31];F[31] = F[32];F[31] += F[3] * F[33];F[31] += F[5] * F[34];F[31] += F[7] * F[35];F[31] += F[9] * F[36];F[31] += F[11] * F[37];F[31] += F[13] * F[38];F[31] += F[15] * F[39];F[31] += F[17] * F[40];F[31] += F[19] * F[41];F[31] += F[21] * F[42];F[43] = (1 / (1 + exp(-F[31])));F[44] = F[43] * (1 - F[43]);F[45] = F[43];F[46] = F[43];F[47] = F[43];F[48] = F[43];F[49] = F[43];
F[50] = F[51];F[51] = F[52];F[51] += F[3] * F[53];F[51] += F[5] * F[54];F[51] += F[7] * F[55];F[51] += F[9] * F[56];F[51] += F[11] * F[57];F[51] += F[13] * F[58];F[51] += F[15] * F[59];F[51] += F[17] * F[60];F[51] += F[19] * F[61];F[51] += F[21] * F[62];F[63] = (1 / (1 + exp(-F[51])));F[64] = F[63] * (1 - F[63]);F[65] = F[63];F[66] = F[63];F[67] = F[63];F[68] = F[63];F[69] = F[63];
F[70] = F[71];F[71] = F[72];F[71] += F[3] * F[73];F[71] += F[5] * F[74];F[71] += F[7] * F[75];F[71] += F[9] * F[76];F[71] += F[11] * F[77];F[71] += F[13] * F[78];F[71] += F[15] * F[79];F[71] += F[17] * F[80];F[71] += F[19] * F[81];F[71] += F[21] * F[82];F[83] = (1 / (1 + exp(-F[71])));F[84] = F[83] * (1 - F[83]);F[85] = F[83];F[86] = F[83];F[87] = F[83];F[88] = F[83];F[89] = F[83];
F[90] = F[91];F[91] = F[92];F[91] += F[3] * F[93];F[91] += F[5] * F[94];F[91] += F[7] * F[95];F[91] += F[9] * F[96];F[91] += F[11] * F[97];F[91] += F[13] * F[98];F[91] += F[15] * F[99];F[91] += F[17] * F[100];F[91] += F[19] * F[101];F[91] += F[21] * F[102];F[103] = (1 / (1 + exp(-F[91])));F[104] = F[103] * (1 - F[103]);F[105] = F[103];F[106] = F[103];F[107] = F[103];F[108] = F[103];F[109] = F[103];
F[110] = F[111];F[111] = F[112];F[111] += F[3] * F[113];F[111] += F[5] * F[114];F[111] += F[7] * F[115];F[111] += F[9] * F[116];F[111] += F[11] * F[117];F[111] += F[13] * F[118];F[111] += F[15] * F[119];F[111] += F[17] * F[120];F[111] += F[19] * F[121];F[111] += F[21] * F[122];F[123] = (1 / (1 + exp(-F[111])));F[124] = F[123] * (1 - F[123]);F[125] = F[123];
F[126] = F[127];F[127] = F[128];F[127] += F[3] * F[129];F[127] += F[5] * F[130];F[127] += F[7] * F[131];F[127] += F[9] * F[132];F[127] += F[11] * F[133];F[127] += F[13] * F[134];F[127] += F[15] * F[135];F[127] += F[17] * F[136];F[127] += F[19] * F[137];F[127] += F[21] * F[138];F[139] = (1 / (1 + exp(-F[127])));F[140] = F[139] * (1 - F[139]);F[141] = F[139];
F[142] = F[143];F[143] = F[144];F[143] += F[3] * F[145];F[143] += F[5] * F[146];F[143] += F[7] * F[147];F[143] += F[9] * F[148];F[143] += F[11] * F[149];F[143] += F[13] * F[150];F[143] += F[15] * F[151];F[143] += F[17] * F[152];F[143] += F[19] * F[153];F[143] += F[21] * F[154];F[155] = (1 / (1 + exp(-F[143])));F[156] = F[155] * (1 - F[155]);F[157] = F[155];
F[158] = F[159];F[159] = F[160];F[159] += F[3] * F[161];F[159] += F[5] * F[162];F[159] += F[7] * F[163];F[159] += F[9] * F[164];F[159] += F[11] * F[165];F[159] += F[13] * F[166];F[159] += F[15] * F[167];F[159] += F[17] * F[168];F[159] += F[19] * F[169];F[159] += F[21] * F[170];F[171] = (1 / (1 + exp(-F[159])));F[172] = F[171] * (1 - F[171]);F[173] = F[171];
F[174] = F[175];F[175] = F[176];F[175] += F[3] * F[177];F[175] += F[5] * F[178];F[175] += F[7] * F[179];F[175] += F[9] * F[180];F[175] += F[11] * F[181];F[175] += F[13] * F[182];F[175] += F[15] * F[183];F[175] += F[17] * F[184];F[175] += F[19] * F[185];F[175] += F[21] * F[186];F[187] = (1 / (1 + exp(-F[175])));F[188] = F[187] * (1 - F[187]);F[189] = F[187];
F[190] = F[191];F[191] = F[125] * F[192] * F[191] + F[193];F[191] += F[3] * F[194] * F[25];F[191] += F[5] * F[195] * F[26];F[191] += F[7] * F[196] * F[27];F[191] += F[9] * F[197] * F[28];F[191] += F[11] * F[198] * F[29];F[13] = (1 / (1 + exp(-F[191])));F[199] = F[13] * (1 - F[13]);
F[200] = F[201];F[201] = F[141] * F[202] * F[201] + F[203];F[201] += F[3] * F[204] * F[45];F[201] += F[5] * F[205] * F[46];F[201] += F[7] * F[206] * F[47];F[201] += F[9] * F[207] * F[48];F[201] += F[11] * F[208] * F[49];F[15] = (1 / (1 + exp(-F[201])));F[209] = F[15] * (1 - F[15]);
F[210] = F[211];F[211] = F[157] * F[212] * F[211] + F[213];F[211] += F[3] * F[214] * F[65];F[211] += F[5] * F[215] * F[66];F[211] += F[7] * F[216] * F[67];F[211] += F[9] * F[217] * F[68];F[211] += F[11] * F[218] * F[69];F[17] = (1 / (1 + exp(-F[211])));F[219] = F[17] * (1 - F[17]);
F[220] = F[221];F[221] = F[173] * F[222] * F[221] + F[223];F[221] += F[3] * F[224] * F[85];F[221] += F[5] * F[225] * F[86];F[221] += F[7] * F[226] * F[87];F[221] += F[9] * F[227] * F[88];F[221] += F[11] * F[228] * F[89];F[19] = (1 / (1 + exp(-F[221])));F[229] = F[19] * (1 - F[19]);
F[230] = F[231];F[231] = F[189] * F[232] * F[231] + F[233];F[231] += F[3] * F[234] * F[105];F[231] += F[5] * F[235] * F[106];F[231] += F[7] * F[236] * F[107];F[231] += F[9] * F[237] * F[108];F[231] += F[11] * F[238] * F[109];F[21] = (1 / (1 + exp(-F[231])));F[239] = F[21] * (1 - F[21]);
F[240] = F[241];F[241] = F[242];F[241] += F[3] * F[243];F[241] += F[5] * F[244];F[241] += F[7] * F[245];F[241] += F[9] * F[246];F[241] += F[11] * F[247];F[241] += F[13] * F[248];F[241] += F[15] * F[249];F[241] += F[17] * F[250];F[241] += F[19] * F[251];F[241] += F[21] * F[252];F[253] = (1 / (1 + exp(-F[241])));F[254] = F[253] * (1 - F[253]);F[255] = F[253];
F[256] = F[257];F[257] = F[258];F[257] += F[3] * F[259];F[257] += F[5] * F[260];F[257] += F[7] * F[261];F[257] += F[9] * F[262];F[257] += F[11] * F[263];F[257] += F[13] * F[264];F[257] += F[15] * F[265];F[257] += F[17] * F[266];F[257] += F[19] * F[267];F[257] += F[21] * F[268];F[269] = (1 / (1 + exp(-F[257])));F[270] = F[269] * (1 - F[269]);F[271] = F[269];
F[272] = F[273];F[273] = F[274];F[273] += F[3] * F[275];F[273] += F[5] * F[276];F[273] += F[7] * F[277];F[273] += F[9] * F[278];F[273] += F[11] * F[279];F[273] += F[13] * F[280];F[273] += F[15] * F[281];F[273] += F[17] * F[282];F[273] += F[19] * F[283];F[273] += F[21] * F[284];F[285] = (1 / (1 + exp(-F[273])));F[286] = F[285] * (1 - F[285]);F[287] = F[285];
F[288] = F[289];F[289] = F[290];F[289] += F[3] * F[291];F[289] += F[5] * F[292];F[289] += F[7] * F[293];F[289] += F[9] * F[294];F[289] += F[11] * F[295];F[289] += F[13] * F[296];F[289] += F[15] * F[297];F[289] += F[17] * F[298];F[289] += F[19] * F[299];F[289] += F[21] * F[300];F[301] = (1 / (1 + exp(-F[289])));F[302] = F[301] * (1 - F[301]);F[303] = F[301];
F[304] = F[305];F[305] = F[306];F[305] += F[3] * F[307];F[305] += F[5] * F[308];F[305] += F[7] * F[309];F[305] += F[9] * F[310];F[305] += F[11] * F[311];F[305] += F[13] * F[312];F[305] += F[15] * F[313];F[305] += F[17] * F[314];F[305] += F[19] * F[315];F[305] += F[21] * F[316];F[317] = (1 / (1 + exp(-F[305])));F[318] = F[317] * (1 - F[317]);F[319] = F[317];
F[320] = F[321];F[321] = F[322];F[321] += F[3] * F[323];F[321] += F[5] * F[324];F[321] += F[7] * F[325];F[321] += F[9] * F[326];F[321] += F[11] * F[327];F[321] += F[13] * F[328];F[321] += F[15] * F[329];F[321] += F[17] * F[330];F[321] += F[19] * F[331];F[321] += F[21] * F[332];F[321] += F[333] * F[334];F[321] += F[335] * F[336];F[337] = (1 / (1 + exp(-F[321])));F[338] = F[337] * (1 - F[337]);F[339] = F[337];F[340] = F[337];F[341] = F[337];F[342] = F[337];F[343] = F[337];F[344] = F[337];F[345] = F[337];F[346] = F[337];F[347] = F[337];F[348] = F[337];
F[349] = F[350];F[350] = F[351];F[350] += F[3] * F[352];F[350] += F[5] * F[353];F[350] += F[7] * F[354];F[350] += F[9] * F[355];F[350] += F[11] * F[356];F[350] += F[13] * F[357];F[350] += F[15] * F[358];F[350] += F[17] * F[359];F[350] += F[19] * F[360];F[350] += F[21] * F[361];F[350] += F[333] * F[362];F[350] += F[335] * F[363];F[364] = (1 / (1 + exp(-F[350])));F[365] = F[364] * (1 - F[364]);F[366] = F[364];F[367] = F[364];F[368] = F[364];F[369] = F[364];F[370] = F[364];F[371] = F[364];F[372] = F[364];F[373] = F[364];F[374] = F[364];F[375] = F[364];
F[376] = F[377];F[377] = F[378];F[377] += F[3] * F[379];F[377] += F[5] * F[380];F[377] += F[7] * F[381];F[377] += F[9] * F[382];F[377] += F[11] * F[383];F[377] += F[13] * F[384];F[377] += F[15] * F[385];F[377] += F[17] * F[386];F[377] += F[19] * F[387];F[377] += F[21] * F[388];F[377] += F[333] * F[389];F[377] += F[335] * F[390];F[391] = (1 / (1 + exp(-F[377])));F[392] = F[391] * (1 - F[391]);F[393] = F[391];
F[394] = F[395];F[395] = F[396];F[395] += F[3] * F[397];F[395] += F[5] * F[398];F[395] += F[7] * F[399];F[395] += F[9] * F[400];F[395] += F[11] * F[401];F[395] += F[13] * F[402];F[395] += F[15] * F[403];F[395] += F[17] * F[404];F[395] += F[19] * F[405];F[395] += F[21] * F[406];F[395] += F[333] * F[407];F[395] += F[335] * F[408];F[409] = (1 / (1 + exp(-F[395])));F[410] = F[409] * (1 - F[409]);F[411] = F[409];
F[412] = F[413];F[413] = F[393] * F[414] * F[413] + F[415];F[413] += F[3] * F[416] * F[339];F[413] += F[5] * F[417] * F[340];F[413] += F[7] * F[418] * F[341];F[413] += F[9] * F[419] * F[342];F[413] += F[11] * F[420] * F[343];F[413] += F[13] * F[421] * F[344];F[413] += F[15] * F[422] * F[345];F[413] += F[17] * F[423] * F[346];F[413] += F[19] * F[424] * F[347];F[413] += F[21] * F[425] * F[348];F[333] = (1 / (1 + exp(-F[413])));F[426] = F[333] * (1 - F[333]);
F[427] = F[428];F[428] = F[411] * F[429] * F[428] + F[430];F[428] += F[3] * F[431] * F[366];F[428] += F[5] * F[432] * F[367];F[428] += F[7] * F[433] * F[368];F[428] += F[9] * F[434] * F[369];F[428] += F[11] * F[435] * F[370];F[428] += F[13] * F[436] * F[371];F[428] += F[15] * F[437] * F[372];F[428] += F[17] * F[438] * F[373];F[428] += F[19] * F[439] * F[374];F[428] += F[21] * F[440] * F[375];F[335] = (1 / (1 + exp(-F[428])));F[441] = F[335] * (1 - F[335]);
F[442] = F[443];F[443] = F[444];F[443] += F[3] * F[445];F[443] += F[5] * F[446];F[443] += F[7] * F[447];F[443] += F[9] * F[448];F[443] += F[11] * F[449];F[443] += F[13] * F[450];F[443] += F[15] * F[451];F[443] += F[17] * F[452];F[443] += F[19] * F[453];F[443] += F[21] * F[454];F[443] += F[333] * F[455];F[443] += F[335] * F[456];F[457] = (1 / (1 + exp(-F[443])));F[458] = F[457] * (1 - F[457]);F[459] = F[457];
F[460] = F[461];F[461] = F[462];F[461] += F[3] * F[463];F[461] += F[5] * F[464];F[461] += F[7] * F[465];F[461] += F[9] * F[466];F[461] += F[11] * F[467];F[461] += F[13] * F[468];F[461] += F[15] * F[469];F[461] += F[17] * F[470];F[461] += F[19] * F[471];F[461] += F[21] * F[472];F[461] += F[333] * F[473];F[461] += F[335] * F[474];F[475] = (1 / (1 + exp(-F[461])));F[476] = F[475] * (1 - F[475]);F[477] = F[475];
F[478] = F[479];F[479] = F[480];F[479] += F[13] * F[481] * F[255];F[479] += F[15] * F[482] * F[271];F[479] += F[17] * F[483] * F[287];F[479] += F[19] * F[484] * F[303];F[479] += F[21] * F[485] * F[319];F[479] += F[333] * F[486] * F[459];F[479] += F[335] * F[487] * F[477];F[479] += F[3] * F[488];F[479] += F[5] * F[489];F[479] += F[7] * F[490];F[479] += F[9] * F[491];F[479] += F[11] * F[492];F[493] = (1 / (1 + exp(-F[479])));F[494] = F[493] * (1 - F[493]);

float output = F[493] * 100;
return output;
}

//LEFT HAND --> MOUTH
//LEFT HAND --> MOUTH
//LEFT HAND --> MOUTH
//LEFT HAND --> MOUTH
float nn_lefthand_mouth_7521(float t1, float t2, float t3, float  t4, float distance, float pitch, float roll) {
  float F[] = {0,0,0};
  /*  F[3]=input[0];
    F[5]=input[1];
    F[7]=input[2];
    F[9]=input[3];
    F[11]=input[4];
    F[13]=input[5];
    F[15]=input[6];
  */
  F[3] =  0.8217821782178217;
  F[5] =  0.8455445544554456;
  F[7] =  0.8415841584158416;
  F[9] =  0.8465346534653465;
  F[11] =  0;
  F[13] =  0.5166666666666667;
  F[15] =  0.6861111111111111;
  F[27] =  0.7668277792523295;
  F[0] =  1.1356204334669315;
  F[1] =  1.190484912649766;
  F[2] =  1.005815646013983;
  F[4] =  -0.054669033928732;
  F[6] =  -0.05112511816407372;
  F[8] =  -0.05747710988364215;
  F[10] =  0.052146154555318125;
  F[12] =  -0.05661663714181266;
  F[14] =  -0.034211355231180456;
  F[16] =  0.023977494335705018;
  F[17] =  0.9999989120898729;
  F[18] =  0.03307801931089955;
  F[19] =  0.9999988755653926;
  F[20] =  0.10530862236046708;
  F[21] =  0.9999988910161212;
  F[22] =  0.12913207472739852;
  F[23] =  0.9999987591879234;
  F[24] =  -0.020857414275839693;
  F[25] =  0.9999989565361637;
  F[26] =  0.031615754996683475;
  F[28] =  0;
  F[222] =  13.947918808676418;
  F[226] =  0.29713475710583737;
  F[227] =  0.21799442674848887;
  F[228] =  0.2516236222254197;
  F[229] =  0.18259416678243867;
  F[230] =  0.24539570406042174;
  F[231] =  0.10031168590165089;
  F[232] =  0.05314966177860108;
  F[224] =  1;
  F[149] =  1;
  F[29] =  0.7668277792523295;
  F[30] =  0.7668277792523295;
  F[31] =  0.7668277792523295;
  F[32] =  0.7668277792523295;
  F[33] =  0.7668277792523295;
  F[34] =  0.7668277792523295;
  F[35] =  0.7668277792523295;
  F[51] =  0.8024099035219964;
  F[36] =  1.3559826091552276;
  F[37] =  1.4014249139276054;
  F[38] =  1.0065088990010773;
  F[39] =  -0.015955314321239977;
  F[40] =  -0.04617094598370222;
  F[41] =  0.07878989680764044;
  F[42] =  0.012130268537332463;
  F[43] =  -0.06051236310773319;
  F[44] =  0.09205262471967682;
  F[45] =  0.08057601616423149;
  F[46] =  0.038015520308438924;
  F[47] =  0.10364475190105732;
  F[48] =  0.04087180601609509;
  F[49] =  0.07222535268181306;
  F[50] =  0.01288857334586362;
  F[52] =  0;
  F[234] =  13.95620607828709;
  F[238] =  0.18326191903180797;
  F[239] =  0.2694385233024666;
  F[240] =  0.31381735787774895;
  F[241] =  0.2727450172261778;
  F[242] =  0.2751310924231197;
  F[243] =  0.15323778781671624;
  F[244] =  0.042314911633773426;
  F[236] =  1;
  F[167] =  1;
  F[53] =  0.8024099035219964;
  F[54] =  0.8024099035219964;
  F[55] =  0.8024099035219964;
  F[56] =  0.8024099035219964;
  F[57] =  0.8024099035219964;
  F[58] =  0.8024099035219964;
  F[59] =  0.8024099035219964;
  F[75] =  0.8357233329406689;
  F[60] =  1.6517979285712634;
  F[61] =  1.6267456161410292;
  F[62] =  1.004715090725226;
  F[63] =  0.10515616227589228;
  F[64] =  0.022066504125856444;
  F[65] =  0.11937802912919356;
  F[66] =  0.09105423313031835;
  F[67] =  0.02633062390547713;
  F[68] =  0.05485674236069093;
  F[69] =  0.03796667852675145;
  F[70] =  0.002067512704120234;
  F[71] =  0.026980582602569924;
  F[72] =  0.11457240507996193;
  F[73] =  0.047776582435606056;
  F[74] =  0.0936208090351191;
  F[76] =  0;
  F[246] =  13.903491658848226;
  F[250] =  0.2915497992907654;
  F[251] =  0.1655995019680365;
  F[252] =  0.3190249531044697;
  F[253] =  0.3327111100899978;
  F[254] =  0.16541808275397926;
  F[255] =  0.07180146068591384;
  F[256] =  0.040573567831441656;
  F[248] =  1;
  F[185] =  1;
  F[77] =  0.8357233329406689;
  F[78] =  0.8357233329406689;
  F[79] =  0.8357233329406689;
  F[80] =  0.8357233329406689;
  F[81] =  0.8357233329406689;
  F[82] =  0.8357233329406689;
  F[83] =  0.8357233329406689;
  F[99] =  0.7982969742011771;
  F[84] =  1.403570450971083;
  F[85] =  1.3756842300124812;
  F[86] =  1.006304562624868;
  F[87] =  0.011882514214784248;
  F[88] =  0.11393899129160952;
  F[89] =  0.09631821307226518;
  F[90] =  0.010323078912766955;
  F[91] =  0.027817878196827225;
  F[92] =  0.09933758292507434;
  F[93] =  0.05990580025915235;
  F[94] =  -0.006872228148289192;
  F[95] =  0.08744062314634304;
  F[96] =  0.06116622680596375;
  F[97] =  0.005025878868428252;
  F[98] =  -0.06571127721905955;
  F[100] =  0;
  F[258] =  13.770363218231388;
  F[262] =  0.24048859381312584;
  F[263] =  0.24250987835083435;
  F[264] =  0.2903251974553898;
  F[265] =  0.2682571034697624;
  F[266] =  0.20520439022224823;
  F[267] =  0.12963790548462623;
  F[268] =  0.13836999010206533;
  F[260] =  1;
  F[203] =  1;
  F[101] =  0.7982969742011771;
  F[102] =  0.7982969742011771;
  F[103] =  0.7982969742011771;
  F[104] =  0.7982969742011771;
  F[105] =  0.7982969742011771;
  F[106] =  0.7982969742011771;
  F[107] =  0.7982969742011771;
  F[123] =  0.8197920174944535;
  F[108] =  1.4726587728858704;
  F[109] =  1.51493902865895;
  F[110] =  1.0040626148542018;
  F[111] =  -0.07100539817873866;
  F[112] =  0.09544825066982734;
  F[113] =  0.09091166598445288;
  F[114] =  0.058324300559690956;
  F[115] =  -0.07286407904018705;
  F[116] =  0.09005490731976348;
  F[117] =  0.01911578642800002;
  F[118] =  0.08986137386980136;
  F[119] =  0.03787525452374107;
  F[120] =  -0.018157201966826776;
  F[121] =  0.10179481387682182;
  F[122] =  0.09162040437722366;
  F[124] =  0;
  F[270] =  13.845521923991528;
  F[274] =  0.16773873704007392;
  F[275] =  0.29633428062754436;
  F[276] =  0.2905666703354708;
  F[277] =  0.29084632809579397;
  F[278] =  0.06538220348630908;
  F[279] =  0.06123732912475433;
  F[280] =  0.053235994649348695;
  F[272] =  1;
  F[221] =  1;
  F[125] =  0.8197920174944535;
  F[126] =  0.8197920174944535;
  F[127] =  0.8197920174944535;
  F[128] =  0.8197920174944535;
  F[129] =  0.8197920174944535;
  F[130] =  0.8197920174944535;
  F[131] =  0.8197920174944535;
  F[147] =  0.9258288481454816;
  F[132] =  2.7021279715490762;
  F[133] =  2.524314102793987;
  F[134] =  1.0318198915642278;
  F[135] =  0.15768373575580252;
  F[136] =  0.12428838026097558;
  F[137] =  0.11797961400899956;
  F[138] =  0.10296262263842546;
  F[139] =  0.2073332042323846;
  F[140] =  0.14176497371260166;
  F[141] =  0.04365437224892618;
  F[142] =  0.13603580991868403;
  F[143] =  0.22964087972255745;
  F[144] =  0.18800606772936834;
  F[145] =  0.16705675672712322;
  F[146] =  0.24743436739693578;
  F[148] =  0;
  F[165] =  0.9262132226256294;
  F[150] =  2.7248055126789255;
  F[151] =  2.5299249229697747;
  F[152] =  1.026241375733733;
  F[153] =  0.2052306754335851;
  F[154] =  0.10001436669470866;
  F[155] =  0.23397439679121848;
  F[156] =  0.19990434444448849;
  F[157] =  0.20610321627537173;
  F[158] =  0.15728483047324762;
  F[159] =  -0.013563991076970795;
  F[160] =  0.1618584860953797;
  F[161] =  0.11185919782751894;
  F[162] =  0.16743752530275055;
  F[163] =  0.2198605461126659;
  F[164] =  0.1513545216107466;
  F[166] =  0;
  F[183] =  0.9232518979103757;
  F[168] =  2.6381593612415988;
  F[169] =  2.4873734517270885;
  F[170] =  1.026650123191874;
  F[171] =  0.1820957791011779;
  F[172] =  0.23366904353303786;
  F[173] =  0.11449267764097425;
  F[174] =  0.1733787953395256;
  F[175] =  0.17193072771795428;
  F[176] =  0.10466348305812814;
  F[177] =  0.012009204032081371;
  F[178] =  0.13988458799948333;
  F[179] =  0.2433616800155868;
  F[180] =  0.1793346765876218;
  F[181] =  0.09067341574770181;
  F[182] =  0.1548068947190119;
  F[184] =  0;
  F[201] =  0.9268879719592062;
  F[186] =  2.6857604656770206;
  F[187] =  2.539839812663095;
  F[188] =  1.0235682624656346;
  F[189] =  0.19693007487857447;
  F[190] =  0.205692217650604;
  F[191] =  0.05808855646194599;
  F[192] =  0.16307009180865245;
  F[193] =  0.1701611386750733;
  F[194] =  0.13530893973154745;
  F[195] =  0.04753484040143934;
  F[196] =  0.18728459279076184;
  F[197] =  0.21478317767052624;
  F[198] =  0.0928933547702839;
  F[199] =  0.17855276802562725;
  F[200] =  0.21754823656428768;
  F[202] =  0;
  F[219] =  0.9313533710440998;
  F[204] =  2.711199004831613;
  F[205] =  2.6076667397824917;
  F[206] =  1.0207714547319966;
  F[207] =  0.21402118259715536;
  F[208] =  0.2261572249747751;
  F[209] =  0.10197343021328362;
  F[210] =  0.06417185231836162;
  F[211] =  0.11536608992286297;
  F[212] =  0.17605163190369433;
  F[213] =  0.08112726266250427;
  F[214] =  0.1674358722268347;
  F[215] =  0.11927462222255161;
  F[216] =  0.23329382089369638;
  F[217] =  0.19004365433162174;
  F[218] =  0.2229781445113637;
  F[220] =  0;
  F[223] =  13.731250928905691;
  F[225] =  0.14065435316752445;
  F[233] =  0;
  F[553] =  10.292543311201534;
  F[299] =  0.02723302458106075;
  F[292] =  -0.9224129052648475;
  F[310] =  1.5564801696873873;
  F[328] =  0.9602426915770393;
  F[346] =  0.4792463316579865;
  F[364] =  -0.19457620804928843;
  F[487] =  0.09328880237322489;
  F[400] =  0.8092688668662942;
  F[504] =  0.1828398704577991;
  F[431] =  0.7325214115332304;
  F[382] =  0.026654575572851964;
  F[415] =  -0.07121070765932971;
  F[446] =  0.04077645468799239;
  F[466] =  0.18150427227629895;
  F[520] =  0.40469200812193185;
  F[540] =  -0.2594815951762941;
  F[235] =  13.698229095472866;
  F[237] =  -0.015855979108662308;
  F[245] =  0;
  F[554] =  15.32753628907582;
  F[317] =  0.008831665685987644;
  F[293] =  -0.9671852192574543;
  F[311] =  1.5445656238432868;
  F[329] =  0.9877642095600812;
  F[347] =  0.5057801304250092;
  F[365] =  -0.20920994631943968;
  F[488] =  0.033547670286046855;
  F[401] =  0.8092688668662942;
  F[505] =  0.16184543551918643;
  F[432] =  0.7325214115332304;
  F[383] =  0.050266619596715835;
  F[416] =  -0.014000946721965167;
  F[447] =  0.04476228596763926;
  F[467] =  0.13237509232000647;
  F[521] =  0.30402594968786717;
  F[541] =  -0.32141332428276337;
  F[247] =  13.712065277466644;
  F[249] =  0.04435840622530814;
  F[257] =  0;
  F[555] =  10.723051574659593;
  F[335] =  0.7456617360768952;
  F[294] =  -0.9304645297087987;
  F[312] =  1.6396842804086564;
  F[330] =  0.8688069113521044;
  F[348] =  0.5847352431992665;
  F[366] =  -0.1738731591058989;
  F[489] =  0.136314433850692;
  F[402] =  0.8092688668662942;
  F[506] =  0.18432870871336912;
  F[433] =  0.7325214115332304;
  F[384] =  0.10396731580474693;
  F[417] =  0.008572034359071794;
  F[448] =  0.0877445403759795;
  F[468] =  0.1692085954433219;
  F[522] =  0.3790672329514624;
  F[542] =  -0.29098463343802944;
  F[259] =  13.599743251358586;
  F[261] =  0.009106832661070407;
  F[269] =  0;
  F[556] =  5.44906323981989;
  F[353] =  0.0026438860781303167;
  F[295] =  -0.8857821247155352;
  F[313] =  1.522777208418866;
  F[331] =  0.8731888180142107;
  F[349] =  0.5333711416996991;
  F[367] =  -0.15245284599707853;
  F[490] =  0.2175577950423004;
  F[403] =  0.8092688668662942;
  F[507] =  0.12258930269102888;
  F[434] =  0.7325214115332304;
  F[385] =  0.018762800662893188;
  F[418] =  -0.007587644038395767;
  F[449] =  0.008854900790191892;
  F[469] =  0.03329252593241696;
  F[523] =  0.3673137274600038;
  F[543] =  -0.26399862694604476;
  F[271] =  13.772963723727205;
  F[273] =  0.10128365273516783;
  F[281] =  0;
  F[557] =  1.8609940683990402;
  F[371] =  0.07350952920511876;
  F[296] =  -0.8208594057190759;
  F[314] =  1.6275546446471798;
  F[332] =  0.862744257457262;
  F[350] =  0.5674954090830207;
  F[368] =  -0.29817876498946627;
  F[491] =  0.12052563883449649;
  F[404] =  0.8092688668662942;
  F[508] =  0.12384553208390725;
  F[435] =  0.7325214115332304;
  F[386] =  0.028275644619322252;
  F[419] =  -0.00736343835748058;
  F[450] =  0.08230960994463311;
  F[470] =  -0.006209915498554604;
  F[524] =  0.3958958287030825;
  F[544] =  -0.14428940775112115;
  F[297] =  0.02723302458106075;
  F[282] =  3.7939833738145747;
  F[283] =  -3.5757141868291;
  F[284] =  25.132299296128256;
  F[285] =  -36.821653205870575;
  F[286] =  -25.64749012738212;
  F[287] =  51.43234197136686;
  F[288] =  -33.53899959808387;
  F[289] =  2.5893045095896365;
  F[290] =  24.956197957778066;
  F[291] =  -0.03293180521971344;
  F[298] =  0;
  F[550] =  4.104640383964244;
  F[315] =  0.008831665685987644;
  F[300] =  -2.7796089903287005;
  F[301] =  -4.720540746751245;
  F[302] =  6.200702008808505;
  F[303] =  -24.725748962986973;
  F[304] =  -16.35271561432375;
  F[305] =  32.0206371705689;
  F[306] =  -8.798742906270608;
  F[307] =  2.088725709396272;
  F[308] =  -5.390320534338301;
  F[309] =  -2.012484033154026;
  F[316] =  0;
  F[333] =  0.7456617360768952;
  F[318] =  1.6265549790119183;
  F[319] =  1.0756069317181118;
  F[320] =  21.307593210033964;
  F[321] =  -22.701938822247296;
  F[322] =  -18.74747695790321;
  F[323] =  28.61932505567683;
  F[324] =  -20.33298394149404;
  F[325] =  2.9328473166721296;
  F[326] =  -4.58821574413209;
  F[327] =  7.609161750968274;
  F[334] =  0;
  F[351] =  0.0026438860781303167;
  F[336] =  -5.416720971143093;
  F[337] =  -5.932858057543547;
  F[338] =  6.970243428035879;
  F[339] =  -4.6132668829787695;
  F[340] =  -7.1373401904121545;
  F[341] =  9.485679000488982;
  F[342] =  -5.325535368380419;
  F[343] =  -1.9964691179600622;
  F[344] =  -2.851508699574493;
  F[345] =  -11.294314336719545;
  F[352] =  0;
  F[369] =  0.07350952920511876;
  F[354] =  -2.294041618659397;
  F[355] =  -2.5339887136434815;
  F[356] =  3.721138447966526;
  F[357] =  -2.5826271426780143;
  F[358] =  -2.2822530206447813;
  F[359] =  3.352937488710267;
  F[360] =  -2.3285953073771775;
  F[361] =  -0.135663531232341;
  F[362] =  -1.0376322207789483;
  F[363] =  -2.170444808932932;
  F[370] =  0;
  F[391] =  0.8092688668662942;
  F[372] =  1.4944878326367168;
  F[373] =  1.445266450142947;
  F[374] =  1.0048559804938137;
  F[375] =  0.11187502841420165;
  F[376] =  0.10523391381674131;
  F[377] =  -0.05853956801121927;
  F[378] =  0.0747431411522736;
  F[379] =  0.07966478196016502;
  F[380] =  0.006379375192823983;
  F[381] =  0.05764863852479936;
  F[387] =  0.9999574293099679;
  F[388] =  -0.07445148979288738;
  F[389] =  0.9999171508897058;
  F[390] =  0.04916337909875758;
  F[392] =  0;
  F[476] =  10.1807956020533;
  F[480] =  0.11227861877483845;
  F[481] =  0.18030268928795945;
  F[482] =  0.22057487486802074;
  F[483] =  0.21251520222524953;
  F[484] =  0.13416918120931076;
  F[485] =  0.10741618810326312;
  F[486] =  0.0019313266624316704;
  F[478] =  1;
  F[455] =  1;
  F[393] =  0.8092688668662942;
  F[394] =  0.8092688668662942;
  F[395] =  0.8092688668662942;
  F[396] =  0.8092688668662942;
  F[397] =  0.8092688668662942;
  F[398] =  0.8092688668662942;
  F[399] =  0.8092688668662942;
  F[422] =  0.7325214115332304;
  F[405] =  1.0758775843484194;
  F[406] =  1.007453051208541;
  F[407] =  1.0052227736041939;
  F[408] =  -0.034483022198982353;
  F[409] =  0.030319115104720455;
  F[410] =  0.025740624830174516;
  F[411] =  0.042202922029885966;
  F[412] =  0.06591745570232846;
  F[413] =  0.012807454421860012;
  F[414] =  -0.06966730383120948;
  F[420] =  0.104555878099153;
  F[421] =  -0.02423839066468724;
  F[423] =  0;
  F[493] =  9.447371296375033;
  F[497] =  0.09381569434026035;
  F[498] =  0.2434474316762261;
  F[499] =  0.1123164325433176;
  F[500] =  0.11471673331773492;
  F[501] =  0.0656824071462864;
  F[502] =  0.04821335284860352;
  F[503] =  -0.008445950368995403;
  F[495] =  1;
  F[475] =  1;
  F[424] =  0.7325214115332304;
  F[425] =  0.7325214115332304;
  F[426] =  0.7325214115332304;
  F[427] =  0.7325214115332304;
  F[428] =  0.7325214115332304;
  F[429] =  0.7325214115332304;
  F[430] =  0.7325214115332304;
  F[453] =  0.8801541423251058;
  F[436] =  2.060053243632776;
  F[437] =  1.9938906561623133;
  F[438] =  1.0255072721974658;
  F[439] =  0.06063655959021816;
  F[440] =  0.13846568236832613;
  F[441] =  0.1161204320224287;
  F[442] =  0.007238447585259768;
  F[443] =  0.09022953556401753;
  F[444] =  0.07443014955175861;
  F[445] =  0.08828607722827615;
  F[451] =  0.20675285838750102;
  F[452] =  0.12740961156764027;
  F[454] =  0;
  F[473] =  0.8987560972131132;
  F[456] =  2.2143993239117994;
  F[457] =  2.183479208331232;
  F[458] =  1.0160171947208754;
  F[459] =  0.07544622299553709;
  F[460] =  -0.006998712306530066;
  F[461] =  0.1682852231925188;
  F[462] =  0.13981924930293435;
  F[463] =  0.0363676193944044;
  F[464] =  0.12370947471403458;
  F[465] =  0.11300395360319503;
  F[471] =  0.1645132399584476;
  F[472] =  0.03526730136935068;
  F[474] =  0;
  F[477] =  10.064301997307602;
  F[479] =  0.07722598641816056;
  F[492] =  0;
  F[558] =  4.117851837050858;
  F[529] =  0.003280401536902284;
  F[525] =  0.3287351427186592;
  F[545] =  -0.14317659812094308;
  F[494] =  9.398406700361209;
  F[496] =  -0.022148438770649054;
  F[509] =  0;
  F[559] =  1.3599904534382878;
  F[549] =  0.06605561936274826;
  F[526] =  0.2913659684992226;
  F[546] =  -0.3176756783033023;
  F[527] =  0.003280401536902284;
  F[510] =  -4.948066708455803;
  F[511] =  -5.716503650441606;
  F[512] =  6.220282062862948;
  F[513] =  -4.800180250438226;
  F[514] =  -6.476016619695008;
  F[515] =  8.53201790492116;
  F[516] =  -4.742557761276855;
  F[517] =  -1.3798876877360915;
  F[518] =  -2.495129715236223;
  F[519] =  -10.004063995962715;
  F[528] =  0;
  F[547] =  0.06605561936274826;
  F[530] =  -2.5823536049825324;
  F[531] =  -2.648919782060078;
  F[532] =  2.7456195446150318;
  F[533] =  -1.9480199485136005;
  F[534] =  -1.5644710363942094;
  F[535] =  2.142529181623122;
  F[536] =  -1.6058215384819912;
  F[537] =  -0.08152211166980143;
  F[538] =  -0.8326545231878906;
  F[539] =  -1.0835059903966227;
  F[548] =  0;
  F[567] =  3.064556034483936e-7;
  F[551] =  -14.998192635693089;
  F[552] =  7.150686193229834;
  F[560] =  -48.528272968372626;
  F[561] =  -23.885709453749154;
  F[562] =  51.38334486336941;
  F[563] =  -20.39835641069843;
  F[564] =  6.813283680560225;
  F[565] =  6.020631930884499;
  F[566] =  0.25520934862344696;
  F[568] =  0;


  F[3] = t1;
  F[5] = t2;
  F[7] = t3;
  F[9] = t4;
  F[11] = distance;
  F[13] = pitch;
  F[15] = roll;
  
  F[0] = F[1];F[1] = F[2];F[1] += F[3] * F[4];F[1] += F[5] * F[6];F[1] += F[7] * F[8];F[1] += F[9] * F[10];F[1] += F[11] * F[12];F[1] += F[13] * F[14];F[1] += F[15] * F[16];F[1] += F[17] * F[18];F[1] += F[19] * F[20];F[1] += F[21] * F[22];F[1] += F[23] * F[24];F[1] += F[25] * F[26];F[27] = (1 / (1 + exp(-F[1])));F[28] = F[27] * (1 - F[27]);F[29] = F[27];F[30] = F[27];F[31] = F[27];F[32] = F[27];F[33] = F[27];F[34] = F[27];F[35] = F[27];
  F[36] = F[37];F[37] = F[38];F[37] += F[3] * F[39];F[37] += F[5] * F[40];F[37] += F[7] * F[41];F[37] += F[9] * F[42];F[37] += F[11] * F[43];F[37] += F[13] * F[44];F[37] += F[15] * F[45];F[37] += F[17] * F[46];F[37] += F[19] * F[47];F[37] += F[21] * F[48];F[37] += F[23] * F[49];F[37] += F[25] * F[50];F[51] = (1 / (1 + exp(-F[37])));F[52] = F[51] * (1 - F[51]);F[53] = F[51];F[54] = F[51];F[55] = F[51];F[56] = F[51];F[57] = F[51];F[58] = F[51];F[59] = F[51];
  F[60] = F[61];F[61] = F[62];F[61] += F[3] * F[63];F[61] += F[5] * F[64];F[61] += F[7] * F[65];F[61] += F[9] * F[66];F[61] += F[11] * F[67];F[61] += F[13] * F[68];F[61] += F[15] * F[69];F[61] += F[17] * F[70];F[61] += F[19] * F[71];F[61] += F[21] * F[72];F[61] += F[23] * F[73];F[61] += F[25] * F[74];F[75] = (1 / (1 + exp(-F[61])));F[76] = F[75] * (1 - F[75]);F[77] = F[75];F[78] = F[75];F[79] = F[75];F[80] = F[75];F[81] = F[75];F[82] = F[75];F[83] = F[75];
  F[84] = F[85];F[85] = F[86];F[85] += F[3] * F[87];F[85] += F[5] * F[88];F[85] += F[7] * F[89];F[85] += F[9] * F[90];F[85] += F[11] * F[91];F[85] += F[13] * F[92];F[85] += F[15] * F[93];F[85] += F[17] * F[94];F[85] += F[19] * F[95];F[85] += F[21] * F[96];F[85] += F[23] * F[97];F[85] += F[25] * F[98];F[99] = (1 / (1 + exp(-F[85])));F[100] = F[99] * (1 - F[99]);F[101] = F[99];F[102] = F[99];F[103] = F[99];F[104] = F[99];F[105] = F[99];F[106] = F[99];F[107] = F[99];
  F[108] = F[109];F[109] = F[110];F[109] += F[3] * F[111];F[109] += F[5] * F[112];F[109] += F[7] * F[113];F[109] += F[9] * F[114];F[109] += F[11] * F[115];F[109] += F[13] * F[116];F[109] += F[15] * F[117];F[109] += F[17] * F[118];F[109] += F[19] * F[119];F[109] += F[21] * F[120];F[109] += F[23] * F[121];F[109] += F[25] * F[122];F[123] = (1 / (1 + exp(-F[109])));F[124] = F[123] * (1 - F[123]);F[125] = F[123];F[126] = F[123];F[127] = F[123];F[128] = F[123];F[129] = F[123];F[130] = F[123];F[131] = F[123];
  F[132] = F[133];F[133] = F[134];F[133] += F[3] * F[135];F[133] += F[5] * F[136];F[133] += F[7] * F[137];F[133] += F[9] * F[138];F[133] += F[11] * F[139];F[133] += F[13] * F[140];F[133] += F[15] * F[141];F[133] += F[17] * F[142];F[133] += F[19] * F[143];F[133] += F[21] * F[144];F[133] += F[23] * F[145];F[133] += F[25] * F[146];F[147] = (1 / (1 + exp(-F[133])));F[148] = F[147] * (1 - F[147]);F[149] = F[147];
  F[150] = F[151];F[151] = F[152];F[151] += F[3] * F[153];F[151] += F[5] * F[154];F[151] += F[7] * F[155];F[151] += F[9] * F[156];F[151] += F[11] * F[157];F[151] += F[13] * F[158];F[151] += F[15] * F[159];F[151] += F[17] * F[160];F[151] += F[19] * F[161];F[151] += F[21] * F[162];F[151] += F[23] * F[163];F[151] += F[25] * F[164];F[165] = (1 / (1 + exp(-F[151])));F[166] = F[165] * (1 - F[165]);F[167] = F[165];
  F[168] = F[169];F[169] = F[170];F[169] += F[3] * F[171];F[169] += F[5] * F[172];F[169] += F[7] * F[173];F[169] += F[9] * F[174];F[169] += F[11] * F[175];F[169] += F[13] * F[176];F[169] += F[15] * F[177];F[169] += F[17] * F[178];F[169] += F[19] * F[179];F[169] += F[21] * F[180];F[169] += F[23] * F[181];F[169] += F[25] * F[182];F[183] = (1 / (1 + exp(-F[169])));F[184] = F[183] * (1 - F[183]);F[185] = F[183];
  F[186] = F[187];F[187] = F[188];F[187] += F[3] * F[189];F[187] += F[5] * F[190];F[187] += F[7] * F[191];F[187] += F[9] * F[192];F[187] += F[11] * F[193];F[187] += F[13] * F[194];F[187] += F[15] * F[195];F[187] += F[17] * F[196];F[187] += F[19] * F[197];F[187] += F[21] * F[198];F[187] += F[23] * F[199];F[187] += F[25] * F[200];F[201] = (1 / (1 + exp(-F[187])));F[202] = F[201] * (1 - F[201]);F[203] = F[201];
  F[204] = F[205];F[205] = F[206];F[205] += F[3] * F[207];F[205] += F[5] * F[208];F[205] += F[7] * F[209];F[205] += F[9] * F[210];F[205] += F[11] * F[211];F[205] += F[13] * F[212];F[205] += F[15] * F[213];F[205] += F[17] * F[214];F[205] += F[19] * F[215];F[205] += F[21] * F[216];F[205] += F[23] * F[217];F[205] += F[25] * F[218];F[219] = (1 / (1 + exp(-F[205])));F[220] = F[219] * (1 - F[219]);F[221] = F[219];
  F[222] = F[223];F[223] = F[149] * F[224] * F[223] + F[225];F[223] += F[3] * F[226] * F[29];F[223] += F[5] * F[227] * F[30];F[223] += F[7] * F[228] * F[31];F[223] += F[9] * F[229] * F[32];F[223] += F[11] * F[230] * F[33];F[223] += F[13] * F[231] * F[34];F[223] += F[15] * F[232] * F[35];F[17] = (1 / (1 + exp(-F[223])));F[233] = F[17] * (1 - F[17]);
  F[234] = F[235];F[235] = F[167] * F[236] * F[235] + F[237];F[235] += F[3] * F[238] * F[53];F[235] += F[5] * F[239] * F[54];F[235] += F[7] * F[240] * F[55];F[235] += F[9] * F[241] * F[56];F[235] += F[11] * F[242] * F[57];F[235] += F[13] * F[243] * F[58];F[235] += F[15] * F[244] * F[59];F[19] = (1 / (1 + exp(-F[235])));F[245] = F[19] * (1 - F[19]);
  F[246] = F[247];F[247] = F[185] * F[248] * F[247] + F[249];F[247] += F[3] * F[250] * F[77];F[247] += F[5] * F[251] * F[78];F[247] += F[7] * F[252] * F[79];F[247] += F[9] * F[253] * F[80];F[247] += F[11] * F[254] * F[81];F[247] += F[13] * F[255] * F[82];F[247] += F[15] * F[256] * F[83];F[21] = (1 / (1 + exp(-F[247])));F[257] = F[21] * (1 - F[21]);
  F[258] = F[259];F[259] = F[203] * F[260] * F[259] + F[261];F[259] += F[3] * F[262] * F[101];F[259] += F[5] * F[263] * F[102];F[259] += F[7] * F[264] * F[103];F[259] += F[9] * F[265] * F[104];F[259] += F[11] * F[266] * F[105];F[259] += F[13] * F[267] * F[106];F[259] += F[15] * F[268] * F[107];F[23] = (1 / (1 + exp(-F[259])));F[269] = F[23] * (1 - F[23]);
  F[270] = F[271];F[271] = F[221] * F[272] * F[271] + F[273];F[271] += F[3] * F[274] * F[125];F[271] += F[5] * F[275] * F[126];F[271] += F[7] * F[276] * F[127];F[271] += F[9] * F[277] * F[128];F[271] += F[11] * F[278] * F[129];F[271] += F[13] * F[279] * F[130];F[271] += F[15] * F[280] * F[131];F[25] = (1 / (1 + exp(-F[271])));F[281] = F[25] * (1 - F[25]);
  F[282] = F[283];F[283] = F[284];F[283] += F[3] * F[285];F[283] += F[5] * F[286];F[283] += F[7] * F[287];F[283] += F[9] * F[288];F[283] += F[11] * F[289];F[283] += F[13] * F[290];F[283] += F[15] * F[291];F[283] += F[17] * F[292];F[283] += F[19] * F[293];F[283] += F[21] * F[294];F[283] += F[23] * F[295];F[283] += F[25] * F[296];F[297] = (1 / (1 + exp(-F[283])));F[298] = F[297] * (1 - F[297]);F[299] = F[297];
  F[300] = F[301];F[301] = F[302];F[301] += F[3] * F[303];F[301] += F[5] * F[304];F[301] += F[7] * F[305];F[301] += F[9] * F[306];F[301] += F[11] * F[307];F[301] += F[13] * F[308];F[301] += F[15] * F[309];F[301] += F[17] * F[310];F[301] += F[19] * F[311];F[301] += F[21] * F[312];F[301] += F[23] * F[313];F[301] += F[25] * F[314];F[315] = (1 / (1 + exp(-F[301])));F[316] = F[315] * (1 - F[315]);F[317] = F[315];
  F[318] = F[319];F[319] = F[320];F[319] += F[3] * F[321];F[319] += F[5] * F[322];F[319] += F[7] * F[323];F[319] += F[9] * F[324];F[319] += F[11] * F[325];F[319] += F[13] * F[326];F[319] += F[15] * F[327];F[319] += F[17] * F[328];F[319] += F[19] * F[329];F[319] += F[21] * F[330];F[319] += F[23] * F[331];F[319] += F[25] * F[332];F[333] = (1 / (1 + exp(-F[319])));F[334] = F[333] * (1 - F[333]);F[335] = F[333];
  F[336] = F[337];F[337] = F[338];F[337] += F[3] * F[339];F[337] += F[5] * F[340];F[337] += F[7] * F[341];F[337] += F[9] * F[342];F[337] += F[11] * F[343];F[337] += F[13] * F[344];F[337] += F[15] * F[345];F[337] += F[17] * F[346];F[337] += F[19] * F[347];F[337] += F[21] * F[348];F[337] += F[23] * F[349];F[337] += F[25] * F[350];F[351] = (1 / (1 + exp(-F[337])));F[352] = F[351] * (1 - F[351]);F[353] = F[351];
  F[354] = F[355];F[355] = F[356];F[355] += F[3] * F[357];F[355] += F[5] * F[358];F[355] += F[7] * F[359];F[355] += F[9] * F[360];F[355] += F[11] * F[361];F[355] += F[13] * F[362];F[355] += F[15] * F[363];F[355] += F[17] * F[364];F[355] += F[19] * F[365];F[355] += F[21] * F[366];F[355] += F[23] * F[367];F[355] += F[25] * F[368];F[369] = (1 / (1 + exp(-F[355])));F[370] = F[369] * (1 - F[369]);F[371] = F[369];
  F[372] = F[373];F[373] = F[374];F[373] += F[3] * F[375];F[373] += F[5] * F[376];F[373] += F[7] * F[377];F[373] += F[9] * F[378];F[373] += F[11] * F[379];F[373] += F[13] * F[380];F[373] += F[15] * F[381];F[373] += F[17] * F[382];F[373] += F[19] * F[383];F[373] += F[21] * F[384];F[373] += F[23] * F[385];F[373] += F[25] * F[386];F[373] += F[387] * F[388];F[373] += F[389] * F[390];F[391] = (1 / (1 + exp(-F[373])));F[392] = F[391] * (1 - F[391]);F[393] = F[391];F[394] = F[391];F[395] = F[391];F[396] = F[391];F[397] = F[391];F[398] = F[391];F[399] = F[391];F[400] = F[391];F[401] = F[391];F[402] = F[391];F[403] = F[391];F[404] = F[391];
  F[405] = F[406];F[406] = F[407];F[406] += F[3] * F[408];F[406] += F[5] * F[409];F[406] += F[7] * F[410];F[406] += F[9] * F[411];F[406] += F[11] * F[412];F[406] += F[13] * F[413];F[406] += F[15] * F[414];F[406] += F[17] * F[415];F[406] += F[19] * F[416];F[406] += F[21] * F[417];F[406] += F[23] * F[418];F[406] += F[25] * F[419];F[406] += F[387] * F[420];F[406] += F[389] * F[421];F[422] = (1 / (1 + exp(-F[406])));F[423] = F[422] * (1 - F[422]);F[424] = F[422];F[425] = F[422];F[426] = F[422];F[427] = F[422];F[428] = F[422];F[429] = F[422];F[430] = F[422];F[431] = F[422];F[432] = F[422];F[433] = F[422];F[434] = F[422];F[435] = F[422];
  F[436] = F[437];F[437] = F[438];F[437] += F[3] * F[439];F[437] += F[5] * F[440];F[437] += F[7] * F[441];F[437] += F[9] * F[442];F[437] += F[11] * F[443];F[437] += F[13] * F[444];F[437] += F[15] * F[445];F[437] += F[17] * F[446];F[437] += F[19] * F[447];F[437] += F[21] * F[448];F[437] += F[23] * F[449];F[437] += F[25] * F[450];F[437] += F[387] * F[451];F[437] += F[389] * F[452];F[453] = (1 / (1 + exp(-F[437])));F[454] = F[453] * (1 - F[453]);F[455] = F[453];
  F[456] = F[457];F[457] = F[458];F[457] += F[3] * F[459];F[457] += F[5] * F[460];F[457] += F[7] * F[461];F[457] += F[9] * F[462];F[457] += F[11] * F[463];F[457] += F[13] * F[464];F[457] += F[15] * F[465];F[457] += F[17] * F[466];F[457] += F[19] * F[467];F[457] += F[21] * F[468];F[457] += F[23] * F[469];F[457] += F[25] * F[470];F[457] += F[387] * F[471];F[457] += F[389] * F[472];F[473] = (1 / (1 + exp(-F[457])));F[474] = F[473] * (1 - F[473]);F[475] = F[473];
  F[476] = F[477];F[477] = F[455] * F[478] * F[477] + F[479];F[477] += F[3] * F[480] * F[393];F[477] += F[5] * F[481] * F[394];F[477] += F[7] * F[482] * F[395];F[477] += F[9] * F[483] * F[396];F[477] += F[11] * F[484] * F[397];F[477] += F[13] * F[485] * F[398];F[477] += F[15] * F[486] * F[399];F[477] += F[17] * F[487] * F[400];F[477] += F[19] * F[488] * F[401];F[477] += F[21] * F[489] * F[402];F[477] += F[23] * F[490] * F[403];F[477] += F[25] * F[491] * F[404];F[387] = (1 / (1 + exp(-F[477])));F[492] = F[387] * (1 - F[387]);
  F[493] = F[494];F[494] = F[475] * F[495] * F[494] + F[496];F[494] += F[3] * F[497] * F[424];F[494] += F[5] * F[498] * F[425];F[494] += F[7] * F[499] * F[426];F[494] += F[9] * F[500] * F[427];F[494] += F[11] * F[501] * F[428];F[494] += F[13] * F[502] * F[429];F[494] += F[15] * F[503] * F[430];F[494] += F[17] * F[504] * F[431];F[494] += F[19] * F[505] * F[432];F[494] += F[21] * F[506] * F[433];F[494] += F[23] * F[507] * F[434];F[494] += F[25] * F[508] * F[435];F[389] = (1 / (1 + exp(-F[494])));F[509] = F[389] * (1 - F[389]);
  F[510] = F[511];F[511] = F[512];F[511] += F[3] * F[513];F[511] += F[5] * F[514];F[511] += F[7] * F[515];F[511] += F[9] * F[516];F[511] += F[11] * F[517];F[511] += F[13] * F[518];F[511] += F[15] * F[519];F[511] += F[17] * F[520];F[511] += F[19] * F[521];F[511] += F[21] * F[522];F[511] += F[23] * F[523];F[511] += F[25] * F[524];F[511] += F[387] * F[525];F[511] += F[389] * F[526];F[527] = (1 / (1 + exp(-F[511])));F[528] = F[527] * (1 - F[527]);F[529] = F[527];
  F[530] = F[531];F[531] = F[532];F[531] += F[3] * F[533];F[531] += F[5] * F[534];F[531] += F[7] * F[535];F[531] += F[9] * F[536];F[531] += F[11] * F[537];F[531] += F[13] * F[538];F[531] += F[15] * F[539];F[531] += F[17] * F[540];F[531] += F[19] * F[541];F[531] += F[21] * F[542];F[531] += F[23] * F[543];F[531] += F[25] * F[544];F[531] += F[387] * F[545];F[531] += F[389] * F[546];F[547] = (1 / (1 + exp(-F[531])));F[548] = F[547] * (1 - F[547]);F[549] = F[547];
  F[550] = F[551];F[551] = F[552];F[551] += F[17] * F[553] * F[299];F[551] += F[19] * F[554] * F[317];F[551] += F[21] * F[555] * F[335];F[551] += F[23] * F[556] * F[353];F[551] += F[25] * F[557] * F[371];F[551] += F[387] * F[558] * F[529];F[551] += F[389] * F[559] * F[549];F[551] += F[3] * F[560];F[551] += F[5] * F[561];F[551] += F[7] * F[562];F[551] += F[9] * F[563];F[551] += F[11] * F[564];F[551] += F[13] * F[565];F[551] += F[15] * F[566];F[567] = (1 / (1 + exp(-F[551])));F[568] = F[567] * (1 - F[567]);
  float output = F[567] * 100;
  return output;
}


//LEFT HAND --> LEFT HEAD
//LEFT HAND --> LEFT HEAD
//LEFT HAND --> LEFT HEAD
//LEFT HAND --> LEFT HEAD
float nn_lefthand_lefthead_5521(float t1, float t2, float t3, float  t4, float distance) {
  float F[] = {0,0,0};
  /*  F[3]=input[0];
    F[5]=input[1];
    F[7]=input[2];
    F[9]=input[3];
    F[11]=input[4];
    F[13]=input[5];
    F[15]=input[6];
  */
F[3] =  0.8217821782178217;
F[5] = 0.8445544554455445;
F[7] = 0.8198019801980198;
F[9] = 0.8613861386138614;
F[11] = 0.196;
F[23] = 0.7987736044000071;
F[0] = 1.379642765053657;
F[1] = 1.3786469364539502;
F[2] = 1.0125056378084403;
F[4] = -0.0169842203607848;
F[6] = -0.016217856507618048;
F[8] = -0.0251051335943615;
F[10] = -0.02911372548729096;
F[12] = 0.062239421349630356;
F[13] = 0.9999970807951025;
F[14] = 0.04867503739793937;
F[15] = 0.9999974644134207;
F[16] = 0.11271383562915258;
F[17] = 0.9999983154070883;
F[18] = 0.08959945136365004;
F[19] = 0.9999988675151186;
F[20] = 0.12670487359877625;
F[21] = 0.999996919010944;
F[22] = 0.05028673088534187;
F[24] = 0;
F[190] = 12.746263542392084;
F[194] = 0.2722771145233883;
F[195] = 0.2546921430266003;
F[196] = 0.32102181900041055;
F[197] = 0.2768277045904216;
F[198] = 0.2858761487479504;
F[192] = 1;
F[125] = 1;
F[25] = 0.7987736044000071;
F[26] = 0.7987736044000071;
F[27] = 0.7987736044000071;
F[28] = 0.7987736044000071;
F[29] = 0.7987736044000071;
F[43] = 0.765403476671015;
F[30] = 1.1833402632838308;
F[31] = 1.1825359987342035;
F[32] = 1.0088968612422216;
F[33] = -0.03612122869111204;
F[34] = 0.1073153888520531;
F[35] = 0.11060949870295203;
F[36] = -0.06857486717083519;
F[37] = 0.050266537065199283;
F[38] = 0.00946767892001248;
F[39] = 0.12481456654468606;
F[40] = -0.05112253918252189;
F[41] = 0.05390957343153179;
F[42] = -0.06430278210843841;
F[44] = 0;
F[200] = 12.886672263971121;
F[204] = 0.3109849096262161;
F[205] = 0.3545269638620498;
F[206] = 0.20830794681314296;
F[207] = 0.18283837446151363;
F[208] = 0.2193511803852496;
F[202] = 1;
F[141] = 1;
F[45] = 0.765403476671015;
F[46] = 0.765403476671015;
F[47] = 0.765403476671015;
F[48] = 0.765403476671015;
F[49] = 0.765403476671015;
F[63] = 0.7567159774031119;
F[50] = 1.135822406012014;
F[51] = 1.134758409981555;
F[52] = 1.0057496003818096;
F[53] = -0.0405319004533066;
F[54] = -0.02851004485340484;
F[55] = -0.02618312512940479;
F[56] = 0.02278302548499794;
F[57] = 0.06649986656343568;
F[58] = 0.1339564495528099;
F[59] = -0.046441048014952815;
F[60] = 0.012514650232124044;
F[61] = -0.0029796953006227833;
F[62] = 0.07987730527729847;
F[64] = 0;
F[210] = 13.295623049544183;
F[214] = 0.21576794760298867;
F[215] = 0.2928978024104502;
F[216] = 0.23068605876738996;
F[217] = 0.33308540194024744;
F[218] = 0.1519507556458878;
F[212] = 1;
F[157] = 1;
F[65] = 0.7567159774031119;
F[66] = 0.7567159774031119;
F[67] = 0.7567159774031119;
F[68] = 0.7567159774031119;
F[69] = 0.7567159774031119;
F[83] = 0.7591327148559166;
F[70] = 1.1467661536341949;
F[71] = 1.1479305192338685;
F[72] = 1.0051526243926383;
F[73] = 0.03933469525634115;
F[74] = -0.046124489800186484;
F[75] = -0.022195568881652337;
F[76] = 0.12252575413188642;
F[77] = -0.07277285559349903;
F[78] = -0.038175921112878854;
F[79] = 0.055444327110717125;
F[80] = 0.02188042613424224;
F[81] = 0.055783471372245215;
F[82] = -0.02028143085874255;
F[84] = 0;
F[220] = 13.692353034086457;
F[224] = 0.18464094157014688;
F[225] = 0.34036201471670097;
F[226] = 0.210073134877544;
F[227] = 0.3215718575650733;
F[228] = 0.11316690675797428;
F[222] = 1;
F[173] = 1;
F[85] = 0.7591327148559166;
F[86] = 0.7591327148559166;
F[87] = 0.7591327148559166;
F[88] = 0.7591327148559166;
F[89] = 0.7591327148559166;
F[103] = 0.8081525544641096;
F[90] = 1.4385783449131153;
F[91] = 1.4380503440280283;
F[92] = 1.0103144487019904;
F[93] = -0.03925061578596839;
F[94] = 0.11418484453040446;
F[95] = -0.001186243515272173;
F[96] = 0.02062334424288597;
F[97] = 0.03300023912813654;
F[98] = 0.07245502151522266;
F[99] = 0.11685666291650879;
F[100] = 0.02493478606176172;
F[101] = 0.01376088628346449;
F[102] = 0.11396043178466796;
F[104] = 0;
F[230] = 12.691670180124664;
F[234] = 0.2129345196092144;
F[235] = 0.23005728754805735;
F[236] = 0.3303325330029281;
F[237] = 0.2946858036686667;
F[238] = 0.226673971585664;
F[232] = 1;
F[189] = 1;
F[105] = 0.8081525544641096;
F[106] = 0.8081525544641096;
F[107] = 0.8081525544641096;
F[108] = 0.8081525544641096;
F[109] = 0.8081525544641096;
F[123] = 0.9330710127996643;
F[110] = 2.635896839914632;
F[111] = 2.6348491455010463;
F[112] = 1.0283401802999732;
F[113] = 0.12865941967729017;
F[114] = 0.18519616221358937;
F[115] = 0.26840968932343984;
F[116] = 0.22317549643175078;
F[117] = 0.06548125833898927;
F[118] = 0.11918927338582977;
F[119] = 0.13127422705060973;
F[120] = 0.22790729948066438;
F[121] = 0.19193575334202825;
F[122] = 0.24347119907244952;
F[124] = 0;
F[139] = 0.9382625917255409;
F[126] = 2.7228657931874216;
F[127] = 2.721139818231617;
F[128] = 1.022181327931326;
F[129] = 0.20797360628513442;
F[130] = 0.23269811374847757;
F[131] = 0.13420055885554483;
F[132] = 0.22393764094793342;
F[133] = 0.10787383419600019;
F[134] = 0.13594366333456298;
F[135] = 0.2423531064906505;
F[136] = 0.14128265660912875;
F[137] = 0.24385397565497452;
F[138] = 0.23517984576256248;
F[140] = 0;
F[155] = 0.941029661417435;
F[142] = 2.7709109650009336;
F[143] = 2.7699400786519077;
F[144] = 1.0257217109676586;
F[145] = 0.11335132299285428;
F[146] = 0.18245988021408663;
F[147] = 0.19844958903651266;
F[148] = 0.08839241748466771;
F[149] = 0.060680889389978836;
F[150] = 0.23020374086274747;
F[151] = 0.27880068065225144;
F[152] = 0.2067670992494455;
F[153] = 0.26556099033949165;
F[154] = 0.26009260747550056;
F[156] = 0;
F[171] = 0.9384860208583333;
F[158] = 2.7263051338192867;
F[159] = 2.7250035086198308;
F[160] = 1.0320046420118552;
F[161] = 0.24134542065929251;
F[162] = 0.22681123425993535;
F[163] = 0.22422710812655405;
F[164] = 0.18753823644083745;
F[165] = 0.0813519713331593;
F[166] = 0.1608346598858404;
F[167] = 0.18648421511884697;
F[168] = 0.13128706384237368;
F[169] = 0.2027472020944587;
F[170] = 0.25017532915677193;
F[172] = 0;
F[187] = 0.9290563290853114;
F[174] = 2.5751074236514473;
F[175] = 2.572283176322474;
F[176] = 1.0287400715161148;
F[177] = 0.2616316766517427;
F[178] = 0.1361017594138568;
F[179] = 0.16253219370727473;
F[180] = 0.1900735375699953;
F[181] = 0.1765157533971453;
F[182] = 0.17563780036838364;
F[183] = 0.20150567912472792;
F[184] = 0.11106540290696437;
F[185] = 0.2759289979259722;
F[186] = 0.10675069030534032;
F[188] = 0;
F[191] = 12.744196354948823;
F[193] = 0.0457776958233228;
F[199] = 0;
F[481] = -3.54478273988455;
F[255] = 0.6367466993408887;
F[248] = -0.35092740780952025;
F[264] = -0.4773989481301353;
F[280] = -7.292517034916289;
F[296] = 4.092160774348083;
F[312] = -0.6780790530243914;
F[421] = 0.17821341775310653;
F[344] = 0.7777098591037162;
F[436] = 0.11771941971395047;
F[371] = 0.7781140081396275;
F[328] = 0.041361398774953304;
F[357] = -0.011245336884147288;
F[384] = 0.04368113169982612;
F[402] = 0.15002016362145956;
F[450] = -0.5375389924596515;
F[468] = -0.49402190607519486;
F[201] = 12.885083019867427;
F[203] = 0.07492102410199783;
F[209] = 0;
F[482] = -2.9729085480778217;
F[271] = 0.5425425891241159;
F[249] = -0.4798204348311659;
F[265] = -0.41567674488550954;
F[281] = -7.2473661641117015;
F[297] = 4.052809277415733;
F[313] = -0.6199036431323127;
F[422] = 0.13593368298700914;
F[345] = 0.7777098591037162;
F[437] = 0.1253478335567613;
F[372] = 0.7781140081396275;
F[329] = 0.02230389320353602;
F[358] = -0.011732641778739027;
F[385] = 0.04821943979869415;
F[403] = 0.0627466044071399;
F[451] = -0.5108684856311113;
F[469] = -0.3990411013654883;
F[211] = 13.29398493419327;
F[213] = 0.07133618147103149;
F[219] = 0;
F[483] = -11.416870117179155;
F[287] = 0.9969231082973843;
F[250] = -0.4196586753449785;
F[266] = -0.4420362803258332;
F[282] = -7.209057819705807;
F[298] = 4.135260127480677;
F[314] = -0.6978663061721381;
F[423] = -0.017040973757168423;
F[346] = 0.7777098591037162;
F[438] = 0.10359291966710356;
F[373] = 0.7781140081396275;
F[330] = -0.0032899151368128475;
F[359] = 0.016674245429208184;
F[386] = -0.014096631952854878;
F[404] = 0.04000846505078448;
F[452] = -0.4326422493328673;
F[470] = -0.46040848555133657;
F[221] = 13.691095196906684;
F[223] = 0.14379052069132112;
F[229] = 0;
F[484] = -21.631791430355626;
F[303] = 0.8224106833469075;
F[251] = -0.33677265999796296;
F[267] = -0.3192609970211353;
F[283] = -7.1866533080409685;
F[299] = 4.1120293537698505;
F[315] = -0.7797696884987028;
F[424] = 0.07714258175367966;
F[347] = 0.7777098591037162;
F[439] = 0.16458429950960962;
F[374] = 0.7781140081396275;
F[331] = 0.10248766753145815;
F[360] = -0.05420264039357341;
F[387] = 0.0854835551466824;
F[405] = 0.024342705748805434;
F[453] = -0.5183188871635892;
F[471] = -0.3938326430899119;
F[231] = 12.690256809427797;
F[233] = 0.13331971437371817;
F[239] = 0;
F[485] = -2.321516218861956;
F[319] = 0.1271025362235535;
F[252] = -0.37247496158140203;
F[268] = -0.3026079878617976;
F[284] = -7.303352273934704;
F[300] = 4.099911581402428;
F[316] = -0.6352774701566867;
F[425] = 0.1842144094164711;
F[348] = 0.7777098591037162;
F[440] = 0.1504528339431136;
F[375] = 0.7781140081396275;
F[332] = 0.0581507694479479;
F[361] = -0.06576136831585233;
F[388] = 0.14372403862072547;
F[406] = 0.08823275083754199;
F[454] = -0.5754362700475726;
F[472] = -0.4855199594856393;
F[253] = 0.6367466993408887;
F[240] = 0.45592269681814296;
F[241] = 0.5612715413672491;
F[242] = 5.999524886305885;
F[243] = -4.8608379545799965;
F[244] = -5.0209660454745;
F[245] = -5.048123804552307;
F[246] = 12.064690633246384;
F[247] = -6.584302327766837;
F[254] = 0;
F[478] = -11.877564223766228;
F[269] = 0.5425425891241159;
F[256] = 0.0848902244588971;
F[257] = 0.17058279823047695;
F[258] = 5.5925013261695;
F[259] = -4.176323867929474;
F[260] = -5.0944897683439265;
F[261] = -4.921723494453828;
F[262] = 11.065928002025508;
F[263] = -5.355785389366288;
F[270] = 0;
F[285] = 0.9969231082973843;
F[272] = 5.515234742864618;
F[273] = 5.780753743965333;
F[274] = 9.553902934245665;
F[275] = -16.44744747439653;
F[276] = 2.3582206148563123;
F[277] = 1.4659339083235428;
F[278] = 54.26295455832008;
F[279] = -16.594929129332442;
F[286] = 0;
F[301] = 0.8224106833469075;
F[288] = 1.418954687903816;
F[289] = 1.53276621029912;
F[290] = 12.324395924187623;
F[291] = 11.397573409479476;
F[292] = -52.922173108338505;
F[293] = -36.4482223819095;
F[294] = 40.44046243040355;
F[295] = -7.113224904948667;
F[302] = 0;
F[317] = 0.1271025362235535;
F[304] = -1.9502381822881674;
F[305] = -1.9268239636743703;
F[306] = 3.8597948066223418;
F[307] = -3.249805842380844;
F[308] = -3.1597515800002123;
F[309] = -3.080850919005727;
F[310] = 6.866089594647844;
F[311] = -1.463387889464287;
F[318] = 0;
F[337] = 0.7777098591037162;
F[320] = 1.251015774204429;
F[321] = 1.2523700533361863;
F[322] = 1.0025731618265965;
F[323] = -0.06583348395530414;
F[324] = 0.007257206567138032;
F[325] = -0.0386309910897995;
F[326] = 0.0603492955451592;
F[327] = -0.08464704173761473;
F[333] = 0.9976880706699125;
F[334] = 0.01265008084600924;
F[335] = 0.9970705761877039;
F[336] = 0.0633992164964588;
F[338] = 0;
F[412] = 6.068289952022081;
F[416] = 0.10253154748601545;
F[417] = 0.14730107235288195;
F[418] = 0.05592286922077444;
F[419] = 0.12527621956254317;
F[420] = 0.004570633761194863;
F[414] = 1;
F[393] = 1;
F[339] = 0.7777098591037162;
F[340] = 0.7777098591037162;
F[341] = 0.7777098591037162;
F[342] = 0.7777098591037162;
F[343] = 0.7777098591037162;
F[364] = 0.7781140081396275;
F[349] = 1.256091033805747;
F[350] = 1.254709353685959;
F[351] = 1.003572091489873;
F[352] = 0.06709120690480229;
F[353] = 0.07930233148495987;
F[354] = 0.010883435188546717;
F[355] = 0.09880390238834254;
F[356] = 0.08634779731990737;
F[362] = 0.10747528290443598;
F[363] = 0.03435760264543091;
F[365] = 0;
F[427] = 5.83117932532161;
F[431] = 0.09631224830226058;
F[432] = 0.1473519932605433;
F[433] = 0.1542445818709198;
F[434] = 0.07874909170891355;
F[435] = 0.08615500327763709;
F[429] = 1;
F[411] = 1;
F[366] = 0.7781140081396275;
F[367] = 0.7781140081396275;
F[368] = 0.7781140081396275;
F[369] = 0.7781140081396275;
F[370] = 0.7781140081396275;
F[391] = 0.8635207628027195;
F[376] = 1.8459747877843782;
F[377] = 1.8448454476236984;
F[378] = 1.0147793501923024;
F[379] = 0.0979553095107962;
F[380] = 0.15255734244149713;
F[381] = 0.08855222838201357;
F[382] = 0.10744293323401344;
F[383] = 0.07057575444735677;
F[389] = 0.015207715557349686;
F[390] = 0.11573209710781004;
F[392] = 0;
F[409] = 0.8288444067687063;
F[394] = 1.576915073469927;
F[395] = 1.5774594049351098;
F[396] = 1.0300493527608148;
F[397] = -0.005900011438339391;
F[398] = 0.02550132449604248;
F[399] = 0.027055261299872506;
F[400] = 0.017691692961603413;
F[401] = -0.034027801813832435;
F[407] = 0.09703884437100367;
F[408] = 0.038169875514975166;
F[410] = 0;
F[413] = 6.067358289245321;
F[415] = 0.10699421180228881;
F[426] = 0;
F[486] = -0.7383427318135145;
F[459] = 0.01912177288146306;
F[455] = -0.6185273228372309;
F[473] = -0.5587725660129266;
F[428] = 5.830015803412684;
F[430] = 0.156076184836362;
F[441] = 0;
F[487] = -0.8434675223158613;
F[477] = 0.018957977892105706;
F[456] = -0.45491871234713854;
F[474] = -0.4785977754173548;
F[457] = 0.01912177288146306;
F[442] = -3.9483933935693267;
F[443] = -3.9376206932322377;
F[444] = 1.8937056923754374;
F[445] = -0.8414348692872529;
F[446] = -1.2512398533367897;
F[447] = -1.3813014855785817;
F[448] = 1.0013160053077559;
F[449] = -0.6731135562641272;
F[458] = 0;
F[475] = 0.018957977892105706;
F[460] = -3.960961015036287;
F[461] = -3.9463904547606874;
F[462] = 1.9135511983281726;
F[463] = -0.8484890855179631;
F[464] = -1.4626770428925409;
F[465] = -1.6379438909481026;
F[466] = 1.0419316230939752;
F[467] = -0.9104828730206199;
F[476] = 0;
F[493] = 0.00000731423383077844;
F[479] = -11.825680954186964;
F[480] = 8.1432679220962;
F[488] = 26.747651965147714;
F[489] = 27.577406366216046;
F[490] = 31.4505697183844;
F[491] = -59.827584351154265;
F[492] = -37.043865198362425;
F[494] = 0;

  F[3] = t1;
  F[5] = t2;
  F[7] = t3;
  F[9] = t4;
  F[11] = distance;

F[0] = F[1];F[1] = F[2];F[1] += F[3] * F[4];F[1] += F[5] * F[6];F[1] += F[7] * F[8];F[1] += F[9] * F[10];F[1] += F[11] * F[12];F[1] += F[13] * F[14];F[1] += F[15] * F[16];F[1] += F[17] * F[18];F[1] += F[19] * F[20];F[1] += F[21] * F[22];F[23] = (1 / (1 + exp(-F[1])));F[24] = F[23] * (1 - F[23]);F[25] = F[23];F[26] = F[23];F[27] = F[23];F[28] = F[23];F[29] = F[23];
F[30] = F[31];F[31] = F[32];F[31] += F[3] * F[33];F[31] += F[5] * F[34];F[31] += F[7] * F[35];F[31] += F[9] * F[36];F[31] += F[11] * F[37];F[31] += F[13] * F[38];F[31] += F[15] * F[39];F[31] += F[17] * F[40];F[31] += F[19] * F[41];F[31] += F[21] * F[42];F[43] = (1 / (1 + exp(-F[31])));F[44] = F[43] * (1 - F[43]);F[45] = F[43];F[46] = F[43];F[47] = F[43];F[48] = F[43];F[49] = F[43];
F[50] = F[51];F[51] = F[52];F[51] += F[3] * F[53];F[51] += F[5] * F[54];F[51] += F[7] * F[55];F[51] += F[9] * F[56];F[51] += F[11] * F[57];F[51] += F[13] * F[58];F[51] += F[15] * F[59];F[51] += F[17] * F[60];F[51] += F[19] * F[61];F[51] += F[21] * F[62];F[63] = (1 / (1 + exp(-F[51])));F[64] = F[63] * (1 - F[63]);F[65] = F[63];F[66] = F[63];F[67] = F[63];F[68] = F[63];F[69] = F[63];
F[70] = F[71];F[71] = F[72];F[71] += F[3] * F[73];F[71] += F[5] * F[74];F[71] += F[7] * F[75];F[71] += F[9] * F[76];F[71] += F[11] * F[77];F[71] += F[13] * F[78];F[71] += F[15] * F[79];F[71] += F[17] * F[80];F[71] += F[19] * F[81];F[71] += F[21] * F[82];F[83] = (1 / (1 + exp(-F[71])));F[84] = F[83] * (1 - F[83]);F[85] = F[83];F[86] = F[83];F[87] = F[83];F[88] = F[83];F[89] = F[83];
F[90] = F[91];F[91] = F[92];F[91] += F[3] * F[93];F[91] += F[5] * F[94];F[91] += F[7] * F[95];F[91] += F[9] * F[96];F[91] += F[11] * F[97];F[91] += F[13] * F[98];F[91] += F[15] * F[99];F[91] += F[17] * F[100];F[91] += F[19] * F[101];F[91] += F[21] * F[102];F[103] = (1 / (1 + exp(-F[91])));F[104] = F[103] * (1 - F[103]);F[105] = F[103];F[106] = F[103];F[107] = F[103];F[108] = F[103];F[109] = F[103];
F[110] = F[111];F[111] = F[112];F[111] += F[3] * F[113];F[111] += F[5] * F[114];F[111] += F[7] * F[115];F[111] += F[9] * F[116];F[111] += F[11] * F[117];F[111] += F[13] * F[118];F[111] += F[15] * F[119];F[111] += F[17] * F[120];F[111] += F[19] * F[121];F[111] += F[21] * F[122];F[123] = (1 / (1 + exp(-F[111])));F[124] = F[123] * (1 - F[123]);F[125] = F[123];
F[126] = F[127];F[127] = F[128];F[127] += F[3] * F[129];F[127] += F[5] * F[130];F[127] += F[7] * F[131];F[127] += F[9] * F[132];F[127] += F[11] * F[133];F[127] += F[13] * F[134];F[127] += F[15] * F[135];F[127] += F[17] * F[136];F[127] += F[19] * F[137];F[127] += F[21] * F[138];F[139] = (1 / (1 + exp(-F[127])));F[140] = F[139] * (1 - F[139]);F[141] = F[139];
F[142] = F[143];F[143] = F[144];F[143] += F[3] * F[145];F[143] += F[5] * F[146];F[143] += F[7] * F[147];F[143] += F[9] * F[148];F[143] += F[11] * F[149];F[143] += F[13] * F[150];F[143] += F[15] * F[151];F[143] += F[17] * F[152];F[143] += F[19] * F[153];F[143] += F[21] * F[154];F[155] = (1 / (1 + exp(-F[143])));F[156] = F[155] * (1 - F[155]);F[157] = F[155];
F[158] = F[159];F[159] = F[160];F[159] += F[3] * F[161];F[159] += F[5] * F[162];F[159] += F[7] * F[163];F[159] += F[9] * F[164];F[159] += F[11] * F[165];F[159] += F[13] * F[166];F[159] += F[15] * F[167];F[159] += F[17] * F[168];F[159] += F[19] * F[169];F[159] += F[21] * F[170];F[171] = (1 / (1 + exp(-F[159])));F[172] = F[171] * (1 - F[171]);F[173] = F[171];
F[174] = F[175];F[175] = F[176];F[175] += F[3] * F[177];F[175] += F[5] * F[178];F[175] += F[7] * F[179];F[175] += F[9] * F[180];F[175] += F[11] * F[181];F[175] += F[13] * F[182];F[175] += F[15] * F[183];F[175] += F[17] * F[184];F[175] += F[19] * F[185];F[175] += F[21] * F[186];F[187] = (1 / (1 + exp(-F[175])));F[188] = F[187] * (1 - F[187]);F[189] = F[187];
F[190] = F[191];F[191] = F[125] * F[192] * F[191] + F[193];F[191] += F[3] * F[194] * F[25];F[191] += F[5] * F[195] * F[26];F[191] += F[7] * F[196] * F[27];F[191] += F[9] * F[197] * F[28];F[191] += F[11] * F[198] * F[29];F[13] = (1 / (1 + exp(-F[191])));F[199] = F[13] * (1 - F[13]);
F[200] = F[201];F[201] = F[141] * F[202] * F[201] + F[203];F[201] += F[3] * F[204] * F[45];F[201] += F[5] * F[205] * F[46];F[201] += F[7] * F[206] * F[47];F[201] += F[9] * F[207] * F[48];F[201] += F[11] * F[208] * F[49];F[15] = (1 / (1 + exp(-F[201])));F[209] = F[15] * (1 - F[15]);
F[210] = F[211];F[211] = F[157] * F[212] * F[211] + F[213];F[211] += F[3] * F[214] * F[65];F[211] += F[5] * F[215] * F[66];F[211] += F[7] * F[216] * F[67];F[211] += F[9] * F[217] * F[68];F[211] += F[11] * F[218] * F[69];F[17] = (1 / (1 + exp(-F[211])));F[219] = F[17] * (1 - F[17]);
F[220] = F[221];F[221] = F[173] * F[222] * F[221] + F[223];F[221] += F[3] * F[224] * F[85];F[221] += F[5] * F[225] * F[86];F[221] += F[7] * F[226] * F[87];F[221] += F[9] * F[227] * F[88];F[221] += F[11] * F[228] * F[89];F[19] = (1 / (1 + exp(-F[221])));F[229] = F[19] * (1 - F[19]);
F[230] = F[231];F[231] = F[189] * F[232] * F[231] + F[233];F[231] += F[3] * F[234] * F[105];F[231] += F[5] * F[235] * F[106];F[231] += F[7] * F[236] * F[107];F[231] += F[9] * F[237] * F[108];F[231] += F[11] * F[238] * F[109];F[21] = (1 / (1 + exp(-F[231])));F[239] = F[21] * (1 - F[21]);
F[240] = F[241];F[241] = F[242];F[241] += F[3] * F[243];F[241] += F[5] * F[244];F[241] += F[7] * F[245];F[241] += F[9] * F[246];F[241] += F[11] * F[247];F[241] += F[13] * F[248];F[241] += F[15] * F[249];F[241] += F[17] * F[250];F[241] += F[19] * F[251];F[241] += F[21] * F[252];F[253] = (1 / (1 + exp(-F[241])));F[254] = F[253] * (1 - F[253]);F[255] = F[253];
F[256] = F[257];F[257] = F[258];F[257] += F[3] * F[259];F[257] += F[5] * F[260];F[257] += F[7] * F[261];F[257] += F[9] * F[262];F[257] += F[11] * F[263];F[257] += F[13] * F[264];F[257] += F[15] * F[265];F[257] += F[17] * F[266];F[257] += F[19] * F[267];F[257] += F[21] * F[268];F[269] = (1 / (1 + exp(-F[257])));F[270] = F[269] * (1 - F[269]);F[271] = F[269];
F[272] = F[273];F[273] = F[274];F[273] += F[3] * F[275];F[273] += F[5] * F[276];F[273] += F[7] * F[277];F[273] += F[9] * F[278];F[273] += F[11] * F[279];F[273] += F[13] * F[280];F[273] += F[15] * F[281];F[273] += F[17] * F[282];F[273] += F[19] * F[283];F[273] += F[21] * F[284];F[285] = (1 / (1 + exp(-F[273])));F[286] = F[285] * (1 - F[285]);F[287] = F[285];
F[288] = F[289];F[289] = F[290];F[289] += F[3] * F[291];F[289] += F[5] * F[292];F[289] += F[7] * F[293];F[289] += F[9] * F[294];F[289] += F[11] * F[295];F[289] += F[13] * F[296];F[289] += F[15] * F[297];F[289] += F[17] * F[298];F[289] += F[19] * F[299];F[289] += F[21] * F[300];F[301] = (1 / (1 + exp(-F[289])));F[302] = F[301] * (1 - F[301]);F[303] = F[301];
F[304] = F[305];F[305] = F[306];F[305] += F[3] * F[307];F[305] += F[5] * F[308];F[305] += F[7] * F[309];F[305] += F[9] * F[310];F[305] += F[11] * F[311];F[305] += F[13] * F[312];F[305] += F[15] * F[313];F[305] += F[17] * F[314];F[305] += F[19] * F[315];F[305] += F[21] * F[316];F[317] = (1 / (1 + exp(-F[305])));F[318] = F[317] * (1 - F[317]);F[319] = F[317];
F[320] = F[321];F[321] = F[322];F[321] += F[3] * F[323];F[321] += F[5] * F[324];F[321] += F[7] * F[325];F[321] += F[9] * F[326];F[321] += F[11] * F[327];F[321] += F[13] * F[328];F[321] += F[15] * F[329];F[321] += F[17] * F[330];F[321] += F[19] * F[331];F[321] += F[21] * F[332];F[321] += F[333] * F[334];F[321] += F[335] * F[336];F[337] = (1 / (1 + exp(-F[321])));F[338] = F[337] * (1 - F[337]);F[339] = F[337];F[340] = F[337];F[341] = F[337];F[342] = F[337];F[343] = F[337];F[344] = F[337];F[345] = F[337];F[346] = F[337];F[347] = F[337];F[348] = F[337];
F[349] = F[350];F[350] = F[351];F[350] += F[3] * F[352];F[350] += F[5] * F[353];F[350] += F[7] * F[354];F[350] += F[9] * F[355];F[350] += F[11] * F[356];F[350] += F[13] * F[357];F[350] += F[15] * F[358];F[350] += F[17] * F[359];F[350] += F[19] * F[360];F[350] += F[21] * F[361];F[350] += F[333] * F[362];F[350] += F[335] * F[363];F[364] = (1 / (1 + exp(-F[350])));F[365] = F[364] * (1 - F[364]);F[366] = F[364];F[367] = F[364];F[368] = F[364];F[369] = F[364];F[370] = F[364];F[371] = F[364];F[372] = F[364];F[373] = F[364];F[374] = F[364];F[375] = F[364];
F[376] = F[377];F[377] = F[378];F[377] += F[3] * F[379];F[377] += F[5] * F[380];F[377] += F[7] * F[381];F[377] += F[9] * F[382];F[377] += F[11] * F[383];F[377] += F[13] * F[384];F[377] += F[15] * F[385];F[377] += F[17] * F[386];F[377] += F[19] * F[387];F[377] += F[21] * F[388];F[377] += F[333] * F[389];F[377] += F[335] * F[390];F[391] = (1 / (1 + exp(-F[377])));F[392] = F[391] * (1 - F[391]);F[393] = F[391];
F[394] = F[395];F[395] = F[396];F[395] += F[3] * F[397];F[395] += F[5] * F[398];F[395] += F[7] * F[399];F[395] += F[9] * F[400];F[395] += F[11] * F[401];F[395] += F[13] * F[402];F[395] += F[15] * F[403];F[395] += F[17] * F[404];F[395] += F[19] * F[405];F[395] += F[21] * F[406];F[395] += F[333] * F[407];F[395] += F[335] * F[408];F[409] = (1 / (1 + exp(-F[395])));F[410] = F[409] * (1 - F[409]);F[411] = F[409];
F[412] = F[413];F[413] = F[393] * F[414] * F[413] + F[415];F[413] += F[3] * F[416] * F[339];F[413] += F[5] * F[417] * F[340];F[413] += F[7] * F[418] * F[341];F[413] += F[9] * F[419] * F[342];F[413] += F[11] * F[420] * F[343];F[413] += F[13] * F[421] * F[344];F[413] += F[15] * F[422] * F[345];F[413] += F[17] * F[423] * F[346];F[413] += F[19] * F[424] * F[347];F[413] += F[21] * F[425] * F[348];F[333] = (1 / (1 + exp(-F[413])));F[426] = F[333] * (1 - F[333]);
F[427] = F[428];F[428] = F[411] * F[429] * F[428] + F[430];F[428] += F[3] * F[431] * F[366];F[428] += F[5] * F[432] * F[367];F[428] += F[7] * F[433] * F[368];F[428] += F[9] * F[434] * F[369];F[428] += F[11] * F[435] * F[370];F[428] += F[13] * F[436] * F[371];F[428] += F[15] * F[437] * F[372];F[428] += F[17] * F[438] * F[373];F[428] += F[19] * F[439] * F[374];F[428] += F[21] * F[440] * F[375];F[335] = (1 / (1 + exp(-F[428])));F[441] = F[335] * (1 - F[335]);
F[442] = F[443];F[443] = F[444];F[443] += F[3] * F[445];F[443] += F[5] * F[446];F[443] += F[7] * F[447];F[443] += F[9] * F[448];F[443] += F[11] * F[449];F[443] += F[13] * F[450];F[443] += F[15] * F[451];F[443] += F[17] * F[452];F[443] += F[19] * F[453];F[443] += F[21] * F[454];F[443] += F[333] * F[455];F[443] += F[335] * F[456];F[457] = (1 / (1 + exp(-F[443])));F[458] = F[457] * (1 - F[457]);F[459] = F[457];
F[460] = F[461];F[461] = F[462];F[461] += F[3] * F[463];F[461] += F[5] * F[464];F[461] += F[7] * F[465];F[461] += F[9] * F[466];F[461] += F[11] * F[467];F[461] += F[13] * F[468];F[461] += F[15] * F[469];F[461] += F[17] * F[470];F[461] += F[19] * F[471];F[461] += F[21] * F[472];F[461] += F[333] * F[473];F[461] += F[335] * F[474];F[475] = (1 / (1 + exp(-F[461])));F[476] = F[475] * (1 - F[475]);F[477] = F[475];
F[478] = F[479];F[479] = F[480];F[479] += F[13] * F[481] * F[255];F[479] += F[15] * F[482] * F[271];F[479] += F[17] * F[483] * F[287];F[479] += F[19] * F[484] * F[303];F[479] += F[21] * F[485] * F[319];F[479] += F[333] * F[486] * F[459];F[479] += F[335] * F[487] * F[477];F[479] += F[3] * F[488];F[479] += F[5] * F[489];F[479] += F[7] * F[490];F[479] += F[9] * F[491];F[479] += F[11] * F[492];F[493] = (1 / (1 + exp(-F[479])));F[494] = F[493] * (1 - F[493]);

float output = F[493] * 100;
return output;
}


// LEFT HAND --> LEFT HEAD
// LEFT HAND --> LEFT HEAD
// LEFT HAND --> LEFT HEAD
// LEFT HAND --> LEFT HEAD
float nn_lefthand_lefthead_7521(float t1, float t2, float t3, float  t4, float distance, float pitch, float roll) {
  float F[] = {0,0,0};
  /*  F[3]=input[0];
    F[5]=input[1];
    F[7]=input[2];
    F[9]=input[3];
    F[11]=input[4];
    F[13]=input[5];
    F[15]=input[6];
  */
F[3] = 0.8643564356435643;
F[5] = 0.8445544554455445;
F[7] = 0.8198019801980198;
F[9] = 0.8613861386138614;
F[11] = 0.196;
F[13] = 0.5972222222222222;
F[15] = 0.3472222222222222;
F[27] = 0.9231264493634256;
F[0] = 2.4837333948631377;
F[1] = 2.48560435090537;
F[2] = 1.405891174734949;
F[4] = 0.1799006129843117;
F[6] = 0.11386927213239777;
F[8] = -0.03672093426121681;
F[10] = 0.09365691921099618;
F[12] = -0.103074261931177;
F[14] = -0.03967183154116981;
F[16] = -0.020079894637437062;
F[17] = 0.9999670869765149;
F[18] = 0.12009124511299428;
F[19] = 0.9999904660099251;
F[20] = 0.18664380533885433;
F[21] = 0.9999936917154942;
F[22] = 0.09649724106849282;
F[23] = 0.9999955306510424;
F[24] = 0.18895199850923192;
F[25] = 0.9999943654810479;
F[26] = 0.2361671266612787;
F[28] = 0;
F[222] = 10.314363083854737;
F[226] = 0.45849377223371013;
F[227] = 0.4322455923613314;
F[228] = -0.38673337706389005;
F[229] = 1.6506687541430902;
F[230] = -0.1331435403694698;
F[231] = 0.6264066575765644;
F[232] = -0.009894498889413161;
F[224] = 1;
F[149] = 1;
F[29] = 0.9231264493634256;
F[30] = 0.9231264493634256;
F[31] = 0.9231264493634256;
F[32] = 0.9231264493634256;
F[33] = 0.9231264493634256;
F[34] = 0.9231264493634256;
F[35] = 0.9231264493634256;
F[51] = 0.7514529922441432;
F[36] = 1.1083858837060734;
F[37] = 1.1063766616659838;
F[38] = 1.008584233046685;
F[39] = -0.02537464819314322;
F[40] = 0.01242568511005298;
F[41] = 0.017576061410690867;
F[42] = 0.04728615974109559;
F[43] = 0.10921043734104972;
F[44] = -0.0654688396025858;
F[45] = 0.0798675926931667;
F[46] = -0.06382459476845508;
F[47] = 0.0658163230412774;
F[48] = 0.07030977796839911;
F[49] = -0.04185413878262631;
F[50] = 0.013604339548535922;
F[52] = 0;
F[234] = 11.564000343437046;
F[238] = 0.20513610146592023;
F[239] = 0.28722582882323205;
F[240] = 0.310575036312112;
F[241] = 0.17991406760594408;
F[242] = 0.1877068459512627;
F[243] = 0.249854114801592;
F[244] = 0.016292032039513387;
F[236] = 1;
F[167] = 1;
F[53] = 0.7514529922441432;
F[54] = 0.7514529922441432;
F[55] = 0.7514529922441432;
F[56] = 0.7514529922441432;
F[57] = 0.7514529922441432;
F[58] = 0.7514529922441432;
F[59] = 0.7514529922441432;
F[75] = 0.7639873691774365;
F[60] = 1.1754288765202563;
F[61] = 1.1746659325115578;
F[62] = 1.0078776598521557;
F[63] = 0.07117592598134084;
F[64] = 0.0034500022002826765;
F[65] = -0.07181713193009855;
F[66] = 0.08357109147760758;
F[67] = 0.008705803234942857;
F[68] = 0.022388911774757544;
F[69] = 0.10106332892123572;
F[70] = 0.013865105424648092;
F[71] = 0.09952763759593657;
F[72] = -0.008684211061675327;
F[73] = 0.00418193239556384;
F[74] = -0.06981637819216627;
F[76] = 0;
F[246] = 11.976928778069523;
F[250] = 0.22046701497249424;
F[251] = 0.2570651760820783;
F[252] = 0.2220156714147373;
F[253] = 0.25442978851780174;
F[254] = 0.007731412600658393;
F[255] = 0.30450067132195147;
F[256] = 0.10658194459441539;
F[248] = 1;
F[185] = 1;
F[77] = 0.7639873691774365;
F[78] = 0.7639873691774365;
F[79] = 0.7639873691774365;
F[80] = 0.7639873691774365;
F[81] = 0.7639873691774365;
F[82] = 0.7639873691774365;
F[83] = 0.7639873691774365;
F[99] = 0.7802121194188094;
F[84] = 1.2673592030332312;
F[85] = 1.266902928629106;
F[86] = 1.0074548077404057;
F[87] = 0.0680705270252057;
F[88] = -0.04102550172578415;
F[89] = 0.06131831973506193;
F[90] = 0.07015806192021021;
F[91] = 0.020467027752615066;
F[92] = 0.020721557866405223;
F[93] = 0.012824636787865806;
F[94] = 0.07000223762089663;
F[95] = 0.05452987975909481;
F[96] = 0.08428460928233904;
F[97] = -0.03748326818197038;
F[98] = -0.0676134167519142;
F[100] = 0;
F[258] = 12.319101332334059;
F[262] = 0.28026247551463507;
F[263] = 0.3583854703640984;
F[264] = 0.2463335935552207;
F[265] = 0.2551209063768816;
F[266] = 0.022272848004441917;
F[267] = 0.12665784322462137;
F[268] = 0.06980164914310115;
F[260] = 1;
F[203] = 1;
F[101] = 0.7802121194188094;
F[102] = 0.7802121194188094;
F[103] = 0.7802121194188094;
F[104] = 0.7802121194188094;
F[105] = 0.7802121194188094;
F[106] = 0.7802121194188094;
F[107] = 0.7802121194188094;
F[123] = 0.7873293004205568;
F[108] = 1.310941056468617;
F[109] = 1.3089016277963494;
F[110] = 1.0084383193751447;
F[111] = 0.09114622888226771;
F[112] = -0.013985655185679952;
F[113] = -0.0472770406483656;
F[114] = 0.02512590178095495;
F[115] = 0.10230768371497173;
F[116] = -0.03343352152158957;
F[117] = 0.08916891400680979;
F[118] = 0.09061500351803592;
F[119] = -0.022676309279957535;
F[120] = 0.07836204087803875;
F[121] = 0.11970072144598497;
F[122] = -0.04643755953899136;
F[124] = 0;
F[270] = 12.090171773167553;
F[274] = 0.2092809555391444;
F[275] = 0.3046382103594231;
F[276] = 0.29284501454258743;
F[277] = 0.24307333445268514;
F[278] = 0.04663433503060104;
F[279] = 0.21613798276591514;
F[280] = -0.07398591285306023;
F[272] = 1;
F[221] = 1;
F[125] = 0.7873293004205568;
F[126] = 0.7873293004205568;
F[127] = 0.7873293004205568;
F[128] = 0.7873293004205568;
F[129] = 0.7873293004205568;
F[130] = 0.7873293004205568;
F[131] = 0.7873293004205568;
F[147] = 0.7287943517106497;
F[132] = 0.9861435653835764;
F[133] = 0.9885142129181279;
F[134] = 1.0590196551755877;
F[135] = -0.10506684002783058;
F[136] = -0.09934030555219196;
F[137] = -0.4772460238324262;
F[138] = 0.44966554917074764;
F[139] = -0.16039806165159287;
F[140] = 0.2538112991618278;
F[141] = -0.0916749785225;
F[142] = 0.0772469531098245;
F[143] = -0.055917204164928506;
F[144] = 0.04156153704212084;
F[145] = 0.018223994492501616;
F[146] = -0.0613057715739493;
F[148] = 0;
F[165] = 0.9185578173048153;
F[150] = 2.4254637034486395;
F[151] = 2.4229114964923784;
F[152] = 1.0449988276258664;
F[153] = 0.10646505495818319;
F[154] = 0.11502271234876896;
F[155] = 0.19371809083949765;
F[156] = 0.1432844080369784;
F[157] = 0.16703103907774355;
F[158] = 0.05802514638776715;
F[159] = -0.05066244344899647;
F[160] = 0.10275401399989069;
F[161] = 0.2266439635978286;
F[162] = 0.1217854398264814;
F[163] = 0.12359569588102835;
F[164] = 0.2819407748778785;
F[166] = 0;
F[183] = 0.9208740485941486;
F[168] = 2.4554120548101914;
F[169] = 2.4542823673696272;
F[170] = 1.0413981331260456;
F[171] = 0.07994376448509936;
F[172] = 0.2610499361033632;
F[173] = 0.23626286936152852;
F[174] = 0.2511840501224949;
F[175] = 0.07620028423961213;
F[176] = 0.09886534558913058;
F[177] = -0.06554366894585488;
F[178] = 0.09087360693903052;
F[179] = 0.21854646378757006;
F[180] = 0.09087896839902282;
F[181] = 0.1389196345704396;
F[182] = 0.1228248913652042;
F[184] = 0;
F[201] = 0.920928887113824;
F[186] = 2.4553867728238843;
F[187] = 2.4550352099063564;
F[188] = 1.0387393219526417;
F[189] = 0.22410337637293096;
F[190] = 0.12354625787123667;
F[191] = 0.132953796887261;
F[192] = 0.2229836373636006;
F[193] = 0.0054386430722970286;
F[194] = 0.10864899600192253;
F[195] = -0.006705289907107453;
F[196] = 0.04096322863458285;
F[197] = 0.17681469796575097;
F[198] = 0.16094777280406913;
F[199] = 0.21347378137529238;
F[200] = 0.16135943479237594;
F[202] = 0;
F[219] = 0.9216611823966749;
F[204] = 2.468211580182548;
F[205] = 2.4651344399046007;
F[206] = 1.0415315350404082;
F[207] = 0.10368613069884251;
F[208] = 0.25671476436154295;
F[209] = 0.16907855460610227;
F[210] = 0.21905832390295724;
F[211] = 0.16115883208616463;
F[212] = 0.1299141966900077;
F[213] = 0.02479197085255323;
F[214] = 0.05695229358562079;
F[215] = 0.1309377634658828;
F[216] = 0.12304102929931789;
F[217] = 0.16354615693290672;
F[218] = 0.1976121977305209;
F[220] = 0;
F[223] = 10.321609214345136;
F[225] = 0.7637600455637649;
F[233] = 0;
F[553] = -0.7991525211635933;
F[299] = 0.11594313594738827;
F[292] = -0.3674306289512132;
F[310] = -3.1813991999835776;
F[328] = -0.5444992903308392;
F[346] = -2.326266270041864;
F[364] = -1.169067664891122;
F[487] = 0.05257600011717986;
F[400] = 0.7343421956621061;
F[504] = 0.1415213323702264;
F[431] = 0.7158166539148201;
F[382] = -0.08406472568008522;
F[415] = -0.08246129779025332;
F[446] = 0.03489933048783891;
F[466] = 0.057088036209796814;
F[520] = -0.3716807655928572;
F[540] = -0.5409968986501394;
F[235] = 11.560637708180286;
F[237] = 0.1710960820962832;
F[245] = 0;
F[554] = 12.882984907411162;
F[317] = 0.09672589381431036;
F[293] = -0.35420100125426923;
F[311] = -3.331842895908385;
F[329] = -0.6433478490253287;
F[347] = -2.335938242770203;
F[365] = -1.2227952500418466;
F[488] = 0.184432678132318;
F[401] = 0.7343421956621061;
F[505] = 0.1706301275443359;
F[432] = 0.7158166539148201;
F[383] = 0.08809857798800345;
F[416] = -0.054266797587157196;
F[447] = 0.09243033117313623;
F[467] = 0.015289657925158403;
F[521] = -0.5236026130364146;
F[541] = -0.4535449080576362;
F[247] = 11.973640479205212;
F[249] = 0.15808929613645836;
F[257] = 0;
F[555] = 1.428667616927714;
F[335] = 0.14609209579181437;
F[294] = -0.3812732636916076;
F[312] = -3.381103939632583;
F[330] = -0.6737577658720005;
F[348] = -2.3920712361941914;
F[366] = -1.1425323641849332;
F[489] = 0.10202054221499243;
F[402] = 0.7343421956621061;
F[506] = 0.125058126557516;
F[433] = 0.7158166539148201;
F[384] = -0.054535243063263794;
F[417] = -0.07256306437701637;
F[448] = -0.015477411261584348;
F[468] = -0.017045822496196905;
F[522] = -0.41638949590436725;
F[542] = -0.48983409676635997;
F[259] = 12.318263337673297;
F[261] = 0.1377411952026283;
F[269] = 0;
F[556] = 7.565987813495022;
F[353] = 0.1598183782196702;
F[295] = -0.3422136586312305;
F[313] = -3.3750623878000168;
F[331] = -0.5647880046257416;
F[349] = -2.3012936194267777;
F[367] = -1.2281775070163996;
F[490] = 0.1615627458985328;
F[403] = 0.7343421956621061;
F[507] = 0.05186223080087783;
F[434] = 0.7158166539148201;
F[385] = -0.08406811100243346;
F[418] = -0.03594060355726987;
F[449] = 0.15519243953952921;
F[469] = 0.08312028393268038;
F[523] = -0.5087544401943888;
F[543] = -0.6028765097111515;
F[271] = 12.086593147450351;
F[273] = 0.1560919216234051;
F[281] = 0;
F[557] = 13.997645690649042;
F[371] = 0.013231576630895742;
F[296] = -0.4324197565805128;
F[314] = -3.374517296842097;
F[332] = -0.7237235254988559;
F[350] = -2.3506204063894365;
F[368] = -1.1693043064194255;
F[491] = 0.07733477659196399;
F[404] = 0.7343421956621061;
F[508] = 0.11933162328321023;
F[435] = 0.7158166539148201;
F[386] = 0.044799951978299044;
F[419] = 0.014744127521330226;
F[450] = 0.0764518883171127;
F[470] = -0.00881959305958686;
F[524] = -0.4409749455832706;
F[544] = -0.43501532627341855;
F[297] = 0.11594313594738827;
F[282] = -2.04530502520062;
F[283] = -2.0314215228548353;
F[284] = 1.9031062008547104;
F[285] = -0.42274319502488056;
F[286] = -0.5740324517869911;
F[287] = -0.6772131781141916;
F[288] = -0.202103540421125;
F[289] = -0.7038134902332176;
F[290] = -0.41472560962256233;
F[291] = -0.26469533654274346;
F[298] = 0;
F[550] = -11.094802280774184;
F[315] = 0.09672589381431036;
F[300] = -2.240177411153509;
F[301] = -2.2341449166255747;
F[302] = 4.704487265802247;
F[303] = 1.457761625950429;
F[304] = 9.463115562854114;
F[305] = 26.97834545373663;
F[306] = -28.11218976459589;
F[307] = -1.0731593937207795;
F[308] = 4.876527934506472;
F[309] = -0.4335046943968748;
F[316] = 0;
F[333] = 0.14609209579181437;
F[318] = -1.758558014306551;
F[319] = -1.7655861315311112;
F[320] = 2.582535353881392;
F[321] = -0.01898951843943727;
F[322] = 0.3985708527827582;
F[323] = 1.8285014994055966;
F[324] = -2.8456956973448775;
F[325] = 0.604459870108949;
F[326] = -1.2258271545536443;
F[327] = 0.13711601355921912;
F[334] = 0;
F[351] = 0.1598183782196702;
F[336] = -1.6631745803035698;
F[337] = -1.6595800505193328;
F[338] = 4.850984233397465;
F[339] = 0.6939359116242055;
F[340] = 5.13849993366003;
F[341] = 15.805366187807333;
F[342] = -17.19232918273676;
F[343] = -0.8553508714087131;
F[344] = 3.8829957582744132;
F[345] = -0.12517051221257539;
F[352] = 0;
F[369] = 0.013231576630895742;
F[354] = -4.303584171377903;
F[355] = -4.3118292432071215;
F[356] = 4.0360665768179995;
F[357] = 3.4906488284288346;
F[358] = 5.154221899255348;
F[359] = 16.983073356687022;
F[360] = -21.62682277037783;
F[361] = 3.0715218404374127;
F[362] = -7.378908036628108;
F[363] = -3.6724499524560885;
F[370] = 0;
F[391] = 0.7343421956621061;
F[372] = 1.0170219553790312;
F[373] = 1.016766095860912;
F[374] = 1.004367751249395;
F[375] = -0.04043554811338862;
F[376] = 0.08202614431373469;
F[377] = 0.02966349909266409;
F[378] = -0.03122397404864167;
F[379] = 0.023194367784545897;
F[380] = -0.0861943413058932;
F[381] = 0.02231862184527335;
F[387] = 0.9968836647849887;
F[388] = 0.001844434393059619;
F[389] = 0.9938590172126529;
F[390] = 0.10842699004501821;
F[392] = 0;
F[476] = 5.76929549600427;
F[480] = 0.13344088252441066;
F[481] = 0.017966553110398714;
F[482] = 0.20794751941140008;
F[483] = 0.06231493511849071;
F[484] = 0.14263245894062976;
F[485] = 0.13428650356621707;
F[486] = 0.010741065569090838;
F[478] = 1;
F[455] = 1;
F[393] = 0.7343421956621061;
F[394] = 0.7343421956621061;
F[395] = 0.7343421956621061;
F[396] = 0.7343421956621061;
F[397] = 0.7343421956621061;
F[398] = 0.7343421956621061;
F[399] = 0.7343421956621061;
F[422] = 0.7158166539148201;
F[405] = 0.9250206937384691;
F[406] = 0.9238044495498178;
F[407] = 1.0055019877193603;
F[408] = -0.058085484489682065;
F[409] = 0.07339804863033955;
F[410] = 0.07661332019137533;
F[411] = 0.04004017434885805;
F[412] = 0.09506621074177492;
F[413] = 0.015434903905965828;
F[414] = -0.06256255167183611;
F[420] = 0.050756535591154665;
F[421] = -0.017125213121848015;
F[423] = 0;
F[493] = 5.086771625873753;
F[497] = 0.14489505049002613;
F[498] = 0.04836811833548172;
F[499] = 0.16691209471975949;
F[500] = 0.1575727726670019;
F[501] = -0.016901851314449604;
F[502] = 0.15909230903746827;
F[503] = 0.09557930115237359;
F[495] = 1;
F[475] = 1;
F[424] = 0.7158166539148201;
F[425] = 0.7158166539148201;
F[426] = 0.7158166539148201;
F[427] = 0.7158166539148201;
F[428] = 0.7158166539148201;
F[429] = 0.7158166539148201;
F[430] = 0.7158166539148201;
F[453] = 0.855729564699985;
F[436] = 1.780086236734819;
F[437] = 1.7802648359242925;
F[438] = 1.0151321537559994;
F[439] = 0.013259917315093109;
F[440] = 0.09128776519080216;
F[441] = 0.09009324411782885;
F[442] = 0.0754148330501451;
F[443] = 0.021496948204083266;
F[444] = 0.0030651427302647305;
F[445] = -0.09557532367328542;
F[451] = 0.11581510396562443;
F[452] = 0.10660224635281154;
F[454] = 0;
F[473] = 0.8211437991213458;
F[456] = 1.5229923333525934;
F[457] = 1.5241161093604063;
F[458] = 1.0235754174830565;
F[459] = 0.11235124843175577;
F[460] = 0.0845594992635647;
F[461] = 0.07953347069173883;
F[462] = 0.038993321403883306;
F[463] = -0.04777774367551219;
F[464] = -0.03784516533305496;
F[465] = -0.045718938836607506;
F[471] = 0.11669631937309737;
F[472] = 0.03531850476219351;
F[474] = 0;
F[477] = 5.767976377030387;
F[479] = 0.06401901120053693;
F[492] = 0;
F[558] = 0.833834579958744;
F[529] = 0.09768154546449603;
F[525] = -0.4809057518866159;
F[545] = -0.49804090323311323;
F[494] = 5.086610570370974;
F[496] = 0.07074558975681748;
F[509] = 0;
F[559] = 0.13086438531963887;
F[549] = 0.02585217106689065;
F[526] = -0.4812918026784784;
F[546] = -0.4347099047293131;
F[527] = 0.09768154546449603;
F[510] = -2.220143042544435;
F[511] = -2.2232548601766684;
F[512] = 1.9796861306955253;
F[513] = -0.15091480695500437;
F[514] = 0.03539502010832467;
F[515] = 1.0145664085690107;
F[516] = -1.8690238465853188;
F[517] = 0.2853160176914034;
F[518] = -0.165966113326523;
F[519] = -0.17816975109542432;
F[528] = 0;
F[547] = 0.02585217106689065;
F[530] = -3.6303601369131124;
F[531] = -3.629168482669714;
F[532] = 1.5651842505580869;
F[533] = -0.31196486431946696;
F[534] = -0.3262637118705861;
F[535] = -0.18463110569733557;
F[536] = -0.9491248386010662;
F[537] = 0.032545622054968006;
F[538] = -0.30445181643517505;
F[539] = -0.15557609030758607;
F[548] = 0;
F[567] = 0.000016078512260034424;
F[551] = -11.038010740994416;
F[552] = -5.707380455065195;
F[560] = 2.8222561835972524;
F[561] = 8.206651186952461;
F[562] = 43.969062815725124;
F[563] = -55.50776152367915;
F[564] = 0.7060893722011707;
F[565] = -4.84124415179232;
F[566] = -8.702060445521225;
F[568] = 0;



  F[3] = t1;
  F[5] = t2;
  F[7] = t3;
  F[9] = t4;
  F[11] = distance;
  F[13] = pitch;
  F[15] = roll;
  
  F[0] = F[1];F[1] = F[2];F[1] += F[3] * F[4];F[1] += F[5] * F[6];F[1] += F[7] * F[8];F[1] += F[9] * F[10];F[1] += F[11] * F[12];F[1] += F[13] * F[14];F[1] += F[15] * F[16];F[1] += F[17] * F[18];F[1] += F[19] * F[20];F[1] += F[21] * F[22];F[1] += F[23] * F[24];F[1] += F[25] * F[26];F[27] = (1 / (1 + exp(-F[1])));F[28] = F[27] * (1 - F[27]);F[29] = F[27];F[30] = F[27];F[31] = F[27];F[32] = F[27];F[33] = F[27];F[34] = F[27];F[35] = F[27];
  F[36] = F[37];F[37] = F[38];F[37] += F[3] * F[39];F[37] += F[5] * F[40];F[37] += F[7] * F[41];F[37] += F[9] * F[42];F[37] += F[11] * F[43];F[37] += F[13] * F[44];F[37] += F[15] * F[45];F[37] += F[17] * F[46];F[37] += F[19] * F[47];F[37] += F[21] * F[48];F[37] += F[23] * F[49];F[37] += F[25] * F[50];F[51] = (1 / (1 + exp(-F[37])));F[52] = F[51] * (1 - F[51]);F[53] = F[51];F[54] = F[51];F[55] = F[51];F[56] = F[51];F[57] = F[51];F[58] = F[51];F[59] = F[51];
  F[60] = F[61];F[61] = F[62];F[61] += F[3] * F[63];F[61] += F[5] * F[64];F[61] += F[7] * F[65];F[61] += F[9] * F[66];F[61] += F[11] * F[67];F[61] += F[13] * F[68];F[61] += F[15] * F[69];F[61] += F[17] * F[70];F[61] += F[19] * F[71];F[61] += F[21] * F[72];F[61] += F[23] * F[73];F[61] += F[25] * F[74];F[75] = (1 / (1 + exp(-F[61])));F[76] = F[75] * (1 - F[75]);F[77] = F[75];F[78] = F[75];F[79] = F[75];F[80] = F[75];F[81] = F[75];F[82] = F[75];F[83] = F[75];
  F[84] = F[85];F[85] = F[86];F[85] += F[3] * F[87];F[85] += F[5] * F[88];F[85] += F[7] * F[89];F[85] += F[9] * F[90];F[85] += F[11] * F[91];F[85] += F[13] * F[92];F[85] += F[15] * F[93];F[85] += F[17] * F[94];F[85] += F[19] * F[95];F[85] += F[21] * F[96];F[85] += F[23] * F[97];F[85] += F[25] * F[98];F[99] = (1 / (1 + exp(-F[85])));F[100] = F[99] * (1 - F[99]);F[101] = F[99];F[102] = F[99];F[103] = F[99];F[104] = F[99];F[105] = F[99];F[106] = F[99];F[107] = F[99];
  F[108] = F[109];F[109] = F[110];F[109] += F[3] * F[111];F[109] += F[5] * F[112];F[109] += F[7] * F[113];F[109] += F[9] * F[114];F[109] += F[11] * F[115];F[109] += F[13] * F[116];F[109] += F[15] * F[117];F[109] += F[17] * F[118];F[109] += F[19] * F[119];F[109] += F[21] * F[120];F[109] += F[23] * F[121];F[109] += F[25] * F[122];F[123] = (1 / (1 + exp(-F[109])));F[124] = F[123] * (1 - F[123]);F[125] = F[123];F[126] = F[123];F[127] = F[123];F[128] = F[123];F[129] = F[123];F[130] = F[123];F[131] = F[123];
  F[132] = F[133];F[133] = F[134];F[133] += F[3] * F[135];F[133] += F[5] * F[136];F[133] += F[7] * F[137];F[133] += F[9] * F[138];F[133] += F[11] * F[139];F[133] += F[13] * F[140];F[133] += F[15] * F[141];F[133] += F[17] * F[142];F[133] += F[19] * F[143];F[133] += F[21] * F[144];F[133] += F[23] * F[145];F[133] += F[25] * F[146];F[147] = (1 / (1 + exp(-F[133])));F[148] = F[147] * (1 - F[147]);F[149] = F[147];
  F[150] = F[151];F[151] = F[152];F[151] += F[3] * F[153];F[151] += F[5] * F[154];F[151] += F[7] * F[155];F[151] += F[9] * F[156];F[151] += F[11] * F[157];F[151] += F[13] * F[158];F[151] += F[15] * F[159];F[151] += F[17] * F[160];F[151] += F[19] * F[161];F[151] += F[21] * F[162];F[151] += F[23] * F[163];F[151] += F[25] * F[164];F[165] = (1 / (1 + exp(-F[151])));F[166] = F[165] * (1 - F[165]);F[167] = F[165];
  F[168] = F[169];F[169] = F[170];F[169] += F[3] * F[171];F[169] += F[5] * F[172];F[169] += F[7] * F[173];F[169] += F[9] * F[174];F[169] += F[11] * F[175];F[169] += F[13] * F[176];F[169] += F[15] * F[177];F[169] += F[17] * F[178];F[169] += F[19] * F[179];F[169] += F[21] * F[180];F[169] += F[23] * F[181];F[169] += F[25] * F[182];F[183] = (1 / (1 + exp(-F[169])));F[184] = F[183] * (1 - F[183]);F[185] = F[183];
  F[186] = F[187];F[187] = F[188];F[187] += F[3] * F[189];F[187] += F[5] * F[190];F[187] += F[7] * F[191];F[187] += F[9] * F[192];F[187] += F[11] * F[193];F[187] += F[13] * F[194];F[187] += F[15] * F[195];F[187] += F[17] * F[196];F[187] += F[19] * F[197];F[187] += F[21] * F[198];F[187] += F[23] * F[199];F[187] += F[25] * F[200];F[201] = (1 / (1 + exp(-F[187])));F[202] = F[201] * (1 - F[201]);F[203] = F[201];
  F[204] = F[205];F[205] = F[206];F[205] += F[3] * F[207];F[205] += F[5] * F[208];F[205] += F[7] * F[209];F[205] += F[9] * F[210];F[205] += F[11] * F[211];F[205] += F[13] * F[212];F[205] += F[15] * F[213];F[205] += F[17] * F[214];F[205] += F[19] * F[215];F[205] += F[21] * F[216];F[205] += F[23] * F[217];F[205] += F[25] * F[218];F[219] = (1 / (1 + exp(-F[205])));F[220] = F[219] * (1 - F[219]);F[221] = F[219];
  F[222] = F[223];F[223] = F[149] * F[224] * F[223] + F[225];F[223] += F[3] * F[226] * F[29];F[223] += F[5] * F[227] * F[30];F[223] += F[7] * F[228] * F[31];F[223] += F[9] * F[229] * F[32];F[223] += F[11] * F[230] * F[33];F[223] += F[13] * F[231] * F[34];F[223] += F[15] * F[232] * F[35];F[17] = (1 / (1 + exp(-F[223])));F[233] = F[17] * (1 - F[17]);
  F[234] = F[235];F[235] = F[167] * F[236] * F[235] + F[237];F[235] += F[3] * F[238] * F[53];F[235] += F[5] * F[239] * F[54];F[235] += F[7] * F[240] * F[55];F[235] += F[9] * F[241] * F[56];F[235] += F[11] * F[242] * F[57];F[235] += F[13] * F[243] * F[58];F[235] += F[15] * F[244] * F[59];F[19] = (1 / (1 + exp(-F[235])));F[245] = F[19] * (1 - F[19]);
  F[246] = F[247];F[247] = F[185] * F[248] * F[247] + F[249];F[247] += F[3] * F[250] * F[77];F[247] += F[5] * F[251] * F[78];F[247] += F[7] * F[252] * F[79];F[247] += F[9] * F[253] * F[80];F[247] += F[11] * F[254] * F[81];F[247] += F[13] * F[255] * F[82];F[247] += F[15] * F[256] * F[83];F[21] = (1 / (1 + exp(-F[247])));F[257] = F[21] * (1 - F[21]);
  F[258] = F[259];F[259] = F[203] * F[260] * F[259] + F[261];F[259] += F[3] * F[262] * F[101];F[259] += F[5] * F[263] * F[102];F[259] += F[7] * F[264] * F[103];F[259] += F[9] * F[265] * F[104];F[259] += F[11] * F[266] * F[105];F[259] += F[13] * F[267] * F[106];F[259] += F[15] * F[268] * F[107];F[23] = (1 / (1 + exp(-F[259])));F[269] = F[23] * (1 - F[23]);
  F[270] = F[271];F[271] = F[221] * F[272] * F[271] + F[273];F[271] += F[3] * F[274] * F[125];F[271] += F[5] * F[275] * F[126];F[271] += F[7] * F[276] * F[127];F[271] += F[9] * F[277] * F[128];F[271] += F[11] * F[278] * F[129];F[271] += F[13] * F[279] * F[130];F[271] += F[15] * F[280] * F[131];F[25] = (1 / (1 + exp(-F[271])));F[281] = F[25] * (1 - F[25]);
  F[282] = F[283];F[283] = F[284];F[283] += F[3] * F[285];F[283] += F[5] * F[286];F[283] += F[7] * F[287];F[283] += F[9] * F[288];F[283] += F[11] * F[289];F[283] += F[13] * F[290];F[283] += F[15] * F[291];F[283] += F[17] * F[292];F[283] += F[19] * F[293];F[283] += F[21] * F[294];F[283] += F[23] * F[295];F[283] += F[25] * F[296];F[297] = (1 / (1 + exp(-F[283])));F[298] = F[297] * (1 - F[297]);F[299] = F[297];
  F[300] = F[301];F[301] = F[302];F[301] += F[3] * F[303];F[301] += F[5] * F[304];F[301] += F[7] * F[305];F[301] += F[9] * F[306];F[301] += F[11] * F[307];F[301] += F[13] * F[308];F[301] += F[15] * F[309];F[301] += F[17] * F[310];F[301] += F[19] * F[311];F[301] += F[21] * F[312];F[301] += F[23] * F[313];F[301] += F[25] * F[314];F[315] = (1 / (1 + exp(-F[301])));F[316] = F[315] * (1 - F[315]);F[317] = F[315];
  F[318] = F[319];F[319] = F[320];F[319] += F[3] * F[321];F[319] += F[5] * F[322];F[319] += F[7] * F[323];F[319] += F[9] * F[324];F[319] += F[11] * F[325];F[319] += F[13] * F[326];F[319] += F[15] * F[327];F[319] += F[17] * F[328];F[319] += F[19] * F[329];F[319] += F[21] * F[330];F[319] += F[23] * F[331];F[319] += F[25] * F[332];F[333] = (1 / (1 + exp(-F[319])));F[334] = F[333] * (1 - F[333]);F[335] = F[333];
  F[336] = F[337];F[337] = F[338];F[337] += F[3] * F[339];F[337] += F[5] * F[340];F[337] += F[7] * F[341];F[337] += F[9] * F[342];F[337] += F[11] * F[343];F[337] += F[13] * F[344];F[337] += F[15] * F[345];F[337] += F[17] * F[346];F[337] += F[19] * F[347];F[337] += F[21] * F[348];F[337] += F[23] * F[349];F[337] += F[25] * F[350];F[351] = (1 / (1 + exp(-F[337])));F[352] = F[351] * (1 - F[351]);F[353] = F[351];
  F[354] = F[355];F[355] = F[356];F[355] += F[3] * F[357];F[355] += F[5] * F[358];F[355] += F[7] * F[359];F[355] += F[9] * F[360];F[355] += F[11] * F[361];F[355] += F[13] * F[362];F[355] += F[15] * F[363];F[355] += F[17] * F[364];F[355] += F[19] * F[365];F[355] += F[21] * F[366];F[355] += F[23] * F[367];F[355] += F[25] * F[368];F[369] = (1 / (1 + exp(-F[355])));F[370] = F[369] * (1 - F[369]);F[371] = F[369];
  F[372] = F[373];F[373] = F[374];F[373] += F[3] * F[375];F[373] += F[5] * F[376];F[373] += F[7] * F[377];F[373] += F[9] * F[378];F[373] += F[11] * F[379];F[373] += F[13] * F[380];F[373] += F[15] * F[381];F[373] += F[17] * F[382];F[373] += F[19] * F[383];F[373] += F[21] * F[384];F[373] += F[23] * F[385];F[373] += F[25] * F[386];F[373] += F[387] * F[388];F[373] += F[389] * F[390];F[391] = (1 / (1 + exp(-F[373])));F[392] = F[391] * (1 - F[391]);F[393] = F[391];F[394] = F[391];F[395] = F[391];F[396] = F[391];F[397] = F[391];F[398] = F[391];F[399] = F[391];F[400] = F[391];F[401] = F[391];F[402] = F[391];F[403] = F[391];F[404] = F[391];
  F[405] = F[406];F[406] = F[407];F[406] += F[3] * F[408];F[406] += F[5] * F[409];F[406] += F[7] * F[410];F[406] += F[9] * F[411];F[406] += F[11] * F[412];F[406] += F[13] * F[413];F[406] += F[15] * F[414];F[406] += F[17] * F[415];F[406] += F[19] * F[416];F[406] += F[21] * F[417];F[406] += F[23] * F[418];F[406] += F[25] * F[419];F[406] += F[387] * F[420];F[406] += F[389] * F[421];F[422] = (1 / (1 + exp(-F[406])));F[423] = F[422] * (1 - F[422]);F[424] = F[422];F[425] = F[422];F[426] = F[422];F[427] = F[422];F[428] = F[422];F[429] = F[422];F[430] = F[422];F[431] = F[422];F[432] = F[422];F[433] = F[422];F[434] = F[422];F[435] = F[422];
  F[436] = F[437];F[437] = F[438];F[437] += F[3] * F[439];F[437] += F[5] * F[440];F[437] += F[7] * F[441];F[437] += F[9] * F[442];F[437] += F[11] * F[443];F[437] += F[13] * F[444];F[437] += F[15] * F[445];F[437] += F[17] * F[446];F[437] += F[19] * F[447];F[437] += F[21] * F[448];F[437] += F[23] * F[449];F[437] += F[25] * F[450];F[437] += F[387] * F[451];F[437] += F[389] * F[452];F[453] = (1 / (1 + exp(-F[437])));F[454] = F[453] * (1 - F[453]);F[455] = F[453];
  F[456] = F[457];F[457] = F[458];F[457] += F[3] * F[459];F[457] += F[5] * F[460];F[457] += F[7] * F[461];F[457] += F[9] * F[462];F[457] += F[11] * F[463];F[457] += F[13] * F[464];F[457] += F[15] * F[465];F[457] += F[17] * F[466];F[457] += F[19] * F[467];F[457] += F[21] * F[468];F[457] += F[23] * F[469];F[457] += F[25] * F[470];F[457] += F[387] * F[471];F[457] += F[389] * F[472];F[473] = (1 / (1 + exp(-F[457])));F[474] = F[473] * (1 - F[473]);F[475] = F[473];
  F[476] = F[477];F[477] = F[455] * F[478] * F[477] + F[479];F[477] += F[3] * F[480] * F[393];F[477] += F[5] * F[481] * F[394];F[477] += F[7] * F[482] * F[395];F[477] += F[9] * F[483] * F[396];F[477] += F[11] * F[484] * F[397];F[477] += F[13] * F[485] * F[398];F[477] += F[15] * F[486] * F[399];F[477] += F[17] * F[487] * F[400];F[477] += F[19] * F[488] * F[401];F[477] += F[21] * F[489] * F[402];F[477] += F[23] * F[490] * F[403];F[477] += F[25] * F[491] * F[404];F[387] = (1 / (1 + exp(-F[477])));F[492] = F[387] * (1 - F[387]);
  F[493] = F[494];F[494] = F[475] * F[495] * F[494] + F[496];F[494] += F[3] * F[497] * F[424];F[494] += F[5] * F[498] * F[425];F[494] += F[7] * F[499] * F[426];F[494] += F[9] * F[500] * F[427];F[494] += F[11] * F[501] * F[428];F[494] += F[13] * F[502] * F[429];F[494] += F[15] * F[503] * F[430];F[494] += F[17] * F[504] * F[431];F[494] += F[19] * F[505] * F[432];F[494] += F[21] * F[506] * F[433];F[494] += F[23] * F[507] * F[434];F[494] += F[25] * F[508] * F[435];F[389] = (1 / (1 + exp(-F[494])));F[509] = F[389] * (1 - F[389]);
  F[510] = F[511];F[511] = F[512];F[511] += F[3] * F[513];F[511] += F[5] * F[514];F[511] += F[7] * F[515];F[511] += F[9] * F[516];F[511] += F[11] * F[517];F[511] += F[13] * F[518];F[511] += F[15] * F[519];F[511] += F[17] * F[520];F[511] += F[19] * F[521];F[511] += F[21] * F[522];F[511] += F[23] * F[523];F[511] += F[25] * F[524];F[511] += F[387] * F[525];F[511] += F[389] * F[526];F[527] = (1 / (1 + exp(-F[511])));F[528] = F[527] * (1 - F[527]);F[529] = F[527];
  F[530] = F[531];F[531] = F[532];F[531] += F[3] * F[533];F[531] += F[5] * F[534];F[531] += F[7] * F[535];F[531] += F[9] * F[536];F[531] += F[11] * F[537];F[531] += F[13] * F[538];F[531] += F[15] * F[539];F[531] += F[17] * F[540];F[531] += F[19] * F[541];F[531] += F[21] * F[542];F[531] += F[23] * F[543];F[531] += F[25] * F[544];F[531] += F[387] * F[545];F[531] += F[389] * F[546];F[547] = (1 / (1 + exp(-F[531])));F[548] = F[547] * (1 - F[547]);F[549] = F[547];
  F[550] = F[551];F[551] = F[552];F[551] += F[17] * F[553] * F[299];F[551] += F[19] * F[554] * F[317];F[551] += F[21] * F[555] * F[335];F[551] += F[23] * F[556] * F[353];F[551] += F[25] * F[557] * F[371];F[551] += F[387] * F[558] * F[529];F[551] += F[389] * F[559] * F[549];F[551] += F[3] * F[560];F[551] += F[5] * F[561];F[551] += F[7] * F[562];F[551] += F[9] * F[563];F[551] += F[11] * F[564];F[551] += F[13] * F[565];F[551] += F[15] * F[566];F[567] = (1 / (1 + exp(-F[551])));F[568] = F[567] * (1 - F[567]);
  float output = F[567] * 100;
  return output;
}








