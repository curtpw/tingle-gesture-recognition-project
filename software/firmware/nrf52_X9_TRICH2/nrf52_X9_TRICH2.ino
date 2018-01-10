
//NOTE: Thermopile values are now smoothed   (((Xt-2...)/2 + Xt-1)/2 + X)/2    6/14/17

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
bool  debug = true;        //turn serial on/off to get data or turn up sample rate
bool  debug_time = false;    //turn loop component time debug on/off


int   speedHz = 16; //throttled loop speed - native max loop speed is about 35 ms or 28Hz
float speedMs = 1000 / speedHz;  //native max loop speed is 62.5 ms or 16Hz

int   selectNN = 0;

float detect_objT_lowpass =    80;
float detect_objT_highpass =   102;
int   tempScaleAdjust =        12;
int   limit_stopRepeatDetect = 200;
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
    int buttonState = 0;         // variable for reading the pushbutton

  //MLP (Multi Layer Perceptron) LSTM Neural Net
  //NN weights
    int     nnLength = 500;
    float   F[500];
    int     transmittedCounter = 0;
    bool    flag_haveNeural = false;

  //Detection
    bool    flag_detect = false;              //gesture detected!
    float   lastDetectTime = 0;
    bool    flag_stopRepeatDetect = false;

  //Timestamp
    float clocktime = 0;
    
  //Bluetooth
    unsigned long microsPerReading, microsPrevious;
    float accelScal;
    int   command_value = 99; //controlls how device and app talk to each other

  //System
    int   varState = 0; //variable state controlled in app and appended to data stream

  //MLX90615 Thermopiles
    float TObj[4] = {0,0,0,0};
    float TAmb[4] = {0,0,0,0};
    float TAmbAv;

  //vl6180x Distance
    float distance = 0;
 
  //KX126 Accelerometer
    // pins used for the connection with the sensor
    // the other you need are controlled by the SPI library):
    const int dataReadyPin = 6;
    const int chipSelectPin = 7;

    float           acc[3];
    double          pitch;
    double          roll;

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
  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Connected event, central: "));
    Serial.println(central.address());
  }
  delay(5);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
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
    }

    //Debug var state
/*    if(debug){
      Serial.print("**COMMAND: ");
      Serial.println(command_value);
    } */

/*********** Neural Network Gesture Recognition **********/
    if(selectNN != 0){
      int settings = 0; //dummy for now
      detectGesture(selectNN, settings, TObj[0], TObj[1], TObj[2], TObj[3], distance, pitch, roll);
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

void detectGesture(int selectNN, int settings, float t1, float t2, float t3, float t4, float distance, float pitch, float roll){
  //something will go here
  float prediction = nn_left_mouth_7521(t1, t2, t3, t4, distance, pitch, roll);
}

//LEFT HAND --> MOUTH
float nn_left_mouth_7521(float t1, float t2, float t3, float  t4, float distance, float pitch, float roll) {
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
  float output = F[567];
  return output;
}








