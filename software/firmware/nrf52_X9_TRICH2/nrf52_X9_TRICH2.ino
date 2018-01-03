
//NOTE: Thermopile values are now smoothed   (((Xt-2...)/2 + Xt-1)/2 + X)/2    6/14/17

/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#define NRF52

#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include <Wire.h>
//#include <math.h>
#include <KX126_SPI.h>        //accelerometer
#include <VL6180X.h>          //distance sensor


/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool  debug = true;        //turn serial on/off to get data or turn up sample rate
bool  debug_time = true;    //turn loop component time debug on/off


int   speedHz = 16; //throttled loop speed - native max loop speed is about 35 ms or 28Hz
float speedMs = 1000 / speedHz;  //native max loop speed is 62.5 ms or 16Hz
  
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
    int   command_value = 0; //controlls how device and app talk to each other

  //System
    int   varState = 0; //variable state controlled in app and appended to data stream

  //MLX90615 Thermopiles
    float TObj[4] = {0,0,0,0};
    float TAmb[4] = {0,0,0,0};
    float TAmbAv;
 
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
  command_value = 1;
  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Connected event, central: "));
    Serial.println(central.address());
  }
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  command_value = 0;
  transmittedCounter = 0; //reset NN weight transmittions counter
  if(debug){
    Serial.print(F("Disconnected event, central: "));
    Serial.println(central.address());
  }
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
  Serial.print("APP COMMAND: "); Serial.println( command_value );



  BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength());
 // if(debug) delay(1000);
  delay(10);
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
    delay(200);

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
    delay(10);

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


    /*********************** VL6180X Distance READ **************************/    
    if(debug){ Serial.println("Reading VL6180X Distance... "); } 
    float distance = 255 - (float)vl6180x.readRangeSingleMillimeters();
 //   delay(15);
    if (vl6180x.timeoutOccurred() && debug) { Serial.print(" TIMEOUT"); }
        
    if(debug_time){ Serial.print("Time after distance read: "); Serial.println( (millis() - clocktime))/1000; }
    

   /************************ Thermopile mgmt **************************/
    //MLX90615 THERMOPILE SENSORS I2C CUSTOM ADDRESSES - SMOOTHED!!!!
    for(int j = 0; j < 4; j++){
        TAmb[j] = (TAmb[j] + readAmbientTempF(j+1))/2; 
        TObj[j] = (TObj[j] + readObjectTempF(j+1))/2;
    }

    TAmbAv = (TAmb[0] + TAmb[1] + TAmb[2] + TAmb[3]) / 4;

    if(debug_time){ Serial.print("Time after thermo read: "); Serial.println( (millis() - clocktime))/1000; }

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
    }

    //Debug var state
/*    if(debug){
      Serial.print("**COMMAND: ");
      Serial.println(command_value);
    } */
  
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
              delay(10);
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

