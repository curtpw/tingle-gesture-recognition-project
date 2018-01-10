
//alternate i2c pins   sda: 15   scl: 22

//NOTE: Thermopile values are now smoothed   (((Xt-2...)/2 + Xt-1)/2 + X)/2    6/14/17

/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#include <SPI.h>
//#include <SPI2.h>
#include <BLEPeripheral.h>
#include <BLEUtil.h>
#include <Wire.h>
//#include <ArduinoLowPower.h>
//#include <KX022.h>
#include <KX022_SPI.h>
//#include <SI114x.h> 
#include <APDS9960.h>





/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool  debug = true;        //turn serial on/off to get data or turn up sample rate
bool  debug_time = false;    //turn loop component time debug on/off

float detect_objT_lowpass =    80;
float detect_objT_highpass =   102;
int   tempScaleAdjust =        12;
int   limit_stopRepeatDetect = 200;

/********************************************************************************************************/
/************************ DEFINITIONS *******************************************************************/
/********************************************************************************************************/

//#define  VIBRATE        

//#define PIN_SERIAL_RX           24
//#define PIN_SERIAL_TX           23

#define BLUE_LED_PIN              19
#define GREEN_LED_PIN             20

#define VIBRATE_PIN               25

//Accelerometer Pins
#define CS_PIN                    7

#define SPI_INTERFACES_COUNT 1

#define KX022_SDI 3
#define KX022_SDO 4
#define KX022_SCL 5
#define KX022_INT 6
#define KX022_NCS 7

#define PIN_SPI_MISO         (KX022_SDO)
#define PIN_SPI_MOSI         (KX022_SDI)
#define PIN_SPI_SCK          (KX022_SCL)
/*
#define  KX022_INT                6
#define  KX022_ADDR               4
#define  KX022_NCS                7
*/

//APDS-9960 Gesture & MLX90615 Thermopile Pins
#define  APDS9960_THERMO_SCL      22
#define  APDS9960_THERMO_SDA      15
#define  APDS9960_INT             17


//Flash Memory Pins
/*#define  MX25_SO                  27
#define  MX25_CE                  28
#define  MX25_SCK                 30
#define  MX25_SI                  31*/

//Heart Rate Pins
#define  SI1143_SDA               10u
#define  SI1143_SCL               9u

//SI1143x Operating Definitions
//#define AMBIENT_LIGHT_SAMPLING   // also samples ambient slight (slightly slower)
#define PRINT_LED_VALS             // print LED raw values
//#define GET_PULSE_READING        // prints HB and signal size

//Accelerometer Addresses
//2A -> 0x54(w) 0x55(r)   2B -> 0x56(w)0x57(r)


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
#define MLX90615_ID1              0x3C
#define MLX90615_ID2              0x3D
#define MLX90615_ID3              0x3E
#define MLX90615_ID4              0x3F


//Neural net 
/*
#define THRESHOLD 2000
#define RECOGNIZE 1
#define CALIBRATE 2

#define NET_INPUTS 3
#define NET_OUTPUTS 10
#define NET_LAYERS 2 */

//dummy LED pin for BLE
#define LED_PIN   3


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/
  //Time
  int speedHz = 8; //throttled loop speed - native max loop speed is about 35 ms or 28Hz
  float speedMs = 1000 / speedHz;  //native max loop speed is 62.5 ms or 16Hz
  
  //LED
  float blueLED_timer = 0;
  float redLED_timer = 0;

  //MLP (Multi Layer Perceptron) LSTM Neural Net
  //NN weights
   int nnLength = 500;
   float F[500];
   int transmittedCounter = 0;
   bool flag_haveNeural = false;

  //Detection
  bool  flag_detect = false;              //gesture detected!
  float   lastDetectTime = 0;
  bool flag_stopRepeatDetect = false;


  
  //Timestamp
    float clocktime;
    
  //Bluetooth
    unsigned long microsPerReading, microsPrevious;
    float accelScal;
    int command_value = 0; //controlls how device and app talk to each other

  //System
  int varState = 0; //variable state controlled in app and appended to data stream

  //MLX90615 Thermopiles
    float TObj1, TAmb1, TObj2, TAmb2, TObj3, TAmb3, TObj4, TAmb4, TAmbAv;
  
  //KX022 Accelerometer
    // pins used for the connection with the sensor
    // the other you need are controlled by the SPI library):
    const int dataReadyPin = 6;
    const int chipSelectPin = 7;

    float           acc[3];
    double          pitch;
    double          roll;

  //APDS-9960 Gesture and Proximity
    uint8_t proximity_data = 0;
    uint16_t rgb_data[] = {0,0,0};
    int isr_flag = 0;
    bool switchSensors =  false;

  //SI1143x Heart Rate
    const uint8_t HR_i2cAddr = 0x5A;
    const int SAMPLES_TO_AVERAGE = 5;             // samples for smoothing 1 to 10 seem useful 5 is default
    int binOut;             // 1 or 0 depending on state of heartbeat
    int BPM;
    unsigned long red;      // read value from visible red LED
    unsigned long _IR1;     // read value from infrared LED1
    unsigned long _IR2;     // read value from infrared LED2
    unsigned long total;    // all three LED reads added together
    int signalSize;         // the heartbeat signal minus the offset


/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/
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
BLECharacteristic        DataCharacteristic("a003", BLERead | BLENotify, 12);  //@param data - an Uint8Array.

//create streaming neural network i/o characteristic
BLECharacteristic    ReadNeuralNetCharacteristic  = BLECharacteristic("a004", BLERead | BLENotify, 20); //@param data - an Uint8Array.
BLECharacteristic    WriteNeuralNetCharacteristic  = BLECharacteristic("a005", BLEWrite, 20); //@param data - an Uint8Array.


//KX022 Accelerometer
//KX022 kx022(KX022_DEVICE_ADDRESS_1E); 
KX022_SPI kx022(CS_PIN);

//Si1143 Heart Rate
//SI114x pulse;
int c=0;

//APDS-9960 Gesture and Proximity
APDS9960 apds = APDS9960();


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
  else if(sensorNum == 1) _addr = MLX90615_I2CADDR1;   
  else if(sensorNum == 2) _addr = MLX90615_I2CADDR2;
  else if(sensorNum == 3) _addr = MLX90615_I2CADDR3;
  else _addr = MLX90615_I2CADDR4;
  
  uint16_t ret;
  Wire.beginTransmission(_addr);                  // start transmission to device 
  Wire.write(a); //delay(1);                        // sends register address to read from
  Wire.endTransmission(false);                    // end transmission
  Wire.requestFrom(_addr, (uint8_t)3);// delay(1);  // send data n-bytes read
  ret = Wire.read();// delay(1);                    // receive DATA
  ret |= Wire.read() << 8;// delay(1);              // receive DATA
  uint8_t pec = Wire.read();// delay(1);
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
/************************ APDS 9960 OPTICAL GESTURE SENSOR FUNCTIONS ************************************/
/********************************************************************************************************/

void interruptRoutine() {
  isr_flag = 1;
}


/********************************************************************************************************/
/************************ SI114X HEART RATE SENSOR FUNCTIONS ********************************************/
/********************************************************************************************************/
/*
bool isPresent() {
  Serial.println("isPresent() ");
  return( getReg(0x00)==0x42);
}

byte getReg (byte reg) {
Serial.println("byte getReg (byte reg) ");
    // get a register
   // Wire.endTransmission(); delay(1);
    Serial.println("Wire.beginTransmission(0x5A); ");
    Wire.beginTransmission(HR_i2cAddr); delay(1);
    Serial.println("Wire.write(reg);");
    Wire.write(reg); delay(3);
    Serial.println("Wire.endTransmission(); ");
    Wire.endTransmission(); delay(3);
    Serial.println("Wire.requestFrom(0x5A, 1); ");
   // requestData(1);
    Wire.requestFrom(HR_i2cAddr, 1); delay(1);
    Serial.println("byte getReg (byte reg) ");
    byte result = Wire.read();
    delay(10); // XXX Nothing in datasheet indicates this is required; was in original code.
    return result;
}
*/
/*
// simple smoothing function for  heartbeat detection and processing
float smooth(float data, float filterVal, float smoothedVal){

    if (filterVal > 1){      // check to make sure param's are within range
        filterVal = .99;
    }
    else if (filterVal <= 0.0){
        filterVal = 0.01;
    }

    smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);
    return smoothedVal;
}


void readPulseSensor(){

    static int foundNewFinger;
    static  int valley=0, peak=0, smoothPeak, smoothValley, binOut, lastBinOut, BPM;
    static unsigned long lastTotal, lastMillis,  valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime=millis(), lastBeat, beat;
    static float baseline, HFoutput, HFoutput2, shiftedOutput, LFoutput, hysterisis;

    unsigned long total=0, start;
    int i=0;
    int signalSize;
    red = 0;
    _IR1 = 0;
    _IR2 = 0;
    total = 0;

    Serial.println("begin readPulseSensor()");

  #ifdef AMBIENT_LIGHT_SAMPLING
    int als_vis, als_ir;
    als_vis = 0;
    als_ir = 0;
  #endif

    start = millis();
         
    
   #ifdef POWERLINE_SAMPLING
     
     while (millis() - start < 16){   // 60 hz - or use 33 for two cycles
     // 50 hz in Europe use 20, or 40
       Serial.print("sample");
   #else     
     while (i < SAMPLES_TO_AVERAGE){      
   #endif


   #ifdef AMBIENT_LIGHT_SAMPLING
     uint16_t* ambientLight = pulse.fetchALSData();
     als_vis += ambientLight[0];
     als_ir += ambientLight[1];
   #endif

         Serial.println("before pulse.fetchLedData()");
     uint16_t* ledValues = pulse.fetchLedData();

         Serial.println("after pulse.fetchLedData()");

     red += ledValues[0];
     _IR1 += ledValues[1];
     _IR2 += ledValues[2];
     i++;
     }
     
    red = red / i;  // get averages
    _IR1 = _IR1 / i;
    _IR2 = _IR2 / i;
    total = red + _IR1 + _IR2;
    
  #ifdef AMBIENT_LIGHT_SAMPLING

    als_vis = als_vis / i;
    als_ir = als_ir / i;

    Serial.print(als_vis);       //  ambient visible
    Serial.print("\t");
    Serial.print(als_ir);        //  ambient IR
    Serial.print("\t");
  #endif

  #ifdef PRINT_LED_VALS
    Serial.print(red);
    Serial.print("\t");
    Serial.print(_IR1);
    Serial.print("\t");
    Serial.print(_IR2);
    Serial.print("\t");
    Serial.println((long)total);   
  #endif
/*
  #ifdef SEND_TOTAL_TO_PROCESSING
    Serial.println(total);
  #endif

  #ifdef GET_PULSE_READING
    // except this one for Processing heartbeat monitor
    // comment out all the bottom print lines

    if (lastTotal < 20000L && total > 20000L) {
        foundNewFinger = 1;  // found new finger!
        Serial.println("found new finger");
    }

    lastTotal = total;

    // if found a new finger prime filters first 20 times through the loop
    if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover 

    if ( foundNewFinger < 20){
        baseline = total - 200;   // take a guess at the baseline to prime smooth filter
    }
    else if(total > 20000L) {    // main running function
        // baseline is the moving average of the signal - the middle of the waveform
        // the idea here is to keep track of a high frequency signal, HFoutput and a 
        // low frequency signal, LFoutput
        // The HF signal is shifted downward slightly (heartbeats are negative peaks)
        // The high freq signal has some hysterisis added. When the HF signal is less than the 
        // shifted LF signal, we have found a heartbeat.
        baseline = smooth(total, 0.99, baseline);   // 
        HFoutput = smooth((total - baseline), 0.2, HFoutput);    // recycling output - filter to slow down response
        HFoutput2 = HFoutput + hysterisis;     
        LFoutput = smooth((total - baseline), 0.95, LFoutput);
        // heartbeat signal is inverted - we are looking for negative peaks
        shiftedOutput = LFoutput - (signalSize * .05);

        // We need to be able to keep track of peaks and valleys to scale the output for 
        // user convenience. Hysterisis is also scaled.
        if (HFoutput  > peak) peak = HFoutput; 
        if (peak > 1500) peak = 1500; 

        if (millis() - lastPeakTime > 1800){  // reset peak detector slower than lowest human HB
            smoothPeak =  smooth((float)peak, 0.6, (float)smoothPeak);  // smooth peaks
            peak = 0;
            lastPeakTime = millis();
        }

        if (HFoutput  < valley)   valley = HFoutput;
        if (valley < -1500) valley = -1500;

        if (millis() - lastValleyTime > 1800){  // reset valleys detector slower than lowest human HB
            smoothValley =  smooth((float)valley, 0.6, (float)smoothValley);  // smooth valleys
            valley = 0;
            lastValleyTime = millis();           
        }

        signalSize = smoothPeak - smoothValley;  // this the size of the smoothed HF heartbeat signal
        
        // Serial.print(" T  ");
        // Serial.print(signalSize); 

        if(HFoutput2 < shiftedOutput){
            lastBinOut = binOut;
            binOut = 1;
         //   Serial.println("\t1");
            hysterisis = - constrain((signalSize / 15), 35, 120) ;   // you might want to divide by smaller number
            // if you start getting "double bumps"
        } else {
         //   Serial.println("\t0");
            lastBinOut = binOut;
            binOut = 0;
            hysterisis = constrain((signalSize / 15), 35, 120);    // ditto above
            
        } 

        if (lastBinOut == 1 && binOut == 0){
          Serial.println(binOut);
        }

        if (lastBinOut == 0 && binOut == 1){
            lastBeat = beat;
            beat = millis();
            BPM = 60000 / (beat - lastBeat);
            Serial.print(binOut);
            Serial.print("\t BPM ");
            Serial.print(BPM);  
            Serial.print("\t signal size ");
            Serial.println(signalSize);  
        }
    }
 #endif 
}
*/
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

void bleNeuralValueUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
  String bleRawVal = "";
  
  for (byte i = 0; i < the_length; i++){ 
    String bleRawVal_temp = String(the_buffer[i], HEX); 
    if(bleRawVal_temp.length() == 1)  bleRawVal += String("0");
    bleRawVal += bleRawVal_temp;
  }
 if(debug){ Serial.print("BLERAWVAL: "); Serial.println(bleRawVal); }

  char neuro_index_buffer1[3];
  char neuro_index_buffer2[3];
  char neuro_index_buffer3[3];
  char neuro_index_buffer4[3];
  char neuro_index_buffer5[3];
  char neuro_index_buffer6[3];
  char neuro_index_buffer7[3];
  char neuro_index_buffer8[3];
  char neuro_index_buffer9[3];
  char neuro_index_buffer10[3];
  char neuro_index_buffer11[3];
  char neuro_index_buffer12[3];
  char neuro_index_buffer13[3];
  char neuro_index_buffer14[3];
  char neuro_index_buffer15[3];
  char neuro_index_buffer16[3];
  char neuro_index_buffer17[3];
  char neuro_index_buffer18[3];
  char neuro_index_buffer19[3];
  char neuro_index_buffer20[3];
  
  bleRawVal.substring(0,2).toCharArray(neuro_index_buffer1, 3);
  bleRawVal.substring(2,4).toCharArray(neuro_index_buffer2, 3);
  bleRawVal.substring(4,6).toCharArray(neuro_index_buffer3, 3);
  bleRawVal.substring(6,8).toCharArray(neuro_index_buffer4, 3);
  bleRawVal.substring(8,10).toCharArray(neuro_index_buffer5, 3);
  bleRawVal.substring(10,12).toCharArray(neuro_index_buffer6, 3);
  bleRawVal.substring(12,14).toCharArray(neuro_index_buffer7, 3);
  bleRawVal.substring(14,16).toCharArray(neuro_index_buffer8, 3);
  bleRawVal.substring(16,18).toCharArray(neuro_index_buffer9, 3);
  bleRawVal.substring(18,20).toCharArray(neuro_index_buffer10, 3);
  bleRawVal.substring(20,22).toCharArray(neuro_index_buffer11, 3);
  bleRawVal.substring(22,24).toCharArray(neuro_index_buffer12, 3);
  bleRawVal.substring(24,26).toCharArray(neuro_index_buffer13, 3);
  bleRawVal.substring(26,28).toCharArray(neuro_index_buffer14, 3);
  bleRawVal.substring(28,30).toCharArray(neuro_index_buffer15, 3);
  bleRawVal.substring(30,32).toCharArray(neuro_index_buffer16, 3);
  bleRawVal.substring(32,34).toCharArray(neuro_index_buffer17, 3);
  bleRawVal.substring(34,36).toCharArray(neuro_index_buffer18, 3);
  bleRawVal.substring(36,38).toCharArray(neuro_index_buffer19, 3);
  bleRawVal.substring(38,40).toCharArray(neuro_index_buffer20, 3);
  
  int neuro_num1 = (int)strtol(neuro_index_buffer1, NULL, 16);
  int neuro_num2 = (int)strtol(neuro_index_buffer2, NULL, 16);
  int neuro_num3 = (int)strtol(neuro_index_buffer3, NULL, 16);
  float neuro_num4 = (float)strtol(neuro_index_buffer4, NULL, 16);
  float neuro_num5 = (float)strtol(neuro_index_buffer5, NULL, 16);
  float neuro_num6 = (float)strtol(neuro_index_buffer6, NULL, 16);
  float neuro_num7 = (float)strtol(neuro_index_buffer7, NULL, 16);
  float neuro_num8 = (float)strtol(neuro_index_buffer8, NULL, 16);
  float neuro_num9 = (float)strtol(neuro_index_buffer9, NULL, 16);
  float neuro_num10 = (float)strtol(neuro_index_buffer10, NULL, 16);
  float neuro_num11 = (int)strtol(neuro_index_buffer11, NULL, 16);
  float neuro_num12 = (int)strtol(neuro_index_buffer12, NULL, 16);
  float neuro_num13 = (int)strtol(neuro_index_buffer13, NULL, 16);
  float neuro_num14 = (float)strtol(neuro_index_buffer14, NULL, 16);
  float neuro_num15 = (float)strtol(neuro_index_buffer15, NULL, 16);
  float neuro_num16 = (float)strtol(neuro_index_buffer16, NULL, 16);
  float neuro_num17 = (float)strtol(neuro_index_buffer17, NULL, 16);
  float neuro_num18 = (float)strtol(neuro_index_buffer18, NULL, 16);
  float neuro_num19 = (float)strtol(neuro_index_buffer19, NULL, 16);
  float neuro_num20 = (float)strtol(neuro_index_buffer20, NULL, 16);

 if(debug){
  Serial.print("BASE10: "); Serial.print( neuro_num1 ); 
  Serial.print(" , "); Serial.print( neuro_num2 );
  Serial.print(" , "); Serial.print( neuro_num3 );
  Serial.print(" , "); Serial.print( neuro_num4 );
  Serial.print(" , "); Serial.print( neuro_num5 );
  Serial.print(" , "); Serial.print( neuro_num6 );
  Serial.print(" , "); Serial.print( neuro_num7 );
  Serial.print(" , "); Serial.print( neuro_num8 );
  Serial.print(" , "); Serial.print( neuro_num9 );
  Serial.print(" , "); Serial.print( neuro_num10 );
  Serial.print(" , "); Serial.print( neuro_num11 ); 
  Serial.print(" , "); Serial.print( neuro_num12 );
  Serial.print(" , "); Serial.print( neuro_num13 );
  Serial.print(" , "); Serial.print( neuro_num14 );
  Serial.print(" , "); Serial.print( neuro_num15 );
  Serial.print(" , "); Serial.print( neuro_num16 );
  Serial.print(" , "); Serial.print( neuro_num17 );
  Serial.print(" , "); Serial.print( neuro_num18 );
  Serial.print(" , "); Serial.print( neuro_num19 );
  Serial.print(" , "); Serial.println( neuro_num20 );
 }
  
  int neuro_value_index = ( neuro_num1 * 100) + neuro_num2;

  float neuro_value_weight1 = neuro_num4*10 + neuro_num5/10 + neuro_num6/1000 + neuro_num7/100000;
  float neuro_value_weight2 = neuro_num8*10 + neuro_num9/10 + neuro_num10/1000 + neuro_num11/100000;
  float neuro_value_weight3 = neuro_num12*10 + neuro_num13/10 + neuro_num14/1000 + neuro_num15/100000;
  float neuro_value_weight4 = neuro_num16*10 + neuro_num17/10 + neuro_num18/1000 + neuro_num19/100000;

  //apply negative sign to weight where needed 
  //the four neuron signs are represented as a binary array converted to base 10 number in the third packet character  pos/neg,pos/net,pos/neg,pos/neg
  float posNeg[4] = {1,1,1,1};

  if(     neuro_num3 ==  1){ posNeg[3] = -1;}                                                    //0001
  else if(neuro_num3 ==  2){ posNeg[2] = -1; }                                                   //0010
  else if(neuro_num3 ==  3){ posNeg[2] = -1; posNeg[3] = -1;}                                    //0011
  else if(neuro_num3 ==  4){ posNeg[1] = -1;}                                                    //0100
  else if(neuro_num3 ==  5){ posNeg[1] = -1; posNeg[3] = -1;}                                    //0101
  else if(neuro_num3 ==  6){ posNeg[1] = -1; posNeg[2] = -1;}                                    //0110
  else if(neuro_num3 ==  7){ posNeg[1] = -1; posNeg[2] = -1; posNeg[3] = -1;}                    //0111
  else if(neuro_num3 ==  8){ posNeg[0] = -1; }                                                   //1000
  else if(neuro_num3 ==  9){ posNeg[0] = -1; posNeg[3] = -1;}                                    //1001
  else if(neuro_num3 == 10){ posNeg[0] = -1;  posNeg[2] = -1;}                                   //1010
  else if(neuro_num3 == 11){ posNeg[0] = -1; posNeg[2] = -1; posNeg[3] = -1;}                    //1011
  else if(neuro_num3 == 12){ posNeg[0] = -1; posNeg[1] = -1;}                                    //1100
  else if(neuro_num3 == 13){ posNeg[0] = -1; posNeg[1] = -1; posNeg[3] = -1;}                    //1101
  else if(neuro_num3 == 14){ posNeg[0] = -1; posNeg[1] = -1; posNeg[2] = -1;}                    //1110
  else if(neuro_num3 == 15){ posNeg[0] = -1; posNeg[1] = -1; posNeg[2] = -1; posNeg[3] = -1;}    //1111

  neuro_value_weight1 = neuro_value_weight1*posNeg[0];
  neuro_value_weight2 = neuro_value_weight2*posNeg[1];
  neuro_value_weight3 = neuro_value_weight3*posNeg[2];
  neuro_value_weight4 = neuro_value_weight4*posNeg[3];

 if(debug){
  Serial.print("NEURO_INDEX_VALUES: "); Serial.print( neuro_value_index ); Serial.print(" , "); Serial.print( neuro_value_index + 1 ); Serial.print(" , "); Serial.print( neuro_value_index + 2 ); Serial.print(" , "); Serial.println( neuro_value_index + 3 );
  Serial.print("NEURO_WEIGHT_VALUE: "); Serial.print( neuro_value_weight1 , 7 ); Serial.print(" , "); Serial.print( neuro_value_weight2 , 7 ); Serial.print(" , "); Serial.print( neuro_value_weight3 , 7 ); Serial.print(" , ");Serial.println( neuro_value_weight4 , 7 );      
 }

  //add neuron weight to NN network weight array
  if( (neuro_value_index + 4) < 500){
      F[neuro_value_index] = neuro_value_weight1;
      F[neuro_value_index + 1] = neuro_value_weight2;
      F[neuro_value_index + 2] = neuro_value_weight3;
      F[neuro_value_index + 3] = neuro_value_weight4;
  }

  //count # of times weights have been trasnmitted
  transmittedCounter = transmittedCounter + 4;
  
  //** RECEIVE NEURAL NETWORK COMPLETE - HAVE NN - ERROR CHECKING - give up if over 1000
  if(transmittedCounter > 994 && flag_haveNeural == false){
      flag_haveNeural = true;
      //check weights
      for(int e = 0; e < 495; e++){
          if(F[e] > 98){ 
            flag_haveNeural = false;
            if(debug){ Serial.print("!ERROR AT NEURON# "); Serial.print( e ); Serial.print("   VAL "); Serial.println( F[e] ); }
          }
      }
      if(flag_haveNeural == true && debug){ for(int e = 0; e < 495; e++){ Serial.print("++NEURON #: "); Serial.print( e ); Serial.print("   VAL: "); Serial.println( F[e] );  } } //display completed NN
  }
  
  if(debug){ Serial.print("# TRANSMITS: "); Serial.println( transmittedCounter ); }

  if(debug){ BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength()); }
 // if(debug) delay(1000);
  delay(10);
}

void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print(F("Characteristic event, writen: "));

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

    // start the SPI library:
//    SPI.begin();
//    delay(50);

   //Configure display LED pins
    pinMode(BLUE_LED_PIN, OUTPUT); 
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(BLUE_LED_PIN, 0);
    digitalWrite(GREEN_LED_PIN, 0);

   //configure haptic feedback pin
    pinMode(VIBRATE_PIN, OUTPUT);  digitalWrite(VIBRATE_PIN, 0);
  
  /************ INIT KX022 ACCELEROMETER *****************************/
      Serial.print("KX002 INIT RESPONSE WAS ");
      Serial.println(kx022.init());
      delay(3000);

  /************ INIT APDS-9960 Gesture and Proximity ******************/
  //SWITCH I2C PINS --> GESTURE/THERMO
  /*  Wire.end();
    delay(5);
    Wire.begin(15, 22); //sda,scl
    delay(10);
    Serial.println("After switch I2C to 15/22 pre APDS9960 init"); */
    
    if ( apds.init() ) {
      if(debug) Serial.println("APDS-9960 INITIALIZATION COMPLETE");
    } else {
      if(debug) Serial.println(F("Something went wrong during APDS-9960 init!"));
    } 

    // Adjust the Proximity sensor gain
    if ( !apds.setProximityGain(PGAIN_8X) ) {
      if(debug) Serial.println("Something went wrong trying to set PGAIN");
    }


  // Start running the APDS-9960 proximity sensor (no interrupts)
    if ( apds.enableProximitySensor(false) ) {
      if(debug) Serial.println("PROXIMITY SENSOR NOW RUNNING");
    } else {
      if(debug) Serial.println("Something went wrong during proximity sensor init!");
    } 
    //Decrease LED power from degault 100mA to 50mA
    apds.setLEDDrive(2); //25mA

  // Start running the APDS-9960 RGB color sensor
    if ( apds.enableLightSensor(false) ) {
      if(debug) Serial.println("RGB COLOR SENSOR NOW RUNNING");
    } else {
      if(debug) Serial.println("Something went wrong during rgb color sensor init!");
    }


    /************ INIT SI1143 HEART RATE *******************************/ 
      //SWITCH I2C PINS --> HEART RATE
   /* Wire.end();
    delay(5);
    Wire.begin(10, 9); //sda,scl
    delay(10);
    Serial.println("After switch I2C to 10/9 pre SI114X init");
    
    if (pulse.isPresent()){ Serial.println("SI114x Pulse Sensor found"); }
    else { Serial.println("No SI114x found"); }
    pulse.initSensor(); */


  /************** INIT MX25 FLASH MEMORY ********************************/


  /************** INIT BLUETOOTH BLE instantiate BLE peripheral *********/
    // set LED pin to output mode
   // pinMode(LED_PIN, OUTPUT);
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
    
    blePeripheral.addAttribute(ReadNeuralNetCharacteristic); //NN weight i/o
    blePeripheral.addAttribute(WriteNeuralNetCharacteristic); //NN weight i/o


   // BLECharCharacteristic    ReadNeuralNetCharacteristic  = BLECharCharacteristic("a004", BLERead);
//BLECharCharacteristic    WriteNeuralNetCharacteristic = BLECharCharacteristic("a005", BLEWrite);

    // assign event handlers for connected, disconnected to peripheral
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  //  blePeripheral.setEventHandler(BLEWritten, blePeripheralServicesDiscoveredHandler);

    // assign event handlers for characteristic
    ReadOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);
    WriteOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);

    ReadNeuralNetCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleNeuralValueUpdatedHandle);
    WriteNeuralNetCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleNeuralValueUpdatedHandle);

    // assign initial values
    char readValue[10] = {0,0,0,0,0,0,0,0,0,0};
    ReadOnlyArrayGattCharacteristic.setValue(0);
    char writeValue[10] = {0,0,0,0,0,0,0,0,0,0};
    WriteOnlyArrayGattCharacteristic.setValue(0);

    char readNeuralValue[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    ReadNeuralNetCharacteristic.setValue(readNeuralValue);
    char writeNeuralValue[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    WriteNeuralNetCharacteristic.setValue(writeNeuralValue);

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

    /************ ENABLE LOW POWER MODE AND INTERRUPT *********************/
  //  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  //  sd_nvic_EnableIRQ(SWI2_IRQn);


  /************ CREATE NON VOLATILE FLASH MEMORY OBJECT *********************/
 // https://github.com/arduino-org/arduino-core-nrf52/blob/master/libraries/BLE/BLEBondStore.h
 // https://github.com/sandeepmistry/arduino-BLEPeripheral/issues/164
  BLEBondStore bondStore1 = BLEBondStore(3);  //three flash page offset
  BLEBondStore bondStore2 = BLEBondStore(4);  
  BLEBondStore bondStore3 = BLEBondStore(5);  
  BLEBondStore bondStore4 = BLEBondStore(6);  
  if(bondStore1.hasData()){ Serial.println("NEURAL NET DATA EXISTS IN STORAGE #1!!!"); }
  if(bondStore2.hasData()){ Serial.println("NEURAL NET DATA EXISTS IN STORAGE #2!!!"); }
  if(bondStore3.hasData()){ Serial.println("NEURAL NET DATA EXISTS IN STORAGE #3!!!"); }
  if(bondStore4.hasData()){ Serial.println("NEURAL NET DATA EXISTS IN STORAGE #4!!!"); }

/*
  //test flash write
  const unsigned char* testWriteData;
  unsigned char* testReadData; 
  uint8_t* testt[5] = {0xfd, 0xe9, 0x5, 0x5, 0x5};
  testWriteData = (const unsigned char*)"01234";
 // testReadData = (unsigned char*)"fffff";
  testReadData = testt;

  bondStore1.clearData();
  
  Serial.println("Write test");
  bondStore1.putData(testWriteData,0,5);                //const unsigned char* data, unsigned int offset, unsigned int length)

  Serial.println("Read test");
  bondStore1.getData(testReadData,0,5);                                        //unsigned char* data, unsigned int offset, unsigned int length);
  Serial.print("Readed: "); 
  Serial.print((uint8_t)testReadData[0]); 
  Serial.print((uint8_t)testReadData[1]); 
  Serial.print((uint8_t)testReadData[2]); 
  Serial.print((uint8_t)testReadData[3]); 
  Serial.println((uint8_t)testReadData[4]);
*/
  delay(500);
     
}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     
  /**************************** SLEEP MODE ****************************/
  //  wait for event/interrupt (low power mode)
  //  Serial.println(F("low power enter"));
  //  sd_app_evt_wait();
  //  Serial.println(F("low power exit"));
  

   /************************ LOOP SPEED CONTROL ***********************/
 if(clocktime + speedMs < millis()){
    /*************************** Timestamp ****************************/
    clocktime = millis();
    if(debug){
        Serial.println(" "); Serial.print("TIME: "); Serial.print( clocktime/1000 ); Serial.println(" s"); 
    }

    if(debug_time){ Serial.print("Time after init speed limit sheck: "); Serial.println(millis() - clocktime); }

    /******************* Bluetooth App Integration ********************/
    blePeripheral.poll(); 

   /*************************** LED mgmt *****************************/
    digitalWrite(BLUE_LED_PIN, 0); //turn on lights for demo OR high = off for old prototype
    digitalWrite(GREEN_LED_PIN, 0);
//    if(blueLED_timer > clocktime){ digitalWrite(BLUE_LED_PIN, 0); } else { digitalWrite(BLUE_LED_PIN, 1); }
//    if(redLED_timer > clocktime){ digitalWrite(GREEN_LED_PIN, 0); } else { digitalWrite(GREEN_LED_PIN, 1); }

    //SWITCH I2C FOR THERMOPILE & GESTURE/PROXIMITY 
  /*  if(flag_i2c_scl != 22){
        Wire.end();
        Wire.begin(15, 22); //sda,scl
        flag_i2c_scl = 22;
        delay(5);
    } */
    
    //MLX90615 THERMOPILE SENSORS I2C CUSTOM ADDRESSES
    //SMOOTHED!!!!
    TAmb1 = (TAmb1 + readAmbientTempF(1))/2;
    TObj1 = (TObj1 + readObjectTempF(1))/2;
    TObj2 = (TObj2 + readObjectTempF(2))/2;
    TAmb2 = (TAmb2 + readAmbientTempF(2))/2;
    TObj3 = (TObj3 + readObjectTempF(3))/2;
    TAmb3 = (TAmb3 + readAmbientTempF(3))/2;
    TObj4 = (TObj4 + readObjectTempF(4))/2;
    TAmb4 = (TAmb4 + readAmbientTempF(4))/2; 
    TAmbAv = (TAmb1 + TAmb2 + TAmb3 + TAmb4) / 4;

    if(debug_time){ Serial.print("Time after thermo read: "); Serial.println( (millis() - clocktime))/1000; }

    if(debug){
    Serial.print("AMB1: "); Serial.print( TAmb1 ); Serial.print("F");
    Serial.print("  OBJ1: "); Serial.print( TObj1 ); Serial.println("F");
    Serial.print("AMB2: "); Serial.print( TAmb2 ); Serial.print("F");
    Serial.print("  OBJ2: "); Serial.print( TObj2 ); Serial.println("F");
    Serial.print("AMB3: "); Serial.print( TAmb3 ); Serial.print("F");
    Serial.print("  OBJ3: "); Serial.print( TObj3 ); Serial.println("F");
    Serial.print("AMB4: "); Serial.print( TAmb4 ); Serial.print("F");
    Serial.print("  OBJ4: "); Serial.print( TObj4 ); Serial.println("F"); 
    Serial.print("  ACC X: "); Serial.print( acc[0] ); Serial.println("F"); 
    Serial.print("  ACC Y: "); Serial.print( acc[1] ); Serial.println("F"); 
    Serial.print("  ACC Z: "); Serial.print( acc[2] ); Serial.println("F"); 
    }


    /************ READ APDS-9960 Gesture and Proximity ******************/







        //APDS-9960 PROXIMITY READ
   // apds.setMode(3, 0); //disable low power standby
    if(switchSensors){
      //  apds.setMode(2, 1); //enable proximity detection
        apds.enableProximitySensor(false);
        
        if ( !apds.readProximity(proximity_data) ) {
            if(debug) Serial.println("Error reading proximity value");
        } else {
            if(debug) Serial.print("PROXIMITY: "); 
            if(debug) Serial.println(proximity_data);
        } 
     //   apds.setMode(2, 0); //disable proximity detection
        apds.disableProximitySensor();
        switchSensors = false;
    } else {
        //read RGB color sensors
     //   apds.setMode(1, 1); //enable rgb color detection
        apds.enableLightSensor(false);
        if ( !apds.readRedLight(rgb_data[0]) ) {
            if(debug) Serial.println("Error reading red value");
        }
        if ( !apds.readGreenLight(rgb_data[1]) ) {
            if(debug) Serial.println("Error reading green value");
        }
        if ( !apds.readBlueLight(rgb_data[2]) ) {
            if(debug) Serial.println("Error reading blue value");
        } else {
            if(debug){ 
                Serial.print("RGB COLOR: "); Serial.println(rgb_data[0]); Serial.print("\t");
                Serial.print(rgb_data[1]); Serial.print("\t");
                Serial.println(rgb_data[2]);
            }
        } 
     //   apds.setMode(1, 0); //disable rgb color detection
        apds.disableLightSensor();
        switchSensors = true;
    }
  //  apds.setMode(3, 1); //enable low power standby 
  apds.setMode(0, 0); //turn everything off
  apds.setMode(0, 1); //turn power back on

    if(debug_time){ Serial.print("Time after proximity read: "); Serial.println( (millis() - clocktime))/1000 ; }

   /******************* READ KX022 ACCELEROMETER *********************/
    //KX022 ACCELEROMETER I2C
    acc[0] = (float)kx022.getAccel(0);
    acc[1] = (float)kx022.getAccel(1);
    acc[2] = (float)kx022.getAccel(2);
    pitch = (180/3.141592) * ( atan2( acc[0], sqrt( acc[1] * acc[1] + acc[2] * acc[2])) );
    roll = (180/3.141592) * ( atan2(-acc[1], -acc[2]) );
    delay(5);
  //  Serial.println("New KX002 SPI sample done");
    /*
    Serial.print((float)kx022.getAccel(0));Serial.print(",");
    Serial.print((float)kx022.getAccel(1));Serial.print(",");
    Serial.println((float)kx022.getAccel(2));
    delay(100);
    */


    if(debug_time){ Serial.print("Time after accelerometer read: "); Serial.println( (millis() - clocktime))/1000; }

  /************ READ Si1143 HEART RATE ********************************/
  /*
    //SWITCH I2C FOR Si1143 HEART RATE 
    if(flag_i2c_scl != 9){
        Wire.end();
        Wire.begin(10, 9); //sda,scl
        flag_i2c_scl = 9;
        delay(5);
    }

    if(debug)Serial.println("Reading HR");
    readPulseSensor();  
*/
 

    /********** EVALUATE REPEAT DETECT PREVENTION ***********/
    /********** EVALUATE REPEAT DETECT PREVENTION ***********/
    if(flag_stopRepeatDetect == true){
     /*(   float difference_stopRepeatDetect = abs(target_proximity - proximity_data) + abs(target_pitch - pitch) + abs(target_roll - roll) + abs(target_Tobj1 - TAmb1) + abs(target_Tobj2 - TAmb2) + abs(target_Tobj3 - TAmb3) + abs(target_Tobj4 - TAmb4);
        //difference of current values for past detected values is great enough to start detecting again
        if(difference_stopRepeatDetect > limit_stopRepeatDetect){ 
          flag_stopRepeatDetect = false;
        } else {
          if(debug){ Serial.print("STOP REPEAT CURRENT: "); Serial.print(difference_stopRepeatDetect); Serial.print(" STOP REPEAT LIMIT: "); Serial.println(limit_stopRepeatDetect); }
        } */
    }
    
    if(debug_time){ Serial.print("Time after repeate detect notify prevention: "); Serial.println( (millis() - clocktime))/1000; }

    /********************* DETECT TARGET - BALL PARK ***********************/
    /********************* DETECT TARGET - BALL PARK ***********************/
 /*   if( (TObj1 > 84 || TObj2 > 84 || TObj3 > 84 || TObj4 > 84) && flag_stopRepeatDetect == false){   //ball park

        //increase speed when in ball park to increase responsiveness
        speedHz = 14;

        //apply low pass filter
        if(TObj1 < detect_objT_lowpass) TObj1 = detect_objT_lowpass;
        if(TObj2 < detect_objT_lowpass) TObj2 = detect_objT_lowpass;
        if(TObj3 < detect_objT_lowpass) TObj3 = detect_objT_lowpass;
        if(TObj4 < detect_objT_lowpass) TObj4 = detect_objT_lowpass; */

    /********************* LSTM NEURAL NETWORK DETECT TARGET ***********************/
    /********************* LSTM NEURAL NETWORK DETECT TARGET ***********************/
        float input[] = {(proximity_data/255), (TObj1/102), (TObj2/102), (TObj3/102), (TObj4/102)};
        double NN_time = millis();
        bool result = false;

        /************* Activate NN Function if have NN ****************/
        if(flag_haveNeural) result = detect(input);
        
        if(debug_time){ Serial.print("Time after neural network detect: "); Serial.println( (millis() - clocktime))/1000; }

    /************************** TARGET DETECTED ALERT ******************************/
    /************************** TARGET DETECTED ALERT ******************************/
    if( (lastDetectTime + 3000) > millis()) flag_stopRepeatDetect = true;
    
        if(result && flag_stopRepeatDetect == false){
          digitalWrite(VIBRATE_PIN, 1);
          delay(600);
          digitalWrite(VIBRATE_PIN, 0);
          digitalWrite(BLUE_LED_PIN, 1);
          delay(200);
          digitalWrite(BLUE_LED_PIN, 0);
          digitalWrite(GREEN_LED_PIN, 1);
          delay(200);
          digitalWrite(GREEN_LED_PIN, 0);
          lastDetectTime = millis();
        }
   

     //   float score_TObj1, score_TObj2, score_TObj3, score_TObj4, score_TObjAv, score_pitch, score_roll, score_proximity;



    //if(debug_time){ Serial.print("Time after gesture detection: "); Serial.println(millis() - clocktime); }
    /********************* END SAVE POWER BASED ON THERMOPILES *********************/
  //  } //end power save - no sensor poll if no thermo body detection
    
    //Reset detection indicators to false
    digitalWrite(VIBRATE_PIN, 0);
    digitalWrite(GREEN_LED_PIN, 0);

    //Debug var state
    if(debug){
      Serial.print("**COMMAND: ");
      Serial.println(command_value);
    }
  
/*********** Bluetooth App Integration *******************/
    unsigned long microsNow;
    
    int roll_ble = roll;
    roll_ble = roll_ble + 180;
    
    int pitch_ble = pitch;
    pitch_ble = pitch_ble + 180;

    int proximity_ble = (int)proximity_data;
    
    // check if it's time to read data and update the filter
    microsNow = micros();
    
    if(microsNow - microsPrevious >= microsPerReading){

          String strRoll = String(roll_ble);
          String strPitch = String(pitch_ble);
          String str = "temporaryvaluefffffffffffffffffff";
          

      //    str = String(roll_ble) + "," + 
         // pitch_ble;// + "," proximity_ble;

          if(debug){Serial.print("STRPITCH  STRROLL: "); Serial.print(strPitch); Serial.print(" "); Serial.println(strRoll);}
        
          BLECentral central = blePeripheral.central();
          
          if(central){ // if a central is connected to peripheral

              const unsigned char imuCharArray[12] = {
                  (uint8_t)roll_ble,
                  (uint8_t)pitch_ble,
                  (uint8_t)proximity_data,
                  //multiply by 2 etc to retain significant digits   max 255 for 8 bit
                  (uint8_t)(TObj1 * 2),
                  (uint8_t)(TObj2 * 2),
                  (uint8_t)(TObj3 * 2),
                  (uint8_t)(TObj4 * 2),
                  (uint8_t)( (acc[0] + 1) * 100),
                  (uint8_t)( (acc[1] + 1) * 100),
                  (uint8_t)( (acc[2] + 1) * 100),
                  (uint8_t)str[10],
                  (uint8_t)str[11]
              };
            //  imuChar.setValue(imuCharArray, 12); //notify central with new data 
              //send data over bluetooth
              DataCharacteristic.setValue(imuCharArray,12);
          }
  
          // increment previous time, so we keep proper pace
          microsPrevious = microsPrevious + microsPerReading;
        
     }

     if(debug_time){ Serial.print("Time after bluetooth send: "); Serial.println( (millis() - clocktime))/1000; }
  
  /*********** Bluetooth App Integration *******************/


  
    if(debug_time){ Serial.print("TIME LOOP: "); Serial.println(millis() - clocktime); }
  } //end loop speed
} //end infinate loop



bool detect(float input[]){
//** For synaptic.js app.neuralNet = new app.Architect.LSTM(5,5,2,1);  
F[3] = input[0]; 
F[5] = input[1]; 
F[7] = input[2]; 
F[9] = input[3]; 
F[11] = input[4]; 

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

  float output[2];
  output[0] = F[493];
  if(debug){ Serial.print("NN OUTPUT: "); Serial.println(output[0]); }
  
  if(output[0] > 0.8) return true;
  else return false;
  //return output;
}

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

