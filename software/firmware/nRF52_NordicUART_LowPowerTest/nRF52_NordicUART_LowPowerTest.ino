 // Import libraries (BLEPeripheral depends on SPI)
 #include <SPI.h>
 #include <BLEPeripheral.h>
 #include "BLESerial.h"

 // define pins (varies per shield/board)
 #define BLE_REQ   10
 #define BLE_RDY   2
 #define BLE_RST   9

 // create ble serial instance, see pinouts above
 BLESerial BLESerial(BLE_REQ, BLE_RDY, BLE_RST);

 void setup() {
    // custom services and characteristics can be added as well
   BLESerial.setLocalName("UART");

   Serial.begin(115200);
   BLESerial.begin();

   // acceptable values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
   int power = -4;
   sd_ble_gap_tx_power_set(power);
   }

 void loop() {
   BLESerial.poll();

    __delay(1000);

   send();

   sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
   sd_app_evt_wait();
  }

  void __delay(uint32_t timeout)
  {
  uint32_t start;
  start = millis();

 do
 {
   __WFE();
 }
   while ((millis() - start) >= timeout);
 }

void send() {
if (Serial) {
int byte;
while ((byte = Serial.read()) > 0) BLESerial.write((char)byte);
}
}
