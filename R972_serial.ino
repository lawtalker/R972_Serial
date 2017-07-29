/***********************************************************************
 * 
 * =========================================
 * =  Sherwood Newcastle R-972 controller  =
 * =========================================
 * 
 * Copyright 2017 Michael Kwun
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 ***********************************************************************/

/* 
 *  Red Bear Lab BLE Nano (v. 1.5, but others will likely work)
 *  Rotary encoder (incremental) with pushbutton (D4-D6 on Nano)
 *  MAX3232 to convert between TTL (0/5V) and RS232 (+/-12V) logic
 *  Male DB9 to connect to serial port on the R-972
 *  5V power supply
 *  
 *  The R-972 to BLE Nano serial connection is wired straight-through;
 *  e.g., the RX pin on the R-972 is connected to RX on the BLE Nano.
 *  Using Sparkfun's MAX3232 breakout board, the connections are:
 *    T1 OUT <---> TX  [DB9]
 *    T2 OUT <---> CTS [DB9]
 *    R1 IN  <---> RX  [DB9]
 *    R2 IN  <---> RTS [DB9]
 *    VIN    <---> VDD [BLE Nano]
 *    GND    <---> GND [BLE Nano & DB9]
 *    T1 IN  <---> TX  [BLE Nano]       
 *    T2 IN  <---> CTS [BLE Nano]
 *    R1 OUT <---> RX  [BLE Nano]
 *    R2 OUT <---> RTS [BLE Nano]
 *    
 */

//#define DEBUG

#include <BLE_API.h>
 
const static int pinA = 4;    // encoder pin A
const static int pinB = 5;    // encoder pin B
const static int pinC = 6;    // pushbutton pin

BLEDevice r972;

const static uint8_t beaconPayload[] = {
  0x4C,0x00,                                                                       // Company Identifier Code = Apple
  0x02,                                                                            // Type = iBeacon
  0x15,                                                                            // Following data length
  0x17,0xcf,0xbf,0x81,0xf7,0x51,0x4b,0x60,0x86,0x95,0xcf,0x78,0xde,0xda,0x17,0x9e, // Beacon UUID (chosen randomly
                                                                                   // 17cfbf81-f751-4b60-8695-cf78deda179e
  0x00,0x00,                                                                       // Major 0000
  0x00,0x00,                                                                       // Minor 0000
  0xC5                                                                             // Measure Power
};

 void setup() {

  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP); 
  pinMode(pinC, INPUT_PULLUP); 
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);

  r972.init(); 
  // set advertisement
  r972.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  r972.accumulateAdvertisingPayload(GapAdvertisingData::MANUFACTURER_SPECIFIC_DATA, beaconPayload, sizeof(beaconPayload));
  // set advertise type  
  //  ADV_CONNECTABLE_UNDIRECTED
  //  ADV_CONNECTABLE_DIRECTED
  //  ADV_SCANNABLE_UNDIRECTED
  //  ADV_NON_CONNECTABLE_UNDIRECTED
  r972.setAdvertisingType(GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED);
  // 100ms; in multiples of 0.625ms
  r972.setAdvertisingInterval(160); 
  // set adv_timeout, in seconds
  r972.setAdvertisingTimeout(0);
  // start advertising
  r972.startAdvertising();

  Serial.begin(9600);  

#ifdef DEBUG
  Serial.println("BEGIN R-972 Serial Controller!");
#endif

} /* end setup() */

void loop() {

#ifdef DEBUG
  const static unsigned char volUp[6] = "vol+ ";
  const static unsigned char volDown[6] = "vol- ";
  const static unsigned char muteOn[6] = "MUTE ";
  const static unsigned char muteOff[6] = "mute ";
  const static unsigned char event[6] = "evnt ";
#else
  const static byte volUp[] = { 0xaa, 0x05, 0x05, 0x21, 0x2b };
  const static byte volDown[] = { 0xaa, 0x05, 0x05, 0x22, 0x2a };
  const static byte muteOn[] = { 0xaa, 0x05, 0x05, 0x23, 0x29 };
  const static byte muteOff[] = { 0xaa, 0x05, 0x05, 0x24, 0x28 };
  const static byte event[] = { 0xaa, 0x05, 0x05, 0xff, 0x4d };
#endif

  static byte history = B1111;  // encoder - only lower nibble used
  static byte switchState = 0;  // is button down or up?
  static byte muteState = 0;    // is mute on or off?
  static byte cycleCount = 0;   // used to debounce button


  /* 
   * (1) Check push-button.
   * 
   * If the button is LOW, switch mute state, and then ignore
   * switch until it is HIGH for 2 cycles through the loop. 
   *
   */
  
  if (! switchState) {      
   
    // Check if pushbutton is pressed (LOW), and process if so.
    if ( ! digitalRead(pinC) ) {
      digitalWrite(LED,LOW);
      switchState = 1;                 // switch to debounce mode
      muteState ^= 1;                  // toggle mute state 
      if (muteState) {
        Serial.write(muteOn,5);
      }
      else {
        Serial.write(muteOff,5);
      }
    }
  }
  
  // If debouncing after a pushbutton press...
  else {
    // Wait until pushbutton settles at button HIGH (off) for 2 
    // cycles, which is enough to debounce due to the loop delay.
    if  (! digitalRead(pinC) )
      cycleCount = 0;
    else if ( ++cycleCount == 2 ) {
      switchState = cycleCount = 0;
      digitalWrite(LED,HIGH);
    }
  }

  /* (2) Read rotary encoder
   * 
   * We use some low-level bit magic to read both pins together. 
   * We keep track of the current reading and the immediately prior reading 
   * this allows us to detect valid sequences, avoiding most bounce 
   * noise.
   *
   */
   
  // store old state in bits 1 & 2
  history >>= 2;                                                  

  /* 
   *  The NRF_GPIO stuff below is a bit of magic that reads both D4 and D5 at 
   *  the same time (it's the BLE Nano equivalent of port manipulation on a 
   *  regular Arduino).
   *  
   *  NRF_GPIO is a struct defined in the Arduino core for the BLE Nano
   *  and within it IN is a uint32_t that reads the general purpose I/O on the
   *  chip.  D4 and D5 are at P0_28 and P0_29.  P0_28 is defined as 28, and
   *  shifting IN (28 - 2) bits places D4 and D5 in bits 3 & 4.
   *  
   */

  // store current state in bits 1 & 2
  history |= ((NRF_GPIO->IN >> (P0_28 - 2)) & B1100);
  
  /* (3) Adjust volume
   * 
   * If light is on and the encoder has been rotated, adjust volume. 
   * 
   * Our rotary dimmer has detents once every four state changes - i.e. 
   * the detents occur after full quadtrature encoding cycles.  We 
   * therefore only look for the state changes that happen when a new 
   * detent position is reached.  Because the two pins are HIGH at the 
   * detents, this means we look for 
   *   B1101 for CW turns, and 
   *   B1110 for CCW turns.
   *
   */
   
  // If there is a CW turn to detent position...
  if ( history == B1101 ) {
    Serial.write(volUp,5);
  }

  // If there is a CCW turn to detent position...
  else if ( history == B1110 ) {
    Serial.write(volDown,5);
  }

  // debounce delay
  else {    
    delayMicroseconds(500);   
  }

} /* end loop() */
