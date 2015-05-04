/*******************************************************************
Bluetooth LE Pushbutton mit Drehencoder und integrierter roter und grüner LED.


*******************************************************************/

#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <avr/power.h>
#include <avr/wdt.h>

// APPLICATION PARAMETERS
#define LED_AUTO_OFF 30000                         // in ms (turns off 
#define PUSHBUTTON_TURNOFF_MILLIS 5000             // in ms
#define INTERVAL_STATUS_MESSAGES 0                 // in ms (0 to stop periodic sending)
#define INTERVAL_BATTERY_MEASUREMENTS 10000        // in ms
#define ENCODER_THRESHOLD 7000                     // in µs
#define PUSHBUTTON_THRESHOLD 250                   // in ms
#define SERIAL_DEBUG 1

// PIN CONFIGURATION
#define PIN_POWER 5
#define PIN_BATTERY A5

#define PIN_LED_ARDUINO 13
#define PIN_LED_RED 10
#define PIN_LED_GREEN 9

#define PIN_PUSHBUTTON 3
#define PIN_PUSHBUTTON_BIT 0 // Arduino Pin 3 = PD0
#define PIN_ENCODER_GND 1
#define PIN_ENCODER_A 0      //
#define PIN_ENCODER_B 2      //
#define PIN_ENCODER_A_BIT 2  // Arduino Pin 0 = PD2
#define PIN_ENCODER_B_BIT 1  // Arduino Pin 2 = PD1
#define INT_ENCODER_A 2      // Arduino Pin 0 = Int 2
#define INT_ENCODER_B 1      // Arduino Pin 2 = Int 1
#define INT_PUSHBUTTON 0     // Arduino Pin 3 = Int 0

// VARIABLES
volatile uint32_t encAtime = 0, encBtime = 0, buttonTime = 0;
volatile uint8_t encAsignal = 0, encBsignal = 0, encAold = 0, encBold = 0;

volatile uint16_t halfSteps = 0;
volatile float halfStepsPerSecond = 0;
volatile float degreesPerSecond = 0;

volatile uint16_t encoderPos = 0;
volatile boolean buttonPressed = false;
volatile boolean buttonStateChanged = false;
uint32_t buttonPressedStartTime;

uint16_t batteryVoltage = 0;
uint8_t batteryPercentage = 100;
uint8_t batteryLowCounter = 0;

uint8_t brightnessRed = 0;
uint8_t brightnessGreen = 0;
uint32_t lastLEDchange = 0;

bool usbDetected = false;
volatile byte watchdogCounter;
extern volatile unsigned long timer0_millis;

ISR(WDT_vect) { increaseWatchdogCounter(); }

void increaseWatchdogCounter() {
  watchdogCounter++;
}

/***

*/
void setupPins() {
  // Ports
  pinMode(PIN_BATTERY, INPUT);
  
  pinMode(PIN_LED_ARDUINO, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  
  pinMode(PIN_PUSHBUTTON, INPUT_PULLUP);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_GND, OUTPUT);
  digitalWrite(PIN_ENCODER_GND, LOW);
}

// SETUP
void setup() {
  // Make sure we have power
  pinMode(PIN_POWER, INPUT);
  digitalWrite(PIN_POWER, HIGH);
  
  setupPins();
  
  // For Blend Micro:
  //   Default pins set to 6 and 7 for REQN and RDYN
  //   So, no need to set for Blend Micro.
  //
  //ble_set_pins(3, 2);
  
  // Set your BLE advertising name here, max. length 10
  ble_set_name("Encoder");
  
  // Enable low power consumption (interrupt based communication)
  ble_low_power();
  
  // Init. and start BLE library.
  ble_begin();
  
  // Interrupts
  attachInterrupt(INT_ENCODER_A, intEncoderA, CHANGE);
  attachInterrupt(INT_ENCODER_B, intEncoderB, CHANGE);
  attachInterrupt(INT_PUSHBUTTON, intPushbutton, FALLING);
  
  // Initial measurement
  measureBattery(true);
  
#if defined(SERIAL_DEBUG)  
  // Debug output
  Serial.begin(115200); // baudrate doesn't matter => virtual usb com port
#endif

  TXLED1;RXLED1;
  
  // Detect USB connection
  USBCON = USBCON | B00010000;
  delay(700); 
  usbDetected = !(UDINT & B00000001);
  
  // blink ;-)
  uint8_t i,g;
  for(i=0;i<3;i++) {
    for(g=0;g<250;g+=2) {
      updateLEDs(0, g);
      delay(1);
    }
    for(g=250;g>0;g-=2) {
      updateLEDs(0, g);
      delay(1);
    }
    updateLEDs(0, 0);
  }
}

void loop() {
 
  handleEncoderEvent();
  
  handleButtonEvent();

  measureBattery();

  sendStatusMessage();
  
  handleRequest();

#if defined(SERIAL_DEBUG)
  //Serial.print("Loop: "); Serial.println(millis());
  //Serial.flush();
#endif  

  ble_do_events();

  if(lastLEDchange + LED_AUTO_OFF < millis()) {
    updateLEDs(0, 0);
  }
  
  // save power (normal: ~13 mA, sleep: 3 mA)
  // first sleep / powerdown turns off SPI, UART, I2C, etc ... normal power usage after the first sleep is ~6 mA)
  if(!usbDetected                        // don't sleep if running on USB power
    && ble_can_sleep()                   // only sleep when BLE chip has nothing to do
    && brightnessRed == 0                // don't sleep if LEDs are on
    && brightnessGreen == 0
    && buttonPressed == false            // don't sleep while button is pressed
    && (micros() - encAtime > 2000000)   // encoder debounce 
    && (micros() - encBtime > 2000000)   // encoder debounce
    && (millis() - buttonTime > 2000)    // button debounce
    && millis() > 5000) {                // don't sleep in the first 5000 ms after startup

    // sleep for some time
    milliSleep(10000); //powerDown();
  }
}


void measureBattery() {
  measureBattery(false);
}

void measureBattery(boolean force) {
  static uint32_t lastMeasurement = 0;
  
  if(!force && lastMeasurement + INTERVAL_BATTERY_MEASUREMENTS > millis())
    return;
    
  // dump first read value
  analogRead(PIN_BATTERY);
    
  // voltage divider is 10k each, so voltage can be measured in a range of 0 - 6600 mV
  batteryVoltage = map(analogRead(PIN_BATTERY), 0, 1023, 0, 6600);
  batteryPercentage = constrain(map(batteryVoltage, 3300, 4100, 0, 100), 0, 100);
  
  // good enough?
  if(batteryVoltage > 3550)
    batteryPercentage = constrain(map(batteryVoltage, 3550, 4100, 50, 100), 50, 100);
  else if(batteryVoltage > 3400)  
    batteryPercentage = constrain(map(batteryVoltage, 3400, 3550, 25, 50), 25, 50);
  else
    batteryPercentage = constrain(map(batteryVoltage, 3150, 3400, 0, 25), 0, 25);
  
  if(batteryPercentage < 25 && !ble_connected()) { // 25% left
    batteryLowCounter++;
  } else {
    batteryLowCounter = 0;
  }
  
  // 10 times below 3,4 V (25%) and not connected => turn off Arduino to save power
  if(batteryLowCounter > 10 && !ble_connected()) {
    turnOff();
  }
  
  // if voltage falls below ~3,1 V Arduino turns off because the external "latched power switch" will turn off
}



void updateLEDs(uint8_t newRed, uint8_t newGreen) {
  if(brightnessRed != newRed || brightnessGreen != newGreen) {
    analogWrite(PIN_LED_RED, newRed);
    analogWrite(PIN_LED_GREEN, newGreen);
    brightnessRed = newRed;
    brightnessGreen = newGreen;

#if defined(SERIAL_DEBUG)  
    Serial.print(F("LEDs changed, new values: red = ")); Serial.print(newRed);
    Serial.print(F(", green = ")); Serial.println(newGreen);
#endif
  }

  lastLEDchange = millis();
}

void sendStatusMessage() {
  sendStatusMessage(false);
}

void sendStatusMessage(boolean force) {
  static uint32_t lastStatus = 0;
  
  if( !force && 
      (INTERVAL_STATUS_MESSAGES <= 0 || lastStatus + INTERVAL_STATUS_MESSAGES > millis()) )
    return;
  
  measureBattery(true);
  
  // BLE output
    
  if( ble_connected() ) {
    ble_write(0x00);
    ble_write((uint8_t)(encoderPos >> 8));
    ble_write((uint8_t)(encoderPos & 0xFF));
    ble_write(brightnessRed);
    ble_write(brightnessGreen);
    ble_write((uint8_t)(batteryVoltage >> 8));
    ble_write((uint8_t)(batteryVoltage & 0xFF));
    ble_write(batteryPercentage);
    ble_write(usbDetected);
  }    
  
#if defined(SERIAL_DEBUG)  
  // Serial output
  Serial.print(F("Encoder position: ")); Serial.print(encoderPos);
  Serial.print(F(", LED (red): ")); Serial.print(brightnessRed);
  Serial.print(F(", LED (green): ")); Serial.print(brightnessGreen);
  Serial.print(F(", Battery: ")); Serial.print(batteryVoltage);
  Serial.print(F("mv (")); Serial.print(batteryPercentage);
  Serial.print(F("%), running on ")); Serial.println(usbDetected ? F("USB power") : F("battery"));
#endif

  lastStatus = millis();
}

/***

*/
void handleEncoderEvent() {
  static uint16_t lastReportedPos = 0;
  static boolean ledOn = false;
  
  if (lastReportedPos != encoderPos) {
    //digitalWrite(PIN_LED_ARDUINO, ledOn ? HIGH : LOW);
    ledOn = !ledOn;
    
    // BLE output
    if( ble_connected() ) {
      uint16_t temp = degreesPerSecond;
      ble_write(0x01);
      ble_write((uint8_t)(encoderPos >> 8));
      ble_write((uint8_t)(encoderPos & 0xFF));
      ble_write((uint8_t)(temp >> 8));
      ble_write((uint8_t)(temp & 0xFF));
    }

#if defined(SERIAL_DEBUG)    
    // Serial output
    Serial.print(F("Encoder position: ")); Serial.print(encoderPos);
    Serial.print(", Speed: "); Serial.print(degreesPerSecond);
    Serial.println(" deg/s");
#endif    
    lastReportedPos = encoderPos;
  }  
  
}


/***
Handle button changes and send them over to connected devices
*/
void handleButtonEvent() {
  if(buttonPressed) {

    // still pressed?
    if(!buttonStateChanged && digitalRead(PIN_PUSHBUTTON)) {
      buttonPressed = false;
      buttonStateChanged = true;
    }

    if(buttonStateChanged) {
      buttonStateChanged = false;

      // write down time the button was pressed
      if(buttonPressed)
        buttonPressedStartTime = millis();

      // BLE output
      if( ble_connected() ) {
        ble_write(0x02); // 0x02 = button
        ble_write(buttonPressed ? 0x01 : 0x00);
      }
      
      #if defined(SERIAL_DEBUG)
      // Serial output
      if(buttonPressed)
        Serial.println(F("Button press detected"));
      else {
        Serial.print(F("Button depress detected after "));
        Serial.print(millis() - buttonPressedStartTime);
        Serial.println(F("ms"));
      }
      #endif
      
      if(buttonPressed) {
        uint8_t tempRed = brightnessRed, tempGreen = brightnessGreen;
        updateLEDs(255, 0);
        delay(100);
        updateLEDs(0, 0);
        delay(50);
        updateLEDs(tempRed, tempGreen);
      }
    }  
    
    // turn off after enough time has passed
    if(buttonPressed) {
      if(millis() - buttonPressedStartTime > PUSHBUTTON_TURNOFF_MILLIS)
        turnOff();  
    }
    
  }
  

}

/***
Handle request made by a connect device:
0x00: send status message
0x01: turn on/off LEDs
*/
void handleRequest() {
  uint8_t tempRed, tempGreen;  
  
  if ( ble_available() )
  {
    uint8_t readVal;
    while ( ble_available() )
    {
      readVal = ble_read();
      switch(readVal) {
        case 0x00:
          sendStatusMessage(true);
          break;
        case 0x01:
          tempRed = ble_read();
          tempGreen = ble_read();
          updateLEDs(tempRed, tempGreen);
          break;
        default:
          break;
      }
    }
  }
  
}


/***
First encoder pin interrupt (where the counting happens)
*/
void intEncoderA(){
  PRR0 = 0x00;  // Power Reduction Register: open timer
  PRR1 = 0x00;

  uint32_t diff = micros() - encAtime;
  if( diff < ENCODER_THRESHOLD )
    return;

  encAold = encAsignal;
  encAsignal = bitRead(PIND, PIN_ENCODER_A_BIT);
  if( encAold == encAsignal )
    return;
    
  encAtime = micros();
  if( encAsignal == encBsignal )
    halfSteps--;
  else
    halfSteps++;
    
  encoderPos = halfSteps / 2;
  halfStepsPerSecond = (1000000.0 / diff) * 0.3 + halfStepsPerSecond * 0.7;
  degreesPerSecond = halfStepsPerSecond / 48 * 360;
}

/***
Second encoder pin interrupt
*/
void intEncoderB(){
  PRR0 = 0x00;  // Power Reduction Register: open timer
  PRR1 = 0x00;

  uint32_t diff = micros() - encBtime;
  if( diff < ENCODER_THRESHOLD )
    return;
    
  encBold = encBsignal;
  encBsignal = bitRead(PIND, PIN_ENCODER_B_BIT); //digitalRead(PIN_ENCODER_B); //
  if( encBold == encBsignal )
    return;
    
  encBtime = micros();
}

/***
Pushbutton interrupt handler
*/
void intPushbutton() {
  PRR0 = 0x00;  // Power Reduction Register: open timer
  PRR1 = 0x00;
    
  if( millis() - buttonTime < PUSHBUTTON_THRESHOLD )
    return;

  buttonPressed = true;
  buttonStateChanged = true;
  buttonTime = millis();  
}

/***
Turn off Arduino
*/
void turnOff() {
  #if defined(SERIAL_DEBUG)
  Serial.println(F("Turning off ..."));
  #endif
  for(uint8_t i=0;i<3;i++) {
    updateLEDs(255,0);
    delay(200);
    updateLEDs(0,255);
    delay(200);
  }
  updateLEDs(0,0);
  digitalWrite(PIN_POWER, LOW);  
}

/***
Init watchdog https://github.com/jcw/jeelib/blob/master/Ports.cpp
*/
void watchdogInterrupts (char mode) {
  // correct for the fact that WDP3 is *not* in bit position 3!
  if (mode & bit(3))
      mode ^= bit(3) | bit(WDP3);
  // pre-calculate the WDTCSR value, can't do it inside the timed sequence
  // we only generate interrupts, no reset
  byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : 0;
  MCUSR &= ~(1<<WDRF);
  
  cli();
  
  #ifndef WDTCSR
  #define WDTCSR WDTCR
  #endif
  WDTCSR |= (1<<WDCE) | (1<<WDE); // timed sequence
  WDTCSR = wdtcsr;
  
  sei();
}


/***
Lose some time
from https://github.com/jcw/jeelib/blob/master/Ports.cpp
*/
void milliSleep(uint16_t milliseconds) {
  uint16_t timeleft = milliseconds;

  while(timeleft >= 16) {
    char wdp = 0; // wdp 0..9 corresponds to roughly 16..8192 ms
    // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
    for (word m = timeleft; m >= 32; m >>= 1)
        if (++wdp >= 9)
            break;
    watchdogCounter = 0;
    watchdogInterrupts(wdp);
    powerDown();
    watchdogInterrupts(-1); // off
    // when interrupted, our best guess is that half the time has passed
    word halfms = 8 << wdp;
    timeleft -= halfms;
    if (watchdogCounter == 0) {
        break;
    }
    timeleft -= halfms;
  }
  
  // adjust millis counter
  timer0_millis += milliseconds - timeleft;
  
}

/***
Power down everything except interrupts and BLE
*/
void powerDown() {
  uint8_t tempADCSRA = ADCSRA;
  ADCSRA = 0;
  power_adc_disable();
  //ACSR |= (1 << ACD); // disable Analog comparator, saves 4 uA
  
  power_usart0_disable();
  //power_spi_disable(); /do that a bit later, after we power RFM12b down
  power_twi_disable();
  power_timer0_disable(); // Do not disable if you need millis()!!!
  power_timer1_disable();
  power_timer3_disable();
  PRR1 |= (uint8_t)(1 << 4); //PRTIM4
  power_usart1_disable();
  
  // Switch to RC Clock 
  UDINT &= ~(1 << SUSPI); // UDINT.SUSPI = 0; Usb_ack_suspend
  USBCON |= ( 1 <<FRZCLK); // USBCON.FRZCLK = 1; Usb_freeze_clock
  PLLCSR &= ~(1 << PLLE); // PLLCSR.PLLE = 0; Disable_pll
  CLKSEL0 |= (1 << RCE); // CLKSEL0.RCE = 1; Enable_RC_clock()
  while ( (CLKSTA & (1 << RCON)) == 0){} // while (CLKSTA.RCON != 1); while (!RC_clock_ready())
  CLKSEL0 &= ~(1 << CLKS); // CLKSEL0.CLKS = 0; Select_RC_clock()
  CLKSEL0 &= ~(1 << EXTE); // CLKSEL0.EXTE = 0; Disable_external_clock
  
  // Datasheet says that to power off the USB interface we have to: 
  // Detach USB interface 
  // Disable USB interface 
  // Disable PLL 
  // Disable USB pad regulator 
 
  // Disable the USB interface 
  USBCON &= ~(1 << USBE); 
  
  // Disable the VBUS transition enable bit 
  USBCON &= ~(1 << VBUSTE); 
  
  // Disable the VUSB pad 
  USBCON &= ~(1 << OTGPADE); 
  
  // Freeze the USB clock 
  USBCON &= ~(1 << FRZCLK); 
  
  // Disable USB pad regulator 
  UHWCON &= ~(1 << UVREGE); 
  
  // Clear the IVBUS Transition Interrupt flag 
  USBINT &= ~(1 << VBUSTI); 
  
  // Physically detact USB (by disconnecting internal pull-ups on D+ and D-) 
  UDCON |= (1 << DETACH); 
  
  power_usb_disable(); // Keep it here, after the USB power down  
  
  //ble_sleep();
  //delay(50);
  power_spi_disable();
  
  TXLED1;
  RXLED1;	
  digitalWrite(PIN_LED_ARDUINO, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  
  pinMode(PIN_BATTERY, INPUT_PULLUP);
  /*
  pinMode(PIN_ENCODER_A, OUTPUT);
  pinMode(PIN_ENCODER_B, OUTPUT);
  digitalWrite(PIN_ENCODER_A, LOW);
  digitalWrite(PIN_ENCODER_B, LOW);
  */

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
#ifdef BODSE
  MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
  MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif 
  sei();
  
  sleep_cpu();

  // Arduino sleeps

  sleep_disable();

  // wake up
  setupPins();
  
  // turn on ADC
  power_adc_enable();
  ADCSRA = tempADCSRA;
  
}

