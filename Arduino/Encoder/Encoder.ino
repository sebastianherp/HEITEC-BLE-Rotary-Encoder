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
#define INTERVAL_STATUS_MESSAGES 0                 // in ms (0 to stop periodic sending)
#define INTERVAL_BATTERY_MEASUREMENTS 10000        // in ms
#define ENCODER_THRESHOLD 7000                     // in µs
#define PUSHBUTTON_THRESHOLD 250                   // in ms
//#define SERIAL_DEBUG 1

// PIN CONFIGURATION
#define PIN_BATTERY A5

#define PIN_LED_ARDUINO 13
#define PIN_LED_RED 10
#define PIN_LED_GREEN 9

#define PIN_PUSHBUTTON 3
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

uint16_t batteryVoltage = 0;
uint8_t batteryPercentage = 100;

uint8_t brightnessRed = 0;
uint8_t brightnessGreen = 0;

char line[5] = "";

// SETUP
void setup() {
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
  
  //if(ble_can_sleep() && millis() > 5000)
  //  powerDown();
}

void goToSleep()
{
    // BLE aus
    //ble_sleep();
    //delay(3000);
    //ble_do_events();
    
    TXLED1;
    RXLED1;	
    digitalWrite(PIN_LED_ARDUINO, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    
    pinMode(PIN_BATTERY, OUTPUT);
    digitalWrite(PIN_BATTERY, LOW);
    pinMode(PIN_ENCODER_A, OUTPUT);
    pinMode(PIN_ENCODER_B, OUTPUT);
    digitalWrite(PIN_ENCODER_A, LOW);
    digitalWrite(PIN_ENCODER_B, LOW);
    delay(3000);
    
    cli();    
    
    ADCSRA &= ~(_BV(ADEN)); // Turn off ADC
    ADCSRB &= ~(_BV(ACME)); // Turn off Analog Comparator
    // Brown-out Detector is disable in Arduino\hardware\blend\boards.txt.
    // The Extended Fuse Byte is setted to 0x0F to achieve this.


    // turn off Watch dog
    SREG = 0X00;
    __asm__ __volatile__ ("wdr" ::);//WDR();
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= _BV(WDCE)|_BV(WDE);
    WDTCSR = 0X00;
    SREG |= 0X80;

    MCUCR |= _BV(JTD); // Disable on-chip debug system.

    DIDR0 = 0xFF; // Analog to digital pin's digital input buffer is turned off

	
    // Int 6 (BLE) and Int 0-2 is used in this application
    EIMSK &= 0xF7;  // Disable INT3
    //EIMSK &= 0x00;

    PCICR &=0xFE;  //Disable PCINT7..0
    PCMSK0 = 0x00;

    PRR0 = 0xAD;  // Power Reduction Register
    PRR1 = 0x99; // Don't disable USB (0x99 to do that)

    /*
    

    wdt_disable();
  
    power_adc_disable();
    power_usart0_disable();
    power_spi_disable();
    power_twi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();
    power_timer3_disable();
    power_usart0_disable();
    power_usart1_disable();
    power_usb_disable();

    */
    
    // USB
    USBCON |= (1 << FRZCLK);   // Freeze the USB Clock
    PLLCSR &= ~(1 << PLLE);  // Disable the USB Clock (PPL)
    USBCON &= ~(1 << USBE);  // Disable the USB      


    bitClear(UDIEN,SOFE);



    set_sleep_mode(SLEEP_MODE_PWR_DOWN); //IDLE //PWR_SAVE
    sleep_enable();
    sei();
    sleep_cpu();

    sleep_disable();
    sei();
	
    //bitSet(UDIEN,SOFE);
    
    //ble_wakeup();
    
    pinMode(PIN_BATTERY, INPUT);
    digitalWrite(PIN_LED_ARDUINO, HIGH);
}

void powerDown() {
  ADCSRA =0;
  power_adc_disable();
  ACSR |= (1 << ACD); // disable Analog comparator, saves 4 uA
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
  
  ble_sleep();
  delay(500);
  power_spi_disable();
  
  TXLED1;
  RXLED1;	
  digitalWrite(PIN_LED_ARDUINO, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  
  pinMode(PIN_BATTERY, INPUT_PULLUP);
  pinMode(PIN_ENCODER_A, OUTPUT);
  pinMode(PIN_ENCODER_B, OUTPUT);
  digitalWrite(PIN_ENCODER_A, LOW);
  digitalWrite(PIN_ENCODER_B, LOW);
  

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
#ifdef BODSE
  MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
  MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif 
  sei();
  
  sleep_cpu();

  sleep_disable();

  digitalWrite(PIN_LED_ARDUINO, HIGH);  
}

void measureBattery() {
  measureBattery(false);
}

void measureBattery(boolean force) {
  static uint32_t lastMeasurement = 0;
  
  if(!force && lastMeasurement + INTERVAL_BATTERY_MEASUREMENTS > millis())
    return;
    
  // voltage divider is 10k each, so voltage can be measured in a range of 0 - 6600 mV
  batteryVoltage = map(analogRead(PIN_BATTERY), 0, 1023, 0, 6600);
  batteryPercentage = constrain(map(batteryVoltage, 3300, 4100, 0, 100), 0, 100);
}

void updateLEDs() {
  static uint8_t oldRed, oldGreen;
  if(oldRed != brightnessRed || oldGreen != brightnessGreen) {
    analogWrite(PIN_LED_RED, brightnessRed);
    analogWrite(PIN_LED_GREEN, brightnessGreen);
    oldRed = brightnessRed;
    oldGreen = brightnessGreen;
    delay(50);
  }
}

void sendStatusMessage() {
  sendStatusMessage(false);
}

void sendStatusMessage(boolean force) {
  static uint32_t lastStatus = 0;
  
  if( !force && 
      (INTERVAL_STATUS_MESSAGES <= 0 || lastStatus + INTERVAL_STATUS_MESSAGES > millis()) )
    return;
  
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
  }    
  
#if defined(SERIAL_DEBUG)  
  // Serial output
  Serial.print(F("Encoder position: ")); Serial.print(encoderPos);
  Serial.print(F(", LED (red): ")); Serial.print(brightnessRed);
  Serial.print(F(", LED (green): ")); Serial.print(brightnessGreen);
  Serial.print(F(", Battery: ")); Serial.print(batteryVoltage);
  Serial.print(F("mv (")); Serial.print(batteryPercentage);
  Serial.println(F("%)"));
#endif

  lastStatus = millis();
}

void handleEncoderEvent() {
  static uint16_t lastReportedPos = 0;
  static boolean ledOn = false;
  
  if (lastReportedPos != encoderPos) {
    digitalWrite(PIN_LED_ARDUINO, ledOn ? HIGH : LOW);
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

void handleButtonEvent() {
  if(buttonPressed) {
   
    // BLE output
    if( ble_connected() ) {
      ble_write(0x02); // 0x03 = button pressed
    }
    
#if defined(SERIAL_DEBUG)
    // Serial output
    Serial.println("Button press detected");
#endif

    buttonPressed = false;
  }  
}

void handleRequest() {
  
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
          brightnessRed = ble_read();
          brightnessGreen = ble_read();
          updateLEDs();
          break;
        default:
          break;
      }
    }
  }
  
}


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

void intPushbutton() {
  PRR0 = 0x00;  // Power Reduction Register: open timer
  PRR1 = 0x00;
    
  if( millis() - buttonTime < PUSHBUTTON_THRESHOLD )
    return;

  buttonPressed = true;
  buttonTime = millis();  
}
