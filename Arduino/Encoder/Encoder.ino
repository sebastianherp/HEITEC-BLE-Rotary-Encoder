/*******************************************************************
Bluetooth LE Pushbutton mit Drehencoder und integrierter roter und grüner LED.


*******************************************************************/

#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>

// APPLICATION PARAMETERS
#define INTERVAL_STATUS_MESSAGES 0                 // in ms (0 to stop periodic sending)
#define INTERVAL_BATTERY_MEASUREMENTS 10000        // in ms
#define ENCODER_THRESHOLD 7000                     // in µs
#define PUSHBUTTON_THRESHOLD 250                   // in ms

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
  
  // Init. and start BLE library.
  ble_begin();
  
  
  
  // Interrupts
  attachInterrupt(INT_ENCODER_A, intEncoderA, CHANGE);
  attachInterrupt(INT_ENCODER_B, intEncoderB, CHANGE);
  attachInterrupt(INT_PUSHBUTTON, intPushbutton, FALLING);
  
  // Initial measurement
  measureBattery(true);
  
  // Debug output
  Serial.begin(115200); // baudrate doesn't matter => virtual usb com port
}

void loop() {
  
  updateLEDs();

  handleEncoderEvent();
  
  handleButtonEvent();

  measureBattery();

  sendStatusMessage();
  
  ble_do_events();
  
  handleRequest();
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
      sprintf(line, "P: %2d", (500000 / 1000) % 100);
      for(int i=0;i<5;i++)
        ble_write(line[i]);
    }    
  
  // Serial output
  Serial.print(F("Battery: ")); Serial.print(batteryVoltage);
  Serial.print(F("mv (")); Serial.print(batteryPercentage);
  Serial.print(F("%), LED (red): ")); Serial.print(brightnessRed);
  Serial.print(F(", LED (green): ")); Serial.print(brightnessGreen);
  Serial.print(F(", Encoder position: ")); Serial.println(encoderPos);
  
  lastStatus = millis();
}

void handleEncoderEvent() {
  static uint16_t lastReportedPos = 0;
  
  if (lastReportedPos != encoderPos) {
    
    // BLE output
    if( ble_connected() ) {
      sprintf(line, "%5d", encoderPos);
      for(int i=0;i<5;i++)
        ble_write(line[i]);
      
    }
    
    Serial.print(F("Encoder position: ")); Serial.print(encoderPos);
    Serial.print(", Speed: "); Serial.print(degreesPerSecond);
    Serial.println(" deg/s");
    lastReportedPos = encoderPos;
  }  
  
}

void handleButtonEvent() {
  if(buttonPressed) {
   
    // BLE output

    // Serial output
    Serial.println("Button press detected");
    
    buttonPressed = false;
  }  
}

void handleRequest() {
  
  if ( ble_available() )
  {
    Serial.print("Incoming: ");
    while ( ble_available() )
    {
      Serial.write(ble_read());
    }
    
    Serial.println();
  }
  
}


void intEncoderA(){
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
  if( millis() - buttonTime < PUSHBUTTON_THRESHOLD )
    return;

  buttonPressed = true;
  buttonTime = millis();  
}
