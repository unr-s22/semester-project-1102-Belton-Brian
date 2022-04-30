//Brian Belton, Monte Howell, Ian Petersen

//----------------------------------------LIBRARIES-----------------------------------------//

#include <dht.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Stepper.h>

//-----------------GLOBAL VARIABLES / DECLARATIONS / DEFINITIONS--------------------//

// Input, Output, High, Low Definitions
#define SET_OUTPUT(ddr, pin) *ddr |= (0x01 << pin);
#define SET_INPUT(ddr, pin) *ddr &= ~(0x01 << pin);
#define SET_HIGH(port, pin) *port |= (0x01 << pin);
#define SET_LOW(port, pin) *port &= ~(0x01 << pin);

// PORT PIN Definition
#define START_STOP 2 // Physical Pin 2 on Arduino
#define YELLOW_LED 0 // Physical Pin 2 on Arduino
#define GREEN_LED 1 // Physical Pin 2 on Arduino
#define BLUE_LED 2 // Physical Pin 2 on Arduino
#define RED_LED 3 // Physical Pin 2 on Arduino

#define dht_apin A1 // Temperature and Humidity Sensor on Physical Pin A1 on Arduino 
#define WATER_SENSOR 0 // Water Level Sensor Port Specific Pin

#define ENABLE 0 // Port Specific Pin, Enable on DC motor control
#define DIN1 1 // Port Specific Pin, Input 1 on DC motor control
#define DIN2 2 // Port Specific Pin, Input 2 on DC motor control

#define IN1 49 // Physical Pin 0 on Arduino to IN1 of Stepper Control
#define IN2 48 // Physical Pin 1 on Arduino to IN2 of Stepper Control
#define IN3 47 // Physical Pin 2 on Arduino to IN3 of Stepper Control
#define IN4 46 // Physical Pin 3 on Arduino to IN4 of Stepper Control
#define clockwiseButton 4
#define counterClkButton 5 

// Variables For State Values
bool DISABLED_ST, IDLE_ST, RUNNING_ST, ERROR_ST;

// Port A Register For LEDS
volatile unsigned char* port_A = (unsigned char*) 0x22;
volatile unsigned char* ddr_A = (unsigned char*) 0x21;
volatile unsigned char* pin_A = (unsigned char*) 0x20;

// Port B Register For DC Motor
volatile unsigned char* port_B = (unsigned char*) 0x25;
volatile unsigned char* ddr_B = (unsigned char*) 0x24;
volatile unsigned char* pin_B = (unsigned char*) 0x23;

// Port D Register For Start/Stop button
volatile unsigned char* port_D = (unsigned char*) 0x2B;
volatile unsigned char* ddr_D = (unsigned char*) 0x2A;
volatile unsigned char* pin_D = (unsigned char*) 0x29;

// Port F Register For Water Sensor input
volatile unsigned char* port_F = (unsigned char*) 0x31;
volatile unsigned char* ddr_F = (unsigned char*) 0x30;
volatile unsigned char* pin_F = (unsigned char*) 0x2F;

// Port L Register For Stepper Motor Control Buttons
volatile unsigned char* port_L = (unsigned char*) 0x10B;
volatile unsigned char* ddr_L = (unsigned char*) 0x10A;
volatile unsigned char* pin_L = (unsigned char*) 0x109;

// ADC Register for Water sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// ISR register
volatile unsigned char* my_ECIRA = (unsigned char*) 0x69;
volatile unsigned char* my_EIMSK = (unsigned char*) 0x3D;
int count = 0;

//function stuff
dht DHT;
LiquidCrystal LCD(7,8,9,10,11,12); // Numbers represent Physical pins on Arduino
RTC_DS1307 rtc;

int stepsPerRevolution = 2048; // Stepper Motor steps for 1 full rotation
Stepper myStepper = Stepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

int waterLvl;       // Water Level Reading
int current_state;  // Current State variable
int state_change = 0; // ISR Change State variable for RTC display purposes
float threshTemp = 21; // Control Temperature
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//---------------------------------------ISR------------------------------------------//

ISR(INT2_vect){
  count++;
  state_change = 1;

  if(count%2 == 1) {
    DISABLED_ST = false;
    RUNNING_ST = false;
    ERROR_ST = false;
    IDLE_ST = true;
  }
  else {
    IDLE_ST = false;
    RUNNING_ST = false;
    ERROR_ST = false;
    DISABLED_ST = true;
  }
}

//-----------------------------------------SETUP----------------------------------------//

void setup() {
  *my_ECIRA |= 0x20; //set falling edge interrupt 
  *my_EIMSK |= 0x04; //enable interrupt on PD2

  SET_INPUT(ddr_D, START_STOP); // set PD2 to input with pullup enabled
  SET_HIGH(port_D, START_STOP);

  SET_INPUT(ddr_F, WATER_SENSOR); // set PF0 to input with pullup enabled
  SET_HIGH(port_F, WATER_SENSOR);

  SET_INPUT(ddr_L, clockwiseButton); // set PL4 to input with pullup enabled
  SET_HIGH(port_L, clockwiseButton);

  SET_INPUT(ddr_L, counterClkButton); // set PL5 to input with pullup enabled
  SET_HIGH(port_L, counterClkButton);
  
  SET_OUTPUT(ddr_A, YELLOW_LED); // set PA0 to output (YELLOw LED)
  SET_OUTPUT(ddr_A, GREEN_LED); // set PA1 to output (GREEN LED)
  SET_OUTPUT(ddr_A, BLUE_LED); // set PA2 to output (BLUE LED)
  SET_OUTPUT(ddr_A, RED_LED); // set PA3 to output (RED LED)

  SET_OUTPUT(ddr_B, ENABLE); // set PB0 as output for DC motor control ENABLE
  SET_OUTPUT(ddr_B, DIN1); // set PB1 as output for DC motor control IN 1 (POSITIVE)
  SET_OUTPUT(ddr_B, DIN2); // set PB2 as output for DC motor control IN 2 (NEGATIVE)

  adc_init(); // Function to setup ADC

  DISABLED_ST = true;
  IDLE_ST = false;
  ERROR_ST = false;
  RUNNING_ST = false;

  Serial.begin(9600);
  LCD.begin(16, 2);
  rtc.begin();
  myStepper.setSpeed(10);
  
  delay(1000);
}

//-----------------------------------------MAIN LOOP---------------------------------------//

void loop() {
  current_state = getState(); // Select State
  switch(current_state) {
    case 1:
        Disabled();
        break;
    case 2:
        Idle();
        break;
    case 3:
        Running();
        break;
    case 4:
        Error();
        break;
    default:
        DISABLED_ST = true;
        break;
  }

  if(!(*pin_L & (0x001 << clockwiseButton))){
    myStepper.step(stepsPerRevolution/8);
    Serial.print("Stepper Motor Clockwise by 45 degrees:     ");
    callRTC();
  }

  if(!(*pin_L & (0x001 << counterClkButton))){
    myStepper.step(-stepsPerRevolution/8);
    Serial.print("Stepper Motor CounterClockwise by 45 degrees:     ");
    callRTC();
  }
  
  delay(200);
}

//----------------------------------------STATE SELECTION-----------------------------------------//

int getState() {
  int value;
  if(DISABLED_ST){
    value = 1;
  }
  else if(IDLE_ST){
    value = 2;
  }
  else if(RUNNING_ST){
    value = 3;
  }
  else if(ERROR_ST){
    value = 4;
  }
  return value;
}

//---------------------------------------DISABLED------------------------------------------//

void Disabled() {
  SET_HIGH(port_A, YELLOW_LED); // turn PA0 on
  SET_LOW(port_A, GREEN_LED); // turn PA1 off
  SET_LOW(port_A, BLUE_LED); // turn PA2 off
  SET_LOW(port_A, RED_LED); // turn PA3 off

  SET_LOW(port_B, ENABLE); // turn DC Motor OFF

  if(state_change){
    Serial.print("System OFF.....DISABLED:     ");
    callRTC();
    state_change = 0;
  }

  LCD.clear(); // Clear LCD Display
}

//---------------------------------------IDLE------------------------------------------//

void Idle() {
  SET_HIGH(port_A, GREEN_LED); // turn PA1 on
  SET_LOW(port_A, YELLOW_LED); //turn PA0 off
  SET_LOW(port_A, BLUE_LED); // turn PA2 off
  SET_LOW(port_A, RED_LED); // turn PA3 off

  if(state_change){
    Serial.print("System ON......IDLE:     ");
    callRTC();
    state_change = 0;
  }

  readTempHumidLCD(); // Display Temp & Humid on LCD  
  waterLvl = readWaterLevel(WATER_SENSOR); // Determine the value of the sensor

  if(waterLvl < 100){
    Serial.print("System OFF.....ERROR:     ");
    callRTC();           // Exact time stamp is displayed on computer
    IDLE_ST = false;
    ERROR_ST = true;
  }

  else if(DHT.temperature > threshTemp){
    Serial.print("System ON......RUNNING:     ");
    callRTC();
    IDLE_ST = false;
    RUNNING_ST = true;
  }
}

//---------------------------------------RUNNING-----------------------------------------//

void Running() {
  SET_HIGH(port_A, BLUE_LED); // turn PA2 on
  SET_LOW(port_A, YELLOW_LED); // turn PA0 off
  SET_LOW(port_A, GREEN_LED); // turn PA1 off
  SET_LOW(port_A, RED_LED); // turn PA3 off

  readTempHumidLCD();

  SET_HIGH(port_B, ENABLE); // turn DC motor ON
  SET_HIGH(port_B, DIN1); // positive terminal
  SET_LOW(port_B, DIN2); // negative terminal
  
  waterLvl = readWaterLevel(WATER_SENSOR);

  if(waterLvl < 100){
    SET_LOW(port_B, ENABLE); // turn DC Motor OFF
    Serial.print("System OFF.....ERROR:     ");
    callRTC();
    RUNNING_ST = false;
    ERROR_ST = true;
  }

  else if(DHT.temperature < threshTemp){
    SET_LOW(port_B, ENABLE); // turn DC Motor OFF
    Serial.print("System ON......IDLE:     ");
    callRTC();
    RUNNING_ST = false;
    IDLE_ST = true;
  }
}

//---------------------------------------ERROR-----------------------------------------//

void Error() {
  SET_HIGH(port_A, RED_LED); // turn PA3 on
  SET_LOW(port_A, YELLOW_LED); // turn PA0 off
  SET_LOW(port_A, GREEN_LED); // turn PA1 off
  SET_LOW(port_A, BLUE_LED); // turn PA2 off

  waterLvl = readWaterLevel(WATER_SENSOR);

  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("  Water Level");
  LCD.setCursor(0,1);
  LCD.print("      Low!");

  if(waterLvl > 200){
    Serial.print("System ON......IDLE:     ");
    callRTC();
    ERROR_ST = false;
    IDLE_ST = true;
  }
}

//--------------------------------DISPLAY TEMP & HUMID ON LCD---------------------------------//

void readTempHumidLCD(){
  
  DHT.read11(dht_apin);
  LCD.clear();
  LCD.setCursor(0,0);
  LCD.print("Temp: ");
  LCD.print(DHT.temperature);
  LCD.print("C");
  LCD.setCursor(0,1);
  LCD.print("Humidity: ");
  LCD.print(DHT.humidity);
  LCD.print("%");
}

//---------------------------------------READ WATER SENSOR----------------------------------------//

unsigned int readWaterLevel(unsigned char adc_channel_num){

  *my_ADMUX  &= 0xE0;       // reset the channel and gain bits
  *my_ADCSRB &= 0xF7;       // clear the channel selection bits
  *my_ADMUX  += adc_channel_num;  // set the channel selection bits
  *my_ADCSRA |= 0x40;       // set bit 6 of ADCSRA to 1 to start a conversion
  
  while((*my_ADCSRA & 0x40)==1);   // wait for the conversion to complete
  
  return *my_ADC_DATA;       // return the result in the ADC data register
}

//----------------------------------DISPLAY RTC ON COMPUTER-------------------------------------//

void callRTC(){
 DateTime now = rtc.now();
  
 Serial.print(now.year(), DEC);
 Serial.print('/');
 Serial.print(now.month(), DEC);
 Serial.print('/');
 Serial.print(now.day(), DEC);
 Serial.print(" (");
 Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
 Serial.print(") ");
 Serial.print(now.hour(), DEC);
 Serial.print(':');
 Serial.print(now.minute(), DEC);
 Serial.print(':');
 Serial.print(now.second(), DEC);
 Serial.println();
}

//---------------------------------------SETUP ADC----------------------------------------//

void adc_init()
{
  // set up the A register
  *my_ADCSRA |= 0x80; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0xDF; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0xF7; // clear bit 3 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0xF8; // clear bit 2-0 to 0 to set prescaler selection to slow reading
  
  // set up the B register
  *my_ADCSRB &= 0xF7; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0xF8; // clear bit 2-0 to 0 to set free running mode
  
  // set up the MUX Register
  *my_ADMUX  &= 0x7F; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0x40; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0xDF; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0xE0; // clear bit 4-0 to 0 to reset the channel and gain bits
}
