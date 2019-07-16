//Joe Sanford
//17JUL02018
//Intellibot

//Sonar FOV-Move Target
/*This sketch helps to automate part of a sonar field of view
 * data set gathering task
 * This sketch moves a target in the y-axis (perpindicular to
 * the sonar field of view) in 1 cm increments
 * This sketch only moves the target in 1-axis
 * 
 * We use an Arduino MEGA 2560 to command a
 * DQ542MA 2H Microstep driver that drives a NEMA STEPPER MOTOR
 * in 1/8 Microstepping
 * 
 * July 2018 V 1.0: Initial Version
 * 
 */

 #include <SoftwareSerial.h>   //Software Serial Port
//#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

int stepPin = 26;
int dirPin = 28;  //direction is set by button press
int enblPin = 32; //stepper motor enabled and disabled in singleStep()

const char firstlineText[17] = "Sonar_FOV_ ";
const char secondlineText[17] = "Move_TargetV1.0 ";

void setup() 
{

  Serial.begin(9600);

  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(firstlineText);
  lcd.setCursor(0,1);
  lcd.print(secondlineText);
  int time = millis();
  lcd.print("Hello, world!");
  time = millis() - time;
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");
  lcd.setBacklight(WHITE);
  //lcd.println("Distances in mm ");
  //delay(2000);
  
  pinMode (stepPin, OUTPUT);
  pinMode (dirPin, OUTPUT);
  pinMode (enblPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  delay(1000);
  //digitalWrite(dirPin, LOW); //these are set in singleStep()
  digitalWrite(enblPin, HIGH); //these are set in singleStep()
} //void setup()

void singleStep()
{
  digitalWrite(enblPin, HIGH);
  for(int x=0; x<1667; x++)
  { //Double Check number of revolutions
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
  } //for(int x=0...
} //void singleStep

//uint8_t i=0;
void loop() 
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(millis()/1000); // print the number of seconds since reset:
  uint8_t buttons = lcd.readButtons(); //setting up button read
  if (buttons) 
  {
    lcd.clear();
    lcd.setCursor(0,0);
    
    if (buttons & BUTTON_LEFT) 
    {
      lcd.print("LEFT ");
      lcd.setBacklight(GREEN);
      digitalWrite(dirPin, LOW);
      singleStep();
      lcd.setBacklight(WHITE); //reset LCD back to white
    } //if (buttons & ... 
      
    if (buttons & BUTTON_RIGHT) 
    {
      lcd.print("RIGHT ");
      lcd.setBacklight(GREEN);
      digitalWrite(dirPin, HIGH);
      singleStep();
      lcd.setBacklight(WHITE); //reset LCD back to white
    } //if (buttons & ...   
  } //if (buttons) {

} // void loop()
