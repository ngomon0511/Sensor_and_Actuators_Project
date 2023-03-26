#define IN1	7
#define IN2	6
#define IN3	3
#define IN4	4
#define button 8
#define button1 9
#define button2 10
#define MAX_SPEED 200
#define MIN_SPEED 0
#define Led 13
boolean but;
boolean but1;
boolean but2;
int state_button = 0;
int FlipState = 0;
int period = 10000;
unsigned long time;
unsigned long time_now = 0;
boolean TarpaulinState = 1;
int RaSensor = 11; 
int PhReSensor = 12;
int state_automatic = 0;

/*
Prepare for reading temperature and humidity by LCD
*/
#include <DHT.h>
#include <Wire.h>
const int DHT_pin = 5;
const int DHT_type = DHT11;
DHT dht(DHT_pin, DHT_type);
byte delayMS = 1000;
int t = 0; // Temperature
int h = 0; // Humidity
unsigned long CountTimes = 0;
#include <LiquidCrystal_I2C.h> // LCD library
LiquidCrystal_I2C lcd(0x27, 16, 2);
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};

/*
Fix LCD library error
*/
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(button, INPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(RaSensor, INPUT);
  pinMode(PhReSensor, INPUT);
  pinMode(Led, OUTPUT);
  
  lcd.init();       
  lcd.backlight();
  lcd.createChar(3, heart);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("AGRICULTURAL    ");
  lcd.setCursor(13, 0);
  for (int i = 0; i<3;i++)
    lcd.printByte(3);
  lcd.setCursor(0, 1);
  lcd.print("   DRYING SYSTEM");
  lcd.setCursor(0, 1);
  lcd.printByte(2);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  for (int i = 0; i<16;i++)
    lcd.printByte(3);
  lcd.setCursor(0,1);
  lcd.print("t=    C H=   %  ");
  lcd.setCursor(5,1);
  lcd.printByte(223);
  dht.begin();
  Serial.begin(9600);
  Serial.println("Read");
}

void motor_1_Stop() {
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
}
 
void motor_2_Stop() {
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}
 
void motor_1_Forward(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);
	digitalWrite(IN1, HIGH);
	analogWrite(IN2, 255 - speed);
}
 
void motor_1_Backward(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);
	digitalWrite(IN1, LOW);
	analogWrite(IN2, speed);
}
 
void motor_2_Forward(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);
	analogWrite(IN3, speed);
	digitalWrite(IN4, LOW);
}
 
void motor_2_Backward(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);
	analogWrite(IN3, 255 - speed);
	digitalWrite(IN4, HIGH);
}

/*
  Control two motors with three buttons
*/
boolean ControlMotor(){
  if (but == HIGH)
  {
    delay(500);
    {
      if (digitalRead(button) == HIGH)
      {
        state_button+= 1;
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////
  if (but1 == HIGH)
  {
    delay(500);
    if (digitalRead(button1) == HIGH)
    {
      delay(500);
      if (digitalRead(button2) == 0)
      {
        Serial.println("HandControlTarpaulin");
        if (state_button%2 == 0)
        {
          TarpaulinControll("Open");
          return true;
        }
        else
        {
          TarpaulinControll("Close");
          return false;
        }
      }
      else 
      {
        Serial.println("ChangeMode");
        state_automatic+= 1;
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////
  if (but2 == HIGH)
  {
    delay(500);
    if (digitalRead(button2) == HIGH)
    {
      delay(500);
      if (digitalRead(button1) == 0)
      {
        Serial.println("HandControlFlip");
        if (state_button%2 == 0)
        {
          motor_2_Forward(MAX_SPEED);
        }
        else
        {
          motor_2_Backward(MAX_SPEED);
        }
        
      }
      else 
      {
        Serial.println("ChangeMode");
        state_automatic+= 1;
      }
    }
    return false;
  }
  else motor_2_Stop();
  return true;
}

/*
  Automatic control two motors with signal from sensor
*/
void FlipControl(){
  Serial.println(TarpaulinState);
  if (TarpaulinState)
  {
    if ( (unsigned long) (millis() - time) > 1000)
      {
        Serial.println(FlipState);
        if (FlipState%2 == 0)
        {
          motor_2_Forward(MAX_SPEED);
          delay(1000);
          motor_2_Stop();
        } 
        else if (FlipState%2 == 1)
        {
          motor_2_Backward(MAX_SPEED);
          delay(1000);
          motor_2_Stop();
        }
        FlipState +=1;
        time = millis();
      }
  }
  else motor_2_Stop();
}
////////////////////////////////////////////////////////////////////////
void TarpaulinControll(String state){
  if (state == "Open")
  {
    TarpaulinState = 1;
    motor_1_Backward(MAX_SPEED);
    delay(1000);
    motor_1_Stop();
    time_now = 0;
  }
  else if (state == "Close")
  {
    TarpaulinState = 0;
    motor_1_Forward(MAX_SPEED);
    delay(1000);
    motor_1_Stop();
  }
}
  
void loop() {
  /*
    Read and display Temperature & Humidity on LCD
  */
  if ((unsigned long) (millis() - CountTimes) > delayMS) // Delay between measurements
    {
    float t = dht.readTemperature(); // Read Temperature
    lcd.setCursor(3,1);
    lcd.print(int(t));
    float h = dht.readHumidity();    // Read Humidity
    lcd.setCursor(11,1);
    lcd.print(int(h));
    CountTimes = millis();
    }
  
  // Read state buttons
  but = digitalRead(button);
  but1 = digitalRead(button1);
  but2 = digitalRead(button2);
  
  // Read value from Sensor
  boolean PhReVal = digitalRead(PhReSensor); 
  boolean RaVal = digitalRead(RaSensor); 
  
  // Check the hand-control method
  if(ControlMotor()) 
  {
    if(millis() > time_now + period)
    {
        time_now = millis();
        FlipControl();
    }
  }
  
  /*
    Open or close tarpaulin
  */
  if (state_automatic%2 == 1) digitalWrite(Led, LOW);
  else digitalWrite(Led, HIGH);
//////////////////////////////////////////////////////////////////////// 
  if ((PhReVal == HIGH) || (RaVal == HIGH) || (h >= 70)) 
  {
    delay(20);
    if (((PhReVal == HIGH) || (RaVal == HIGH) || (h >= 70)) && (TarpaulinState == 1) && (state_automatic%2 == 0))
    {
      TarpaulinControll("Close");
    }
  }
////////////////////////////////////////////////////////////////////////
  if ((PhReVal == LOW) && (RaVal == LOW) && (h < 70)) 
  {
    delay(20);
    if (((PhReVal == LOW) && (RaVal == LOW) && (h < 70)) && (TarpaulinState == 0) && (state_automatic%2 == 0))
    {
      TarpaulinControll("Open");
    }
  }
}
