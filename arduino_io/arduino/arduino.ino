
#include <string.h>
#include <Servo.h>    // 声明调用Servo.h库


//ros::NodeHandle  nh;/

int keyboardPin = A0;    // select the input pin for the potentiometer
int ledRPin = 8;      // select the pin for the LED
int ledGPin = 9;      // select the pin for the LED
int ledYPin = 10;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

int agv_state = 0;
int goal = 0;
String inString = "0";
String stringDwonToUp = "STATEDTU";
String stringUpToDwon = "STATEUTD";
String stringSucked = "GOAL";
String stringOver = "OVER";
String outString = stringDwonToUp + inString + stringDwonToUp;

//true 1
//false 0
int sucked = 0;
bool serialflag = false;

int adc_key_val[5] = {50, 200, 400, 600, 800};
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

int get_key(unsigned int input) {
  int k;
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k]) {
      if(k==4){
        k=-1;
      }
      return k+1;
    }
  }
  if (k >= NUM_KEYS)k = -1;  // No valid key pressed
  return k;
}

void set_led(int state) {
  switch (state) {
    case 0:
      set_rgy(true, false, false);
      break;
    case 1:
      set_rgy(false, true, false);
      break;
    case 2:
      set_rgy(false, false, true);
      break;
    default:
      set_rgy(true, false, false);
      break;
  }
}

void set_rgy(bool r, bool g, bool y) {
  if (r) {
    digitalWrite(ledRPin, HIGH);
  } else {
    digitalWrite(ledRPin, LOW);
  }
  if (g) {
    digitalWrite(ledGPin, HIGH);
  } else {
    digitalWrite(ledGPin, LOW);
  }
  if (y) {
    digitalWrite(ledYPin, HIGH);
  } else {
    digitalWrite(ledYPin, LOW);
  }

}

void setup() {
  pinMode(ledRPin, OUTPUT);
  pinMode(ledGPin, OUTPUT);
  pinMode(ledYPin, OUTPUT);
  analogReference(INTERNAL); //调用板载1.1V基准源
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(keyboardPin);
  key = get_key(sensorValue);  // convert into key press
  if (key != oldkey)   // if keypress is detected
  {
    delay(50);  // wait for debounce time
    adc_key_in = analogRead(keyboardPin);    // read the value from the sensor
    key = get_key(adc_key_in);    // convert into key press
    if (key != oldkey) {
      oldkey = key;
      if (key >= 0) {
        goal = key;
      }
    }
  }

  if (serialflag) {
    char inChar = '\n';
    while (Serial.available() > 0) {
      inChar = Serial.read();

      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char) inChar;
    }
    sscanf(inString.c_str(), "%*s\nSTATEUTD%dSTATEUTD", &agv_state);
    set_led(agv_state);
  }
  serialflag = !serialflag;

  outString = "";
  outString += stringDwonToUp;
  char temp_str[20];
  //outString += itoa((int)myservo.read() * 10 * 1.0, temp_str, 10);  
  outString += itoa((int) agv_state, temp_str, 10);
  //outString += stringDwonToUp;
  outString += stringSucked;
  outString += itoa((int) goal, temp_str, 10);
  outString += stringOver;
  inString = "";


  Serial.println(outString);
  delay(100);
}
