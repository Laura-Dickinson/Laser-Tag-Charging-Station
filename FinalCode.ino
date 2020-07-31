//capacitive sensor libraries
#include <Adafruit_CAP1188.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_LiquidCrystal.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

Adafruit_LiquidCrystal lcd(0);

//initialize the xBee unit
SoftwareSerial xBee(2, 3);
const String tabChar = "\t";
const byte newLineChar = 0x0A;
int messages = 0;

bool msgComplete = false;
String msg = "";

//initialize the button and team states
int powerButtonState = 0, teamButtonOneState = 0;
int team = 1;

const int CHARGE_TIME = 5000;
const int hallSensorPin = A3;
unsigned long magnetDetectStart = 0;
bool charging = false;
int chargeBar = 0;

//initialize the capacitive sensor
#define CAP1188_RESET  6
#define CAP1188_CS  9
#define CAP1188_MOSI  10
#define CAP1188_MISO  7
#define CAP1188_CLK  8
Adafruit_CAP1188 cap = Adafruit_CAP1188(CAP1188_CLK, CAP1188_MISO, CAP1188_MOSI, CAP1188_CS, CAP1188_RESET);

//the rectangle custom character for the charge bar
byte customChar[] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const int powerButton = 4;
const int teamButtonOne = A2;

//initialize the RGB LED
const int redPin = 14;
const int greenPin = 5;
const int bluePin = 11;
#define COMMON_ANODE

void setup() {
  pinMode(powerButton, INPUT);
  pinMode(teamButtonOne, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  lcd.begin(16, 2);
  lcd.createChar(0, customChar);
  pinMode(A3, INPUT_PULLUP);
  Serial.begin(9600);
  xBee.begin(9600);

  //from the Adafruit tutorial on using capacitive sensors
  if (!cap.begin()) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
  uint8_t reg = cap.readRegister( 0x1f ) & 0x0f;
  //changes the sensitivity of the capacitive sensor to least sensitive
  cap.writeRegister(0x1f, reg | 0x40);
}

void loop() {
  //check powerbutton
  powerButtonState = digitalRead(powerButton);
  //if powerbutton on, continue
  if (powerButtonState == HIGH) {
    lcd.setBacklight(HIGH);
    //check teambuttons
    teamButtonOneState = digitalRead(teamButtonOne);
    //if button active change team
    if (teamButtonOneState == HIGH) {
      team++;
      if (team > 2) {
        team = 1;
      }
      delay(150);
    }
    //turn the RGB LED the appropriate team color
    teamColor();

    //check hall sensor, charge gun after 5 seconds
    handleCharging();
    //}
  }

  //Finn wrote a bunch of the code for the hall sensor detection
  void handleCharging() {
    bool magnetPresent = (digitalRead(hallSensorPin) == LOW);
    uint8_t touched = cap.touched();
    // identify charging start
    // identify charging for enough time
    Serial.print("magnetPresent:");
    Serial.print(magnetPresent);
    Serial.println();
    Serial.print("touched:");
    Serial.print(touched);
    Serial.println();
    if (magnetPresent && touched) {
      if (!charging) {
        // identify start of charge
        charging = true;
        magnetDetectStart = millis();
        xBee.println("p1 charging");
        Serial.println("sent p1 charging");
        lcd.setCursor(0, 1);
        lcd.print("Gun charging...");

        lcd.setCursor(0, 0);
        lcd.print("[          ]");
      } else {
        //increase bar progress
        if ((millis() - magnetDetectStart) >= 500 + 500 * chargeBar) {
          lcd.setCursor(1 + chargeBar, 0);
          lcd.write(0);
          chargeBar++;
        }

        // identify charging for long enough
        if ((millis() - magnetDetectStart) > CHARGE_TIME) {
          xBee.println("p1 charged");
          Serial.println("sent p1 charged");
          lcd.setCursor(0, 1);
          lcd.print("Gun charged     ");
          //turn the RBG LED green to signify "charged"
          setColor(0, 255, 0);
          Serial.println("should have turned green now");
          delay(2000);
          //reset the LCD and charging protocol
          lcd.setCursor(0, 1);
          lcd.print("                ");
          teamColor();
          chargeBar = 0;
        }
      }
    } else {
      charging = false;
      magnetDetectStart = 0;
      chargeBar = 0;
    }
  }

  //if the xBee receieves a message, the message is read
  void checkMessageReceived() {
    if (xBee.available()) {
      byte ch = xBee.read();
      if (ch == newLineChar) {
        msgComplete = true;
        messages++;
      }
      else {
        msg += char(ch);
        //prints the message in the serial monitor
        Serial.println(msg);
      }
    }
  }

  //sets the color of the team that has "claimed" the charging station
  void teamColor() {
    //turns the RGB LED the appropriate team color
    if (team == 1) {
      setColor(255, 115, 0);     //yellow
    } else if (team == 2) {
      setColor(255, 0, 0);       //red
    }
  }

  //from the built in RGB LED tutorial
  //uses the RGB values to modify the ratio of red, green, and blue emitted by the LED
  void setColor(int red, int green, int blue) {
#ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
#endif
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
  }
