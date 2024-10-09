#include <Adafruit_SSD1306.h> 
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <CD74HC4067.h>
#include <Bluepad32.h>
#include <multiplexedQTR.h>
#include <locomotion.h>
#include <bitmaps_triatlon.h>
#include <progress_bars.h>

/* Global section
--------------------------------------------------------------------------*/

bool ctlConnected = false;

Motor motorRight(26, 27);
Motor motorLeft(16, 17);

/* End of global section
--------------------------------------------------------------------------*/
/* Menu section */

// Define display size in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_MARGIN_X 0
#define SCREEN_MARGIN_Y 0
#define OLED_RESET -1

#define ICON_SIZE 16
#define ICON_MARGIN_X 4
#define FIRST_ICON_MARGIN_Y 4
#define SECOND_ICON_MARGIN_Y 24
#define THIRD_ICON_MARGIN_Y 46

#define SELECTED_BACKROUND_HEIGHT 20
#define SELECTED_BACKROUND_MARGIN_X 0
#define SELECTED_BACKROUND_MARGIN_Y 22

#define SPRINTER_SCREEN_MARGIN_X 43
#define SPRINTER_SCREEN_MARGIN_Y 0
#define SPRINTER_SCREEN_WIDTH 43
#define SPRINTER_SCREEN_HEIGHT 64

#define CLEANER_SCREEN_MARGIN_X 2
#define CLEANER_SCREEN_MARGIN_Y 0
#define CLEANER_SCREEN_WIDTH 123
#define CLEANER_SCREEN_HEIGHT 64

#define FIRST_SAFETY_TIMEOUT 3000
#define SECOND_SAFETY_TIMEOUT 2000

#define NUM_MODALITIES 3
#define MAX_ITEM_LENGTH 20

// Define display buttons 
#define PIN_SELECT 18 //18
#define PIN_DOWN 5 //5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

ProgressBar displayPB(display);

// Prints a menu
void DisplayMenu(){
  if (current_screen == selection) {
    display.clearDisplay();

    // Prints previous item name
    u8g2_for_adafruit_gfx.setFontMode(1);
    u8g2_for_adafruit_gfx.setFontDirection(0);
    u8g2_for_adafruit_gfx.setForegroundColor(WHITE);
    u8g2_for_adafruit_gfx.setFont(u8g2_font_7x14_mf);
    u8g2_for_adafruit_gfx.setCursor(25, 15);
    u8g2_for_adafruit_gfx.print(F(menu_items [previous]));

    // Prints selected item name
    u8g2_for_adafruit_gfx.setFontMode(1);
    u8g2_for_adafruit_gfx.setFontDirection(0);
    u8g2_for_adafruit_gfx.setForegroundColor(WHITE);
    u8g2_for_adafruit_gfx.setFont(u8g2_font_7x14B_mf);
    u8g2_for_adafruit_gfx.setCursor(30, 37);
    u8g2_for_adafruit_gfx.print(F(menu_items [selected]));
  
    // Prints next item name
    u8g2_for_adafruit_gfx.setFontMode(1);
    u8g2_for_adafruit_gfx.setFontDirection(0);
    u8g2_for_adafruit_gfx.setForegroundColor(WHITE);
    u8g2_for_adafruit_gfx.setFont(u8g2_font_7x14_mf);
    u8g2_for_adafruit_gfx.setCursor(25, 59);
    u8g2_for_adafruit_gfx.print(F(menu_items [next]));
    
    // Prints selection frame
    display.drawXBitmap (SELECTED_BACKROUND_MARGIN_X, SELECTED_BACKROUND_MARGIN_Y, epd_bitmap_selected_background, SCREEN_WIDTH, SELECTED_BACKROUND_HEIGHT, WHITE);
    
    // Prints previous item icon
    display.drawXBitmap (ICON_MARGIN_X, FIRST_ICON_MARGIN_Y, bitmap_icons[previous], ICON_SIZE, ICON_SIZE, WHITE);

    // Prints selected item icon
    display.drawXBitmap (ICON_MARGIN_X, SECOND_ICON_MARGIN_Y, bitmap_icons[selected], ICON_SIZE, ICON_SIZE, WHITE);

    // Prints next item icon
    display.drawXBitmap (ICON_MARGIN_X, THIRD_ICON_MARGIN_Y, bitmap_icons[next], ICON_SIZE, ICON_SIZE, WHITE);

    display.display();
  } 
  else if (current_screen == modality && selected == sprinter) {
    displayPB.load(SPRINTER_SCREEN_MARGIN_X, SPRINTER_SCREEN_MARGIN_Y, SPRINTER_SCREEN_WIDTH, SPRINTER_SCREEN_HEIGHT, FIRST_SAFETY_TIMEOUT);
    displayPB.unload(SPRINTER_SCREEN_MARGIN_X, SPRINTER_SCREEN_MARGIN_Y, SPRINTER_SCREEN_WIDTH, SPRINTER_SCREEN_HEIGHT, SECOND_SAFETY_TIMEOUT);

    current_screen = flags;
  }
  else if (current_screen == modality && selected == areaCleaner) {
    displayPB.load(CLEANER_SCREEN_MARGIN_X, CLEANER_SCREEN_MARGIN_Y, CLEANER_SCREEN_WIDTH, CLEANER_SCREEN_HEIGHT, FIRST_SAFETY_TIMEOUT);   
    displayPB.unload(CLEANER_SCREEN_MARGIN_X, CLEANER_SCREEN_MARGIN_Y, CLEANER_SCREEN_WIDTH, CLEANER_SCREEN_HEIGHT, SECOND_SAFETY_TIMEOUT);

    current_screen = flags;
  }
  else if (current_screen == flags){
    display.clearDisplay();
    display.drawXBitmap(SCREEN_MARGIN_X, SCREEN_MARGIN_Y, epd_bitmap_flag, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    display.display();
  }
  else if (current_screen == modality && selected == sumo){
    if (!ctlConnected) {
      display.clearDisplay();
      display.drawXBitmap(SCREEN_MARGIN_X, SCREEN_MARGIN_Y, bitmap_screens[selected], SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
      display.display();

      delay(250);

      display.clearDisplay();
      display.display();

      delay(250);
    } else {
      display.clearDisplay();
      display.drawXBitmap(SCREEN_MARGIN_X, SCREEN_MARGIN_Y, bitmap_screens[selected], SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
      display.display();
    }
  }
  else if (current_screen == modality && selected == sprinter){
    display.clearDisplay();
    display.drawXBitmap(SCREEN_MARGIN_X, SCREEN_MARGIN_Y, bitmap_screens[selected], SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    display.display();
  }

  UpdateScreenStatus();

  if (current_screen == selection){  
    motorRight.StayStill();
    motorLeft.StayStill();

    BP32.enableNewBluetoothConnections(false);
  }
}

/* End of menu section
--------------------------------------------------------------------------*/
/* Sprinter section */

#define QTR_QUANTITY 8

multiplexedQTR qtr;

const uint8_t SensorCount = QTR_QUANTITY;
uint16_t sensorValues[SensorCount];

int position;

// Gets line position
int getPosition() {
  position = qtr.readLineWhite(sensorValues);
  return position;
}

// Calibrates sprinters QTR sensors
void StartSprinterCalibration() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, SensorCount); 

  delay(500);

  // Prints calibration big icon
  display.clearDisplay();
  display.drawXBitmap(SCREEN_MARGIN_X, SCREEN_MARGIN_Y, bitmap_calibration, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.display();

  for (uint16_t i = 0; i < 350; i++){
    qtr.calibrate();   
  }

  current_screen = modality;
}

int setPoint = 1500; // Sets line position

int proportional;
int derivative;
int integral;
int lastError;

int maxSpeed = 170;
int minSpeed = 100;
int speed = 135;

// PID const
float kp = 0.033;
float ki = 0;
float kd = 0.047;
float pid;
float pidRight;
float pidLeft;

//PID control system code
void StartSprinterModality(){
  position = getPosition();

  proportional = position - setPoint; // Newest error
  integral += proportional; // Integral of the error
  derivative = proportional - lastError; // Derivative of the error

  // PID aftermath
  pid = (proportional * kp) + (integral * ki) + (derivative * kd);
    
  lastError = proportional; // Saves last error

  pidRight = speed + pid;
  pidLeft = speed - pid;

  // Defines speed limits for right motor
  if (pidRight > maxSpeed){pidRight = maxSpeed;} 
  else if (pidRight < minSpeed){pidRight = minSpeed;}

  // Defines speed limits for left motor
  if (pidLeft > maxSpeed){pidLeft = maxSpeed;} 
  else if (pidLeft < minSpeed){pidLeft = minSpeed;}

  // Defines turning speed
  if (pidRight <= minSpeed&& pidLeft > minSpeed){ // Turns right 
    motorRight.MoveBackwards(pidRight);
    motorLeft.MoveForward(pidLeft);
  } else if (pidLeft <= minSpeed && pidRight > minSpeed){ // Turns left
    motorRight.MoveForward(pidRight);
    motorLeft.MoveBackwards(pidLeft);
  } else { // Goes stright
    motorRight.MoveForward(pidRight);
    motorLeft.MoveForward(pidLeft);
  }
}

/* End of sprinter section
--------------------------------------------------------------------------*/
/* Area cleaner section */

#define PIN_SIG 34

// Sharp
int sharp_left;
int sharp_right;
int sharp_front_right;
int sharp_front;
int sharp_front_left;

int qre_right;
int qre_left;
int qre_back;

int touch_speed = 90;
int low_speed = 100;
int mid_speed = 150;
int full_speed = 200;

int signal_input;

#define PIN_MA1 26
#define PIN_MA2 27
#define PIN_MB1 16
#define PIN_MB2 17

CD74HC4067 my_mux(4, 25, 33, 32); // s0, s1, s2, s3

void MoveLeft() {

  analogWrite(PIN_MA1, mid_speed);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, mid_speed);

}

void MoveRight() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, mid_speed);
  analogWrite(PIN_MB1, mid_speed);
  analogWrite(PIN_MB2, 0);

}

void MoveSoftLeft() {

  analogWrite(PIN_MA1, low_speed);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, low_speed);

}

void MoveTouchLeft() {

  analogWrite(PIN_MA1, touch_speed);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, touch_speed);

}

void MoveSoftRight() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, low_speed);
  analogWrite(PIN_MB1, low_speed);
  analogWrite(PIN_MB2, 0);

}

void MoveTouchRight() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, touch_speed);
  analogWrite(PIN_MB1, touch_speed);
  analogWrite(PIN_MB2, 0);

}

void MoveStop() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, 0);

}

void MoveReverse() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, full_speed);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, full_speed);

}

void MoveForward() {
  
  analogWrite(PIN_MA1, full_speed);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, full_speed);
  analogWrite(PIN_MB2, 0);

}

void MoveSoftForward() {
  
  analogWrite(PIN_MA1, low_speed);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, low_speed);
  analogWrite(PIN_MB2, 0);

}

void MoveTouchForward() {
  
  analogWrite(PIN_MA1, touch_speed);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, touch_speed);
  analogWrite(PIN_MB2, 0);

}

void MoveSlowForward() {
  
  analogWrite(PIN_MA1, 90);
  analogWrite(PIN_MA2, 0);
  analogWrite(PIN_MB1, 90);
  analogWrite(PIN_MB2, 0);

}

void MoveBackwards() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, full_speed);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, full_speed);

}

void MoveSoftBackwards() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, low_speed);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, low_speed);

}

void MoveTouchBackwards() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, touch_speed);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, touch_speed);

}

void MoveMidBackwards() {

  analogWrite(PIN_MA1, 0);
  analogWrite(PIN_MA2, mid_speed);
  analogWrite(PIN_MB1, 0);
  analogWrite(PIN_MB2, mid_speed);

}


void ReadCleanerSensors() {

  //int datoBT;

  //datoBT = //.read();

  for (int x = 8; x < 15; x++) {
    
    my_mux.channel(x);
  
    signal_input = analogRead(PIN_SIG);

    switch (x) {

      case 8: {
        
        sharp_right = signal_input;
        
        ////.print("Right sharp =");
        ////.println(signal_input);

      }
      case 9: {
        
        sharp_front_right = signal_input;

        ////.print("Front Right sharp =");
        ////.println(signal_input);

      }
      case 10: {
        
        sharp_front = signal_input;

        ////.print("Front sharp =");
        ////.println(signal_input);

      }
      case 11: {
        
        sharp_front_left = signal_input;
        
        ////.print("Front Left sharp =");
        ////.println(signal_input);
      
      }
      case 12: {
        
        sharp_left = signal_input;
        
        ////.print("Left sharp =");
        ////.println(signal_input);
        
      } 
      case 13: {

        qre_right = signal_input;

        ////.print ("Right = ");
        ////.println (signal_input);

      }
      case 14: {

        qre_back = signal_input;

        //.print ("Back = ");
        //.println (signal_input);
      
      }
      case 15: {

        qre_left = signal_input;

        ////.print ("Left = ");
        ////.println (signal_input);

      }
    } 
  
  }  

}

/*#define QRE_BLACK   3900
#define SharpAtaque 1000
#define set_doTime  750

bool object_left;
bool object_right;
bool offRoad;

void StartAreaCleanerModality(){
  ReadCleanerSensors();

  //.println (object_left);
  //.println (object_right);

  int Action;
  int ActionQRE;

  unsigned long time = 0;
  


  if (sharp_front > SharpAtaque) {
    Action = 'F';
    //.println ("Front");
  } 
  else if (sharp_front < SharpAtaque && sharp_left > SharpAtaque && sharp_right < SharpAtaque) {
    Action = 'L';
    //.println ("Left");
  }
  else if (sharp_front_left > SharpAtaque) {
    Action = 'F';
    //.println ("Front");
  }
  else if (sharp_front_right > SharpAtaque) {
    Action = 'F';
    //.println ("Front");
  }
  else if (sharp_front < SharpAtaque && sharp_left < SharpAtaque && sharp_right > SharpAtaque) {
    Action = 'R';
    //.println ("Right");
  }
  else if (sharp_front < SharpAtaque && sharp_left < SharpAtaque && sharp_right < SharpAtaque) {
    Action = 'B';
    //.println ("B");
  }


  
  if (qre_left > QRE_BLACK || qre_right > QRE_BLACK) {
    ActionQRE = 'QRE_FRONT';
    //.println ("QRE_FRONT");
  }
  else if (qre_left < QRE_BLACK && qre_right < QRE_BLACK && qre_back > 3700) {
    ActionQRE = 'QRE_BACK';
    //.println ("QRE_BACK");
  }
  else if (qre_left > QRE_BLACK && qre_right > QRE_BLACK && qre_back > QRE_BLACK) {
    ActionQRE = 'QRE_ALL';
  }


  if (qre_left > QRE_BLACK || qre_right > QRE_BLACK || qre_back > QRE_BLACK) {
    offRoad = 1;
    //.println ("OFF_ROAD");
  } 
  else if (qre_left < QRE_BLACK && qre_right < QRE_BLACK && qre_back < QRE_BLACK) {
    offRoad = 0;
    //.println ("IN_ROAD");
  }


switch (offRoad) {
  
  
  case 1: {
  switch (ActionQRE) {

    case 'QRE_FRONT': {

      time = millis();

      MoveStop();
      delay (50);

      while (millis() < time + set_doTime) {
        MoveMidBackwards();
        delay (10);
        //.println ("while Backwards");
      }

      ReadCleanerSensors();
    
      //.println ("OUT");

      break;
    }

    case 'QRE_BACK': {

      time = millis();

      MoveStop();
      delay (50);

      while (millis() < time + set_doTime) {
        MoveSoftForward();
        delay (10);
        //.println ("QRE_BACK Forward");
      }

      ReadCleanerSensors();

      break;
    }

    case 'QRE_ALL': {
      
      time = millis();

      MoveStop();
      delay (50);

      while (millis() < time + set_doTime) {
        MoveMidBackwards();
        delay (10);
        //.println ("QRE_BACK Forward");
      }
      ReadCleanerSensors();
    }
  }
  break;
}
  
  
  
  case 0: {
  switch (Action) {
    case 'F': {
      MoveSoftForward();
      object_left = false;
      object_right = false;
      ReadCleanerSensors();
      break;
    }

    case 'L': {
      object_left = true;
      MoveSoftLeft();
      ReadCleanerSensors();
      break;
    }

    case 'R': {
      object_right = true;
      MoveSoftRight();
      ReadCleanerSensors();
      break;
    }

    case 'B': {
      if (object_left) {
        MoveSoftLeft();
      }
      else if (object_right) {
        MoveSoftRight();
      }
      else {
        MoveSoftLeft();
      }
      break;
    }

    /*case 'B': {
      MoveSoftLeft();
    }
  }
  break;
}

  
}
}*/

#define QRE_BLACK   3900
#define SharpAtaque 1000
#define set_doTime  800

bool object_left;
bool object_right;
bool offRoad;

void StartAreaCleanerModality(){
  ReadCleanerSensors();

  //.println (object_left);
  //.println (object_right);

  int Action;
  int ActionQRE;
  

  unsigned long time = 0;




  if (sharp_front > SharpAtaque) {
    Action = 'F';
    //.println ("Front");
  } 
  else if (sharp_front < SharpAtaque && sharp_left > SharpAtaque && sharp_right < SharpAtaque) {
    Action = 'L';
    //.println ("Left");
  }
  else if (sharp_front_left > SharpAtaque) {
    Action = 'F';
    //.println ("Front");
  }
  else if (sharp_front_right > SharpAtaque) {
    Action = 'F';
    //.println ("Front");
  }
  else if (sharp_front < SharpAtaque && sharp_left < SharpAtaque && sharp_right > SharpAtaque) {
    Action = 'R';
    //.println ("Right");
  }
  else if (sharp_front < SharpAtaque && sharp_left < SharpAtaque && sharp_right < SharpAtaque) {
    Action = 'B';
    //.println ("B");
  }



  if (qre_left > QRE_BLACK || qre_right > QRE_BLACK) {
    ActionQRE = 'QRE_FRONT';
    //.println ("QRE_FRONT");
  }
  else if (qre_left < QRE_BLACK && qre_right < QRE_BLACK && qre_back > QRE_BLACK) {
    ActionQRE = 'QRE_BACK';
    //.println ("QRE_BACK");
  }
  else if (qre_left > QRE_BLACK && qre_right > QRE_BLACK && qre_back > QRE_BLACK) {
    ActionQRE = 'QRE_ALL';
  }

  
  if (qre_left > QRE_BLACK || qre_right > QRE_BLACK || qre_back > QRE_BLACK) {
    offRoad = 1;
    //.println ("OFF_ROAD");
  } else if (qre_left < QRE_BLACK && qre_right < QRE_BLACK && qre_back < QRE_BLACK) {
    offRoad = 0;
    //.println ("IN_ROAD");
  }

switch (offRoad) {
  
  case 0: {
  switch (Action) {
    case 'F': {
      MoveSoftForward();
      object_left = false;
      object_right = false;
      break;
    }

    case 'L': {
      object_left = true;
      MoveSoftLeft();
      break;
    }

    case 'R': {
      object_right = true;
      MoveSoftRight();
      break;
    }

    case 'B': {
      if (object_left) {
        MoveSoftLeft();
      }
      else if (object_right) {
        MoveSoftRight();
      }
      else{
        MoveSoftLeft();
      }
      break;
    }

    /*case 'B': {
      MoveTouchLeft();
    }*/
  }
  break;
}

  case 1: {
  switch (ActionQRE) {

    case 'QRE_FRONT': {

      time = millis();

      /*MoveStop();
      delay (50);*/

      while (millis() < time + set_doTime) {
        MoveMidBackwards();
        delay (10);
        //.println ("while Backwards");
      }
    
      //.println ("OUT");

      break;
    }

    case 'QRE_BACK': {

      time = millis();

      /*MoveStop();
      delay (50);*/

      while (millis() < time + set_doTime) {
        MoveForward();
        delay (10);
        //.println ("QRE_BACK Forward");
      }

      ReadCleanerSensors();

      break;
    }

    case 'QRE_ALL': {
      time = millis();

      /*MoveStop();
      delay (50);*/

      while (millis() < time + set_doTime) {
        MoveSoftBackwards();
        delay (10);
        //.println ("QRE_BACK Forward");
      }

      ReadCleanerSensors();

      break;
    }
  }
  break;
}
}
}

/* End of area cleaner section
--------------------------------------------------------------------------*/
/* Sumo section */

bool dataUpdated;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
  
  bool foundEmptySlot = false;

  ctlConnected = true;
  
  if (myControllers[0] == nullptr) {
    Serial.printf("CALLBACK: Controller is connected, index=%d\n");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    myControllers[0] = ctl;
    foundEmptySlot = true;
  }

  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  ctlConnected = false;
    
  if (myControllers[0] == ctl) {
    Serial.printf("CALLBACK: Controller disconnected from index=%d\n");
    myControllers[0] = nullptr;
    foundController = true;
  } 

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  bool brakeRight;
  bool brakeLeft;

  // Variables to store the input of each stick
  int yAxisValueR = ctl->axisRY();
  int yAxisValueL = ctl->axisY();

  int RBValue = ctl->throttle();
  int LBValue = ctl->brake();

  // Mapping of each stick input to 8bit 
  int rightWheelSpeedF = map(yAxisValueR, 0, -511, 0, 180);
  int rightWheelSpeedB = map(yAxisValueR, 0, 512, 0, 180);
  
  int leftWheelSpeedF = map(yAxisValueL, 0, -511, 0, 180);
  int leftWheelSpeedB = map(yAxisValueL, 0, 512, 0, 180);

  int forwardBoost = map(RBValue, 0, 1023, 0, 255);
  int backwardsBoost = map(LBValue, 0, 1023, 0, 255);

  if (ctl->r1()) {brakeRight = true;}
  else {brakeRight = false;}
  if (ctl->l1()) {brakeLeft = true;}
  else {brakeLeft = false;}

  if (!brakeRight) {
    if (yAxisValueR < -70) {motorRight.MoveForward(rightWheelSpeedF);}
    else if (yAxisValueR > 70) {motorRight.MoveBackwards(rightWheelSpeedB);}
    else {motorRight.StayStill();}
    
    if (yAxisValueL < -70) {motorLeft.MoveForward(leftWheelSpeedF);}
    else if (yAxisValueL > 70) {motorLeft.MoveBackwards(leftWheelSpeedB);}
    else {motorLeft.StayStill();}
  }
  else {
    motorRight.MoveBackwards(60);
    motorLeft.MoveBackwards(60);
  }

  if (RBValue > 150) {
    motorRight.MoveForward(forwardBoost);
    motorLeft.MoveForward(forwardBoost);
  }

  if (LBValue > 150) {
    motorRight.MoveBackwards(backwardsBoost);
    motorLeft.MoveBackwards(backwardsBoost);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      processGamepad(myController);
    }
  }
}

void StartSumoModality(){
  BP32.enableNewBluetoothConnections(true);

  dataUpdated = BP32.update();
  
  if (dataUpdated) {
    processControllers();
  }
}

/* End of sumo section
--------------------------------------------------------------------------*/
/* Triggers section*/

void StartModalityTriggers() {
  // Calibration trigger
  if (current_screen == calibration){StartSprinterCalibration();}

  // Sumo trigger
  else if (current_screen == modality && selected == sumo){StartSumoModality();}

  // Area cleaner trigger
  else if (current_screen == flags && selected == areaCleaner){StartAreaCleanerModality();}

  // Sprinter trigger
  else if (current_screen == flags && selected == sprinter){StartSprinterModality();}
}

/* End of triggers section
--------------------------------------------------------------------------*/
/* Setup and loop section */

#define PIN_LED 23

void setup(){    
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // Begin display connection
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {    
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);  
  }

  u8g2_for_adafruit_gfx.begin(display); // Begins u8g2 for gfx library

  BP32.setup(&onConnectedController, &onDisconnectedController);

  pinMode(PIN_SELECT, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);

  // Displays team logo
  display.clearDisplay();
  display.drawBitmap( 0, 0, bitmap_alita, 128, 64, WHITE); // Prints teams logo
  display.display();

  delay(3000);
}

void loop() {
  DisplayMenu();
  StartModalityTriggers();           
}