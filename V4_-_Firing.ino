// Load libraries             
#include <Wire.h>
#include <DFRobotIRPosition.h>
#include <Samco.h>
#include <Mouse.h>
#include <ArduinoLog.h>
#include <EasyButton.h>
#include <math.h>

int xCenter = 512;          // If second calibration seems more accurate you can replace these values with the altered camera center values from serial monitor
int yCenter = 368;
         
int yTop = 0;
int yBottom = 0; 
int xLeft = 0;  
int xRight = 0;   

// Values after tilt correction 
int finalX = 0;                 
int finalY = 0;

int MoveXAxis = 0;
int MoveYAxis = 0;

int conMoveXAxis = 0;
int conMoveYAxis = 0;

int joystickX = 0;
int joystickY = 0;

//int res_x = 1920;
//int res_y = 1080;

int res_x = 1080; // Vertical
int res_y = 1920;

//int res_x = 1023;
//int res_y = 768;
//int res_x = 767; // verical mode
//int res_y = 1023;


float calibrateX = 0;
float calibrateY = 0;


/* ----Smoothing--- */

// Define the number of data points to include in the moving average
const int numDataPoints = 5;  // You can adjust this value as needed

// Create an array to store the previous data points
int previousXValues[numDataPoints];
int previousYValues[numDataPoints];
int currentIndex = 0;

int smoothedX = 0;
int smoothedY = 0;  


/* ---------------- */

enum CalibrationState {
  CALIBRATION_UNKNOWN,
  CALIBRATION_CHECKING,
  CALIBRATION_NEEDED,
  CALIBRATION_CALIBRATING,
  CALIBRATION_DONE
};

enum CalibrationStep {
  CALIBRATING_STEP_NOT_STARTED,
  CALIBRATING_STEP_MIDDLE,
  CALIBRATING_STEP_TOP_MIDDLE,
  CALIBRATING_STEP_BOTTOM_MIDDLE,
  CALIBRATING_STEP_LEFT_MIDDLE,
  CALIBRATING_STEP_RIGHT_MIDDLE,
  CALIBRATING_STEP_COMPLETE
};

CalibrationState calibrationState = CALIBRATION_UNKNOWN;
CalibrationStep calibrationStep = CALIBRATING_STEP_NOT_STARTED;

DFRobotIRPosition myDFRobotIRPosition;
Samco mySamco;

/* ---------------------- Calibration Stuff --------------------------- */


int firingButtonPin = 10;
int calibrateButtonPin = 10;
const int calibrateLongPressTime = 1000;

// (calibrateButtonPin, debounce, pullup, invert);
EasyButton calibrateButton(calibrateButtonPin, 50, true, true); 
EasyButton firingButton(calibrateButtonPin, 50, true, true); 

void checkCalibrationState(){
  calibrationState = CALIBRATION_NEEDED;
  
  if(calibrationState == CALIBRATION_NEEDED){
    calibrationState == CALIBRATION_CALIBRATING;
    calibrateOnPressed();
  }
}

void calibrateOnPressedForDuration() {
  calibrationState = CALIBRATION_CALIBRATING;
  calibrationStep = CALIBRATING_STEP_TOP_MIDDLE;
  
  Serial.println("Entering Calibration..\n"
    CR);
  Serial.println("Waiting for TOP MIDDLE \n"
    CR);

  Serial.println("2");

  Joystick.X(res_x / 2);
  Joystick.Y(90);
  Joystick.send_now();
}

void calibrateOnPressed() {
//  digitalWrite (14, LOW);
//  digitalWrite (16, HIGH);

  if(calibrationState == CALIBRATION_CALIBRATING){
    if (calibrationStep == CALIBRATING_STEP_TOP_MIDDLE) {
      Serial.println("CALIBRATING_TOP_MIDDLE");
      calibrationStep = CALIBRATING_STEP_BOTTOM_MIDDLE;

      getPosition();
      // pitchBackCalibrationValue = finalY;
      
      yTop = finalY;

      Serial.println("Waiting for BOTTOM MIDDLE\n"
    CR);

      Joystick.X(res_x / 2);
      Joystick.Y(res_y);
      Joystick.send_now();
      
    } else if (calibrationStep == CALIBRATING_STEP_BOTTOM_MIDDLE) {
      Serial.println("CALIBRATING_BOTTOM_MIDDLE");
      calibrationStep = CALIBRATING_STEP_LEFT_MIDDLE;
      
      getPosition();
      // pitchBackCalibrationValue = finalY;
      yBottom = finalY;

      Serial.println("Waiting for LEFT MIDDLE\n"
    CR);

      Joystick.X(50);
      Joystick.Y(400);
      Joystick.send_now();
      
    } else if (calibrationStep == CALIBRATING_STEP_LEFT_MIDDLE) {
      Serial.println("CALIBRATING_LEFT_MIDDLE");
      calibrationStep = CALIBRATING_STEP_RIGHT_MIDDLE;

      getPosition();
      // pitchBackCalibrationValue = finalY;
      xLeft = finalX;

      Serial.println("Waiting for RIGHT MIDDLE\n"
    CR);

      Joystick.X(res_x - 100);
      Joystick.Y(400);
      Joystick.send_now();
      
    } else if (calibrationStep == CALIBRATING_STEP_RIGHT_MIDDLE){
      Serial.println("CALIBRATING_STEP_RIGHT_MIDDLE");
      calibrationState = CALIBRATION_DONE;

      getPosition();
      
      xRight = finalX;

      xCenter = ((xRight - xLeft) / 2) + min(xRight, xLeft);
      yCenter = ((yBottom - yTop) / 2) + min(yBottom, yTop);

      Serial.println("CALIBRATION COMPLETE\n"
    CR);

    }
  } 
}



/* ------------------------------------------------------------------- */

/* ---------------------- 4IR Tracking Stuff --------------------------- */
void getPosition() {

myDFRobotIRPosition.requestPosition();
    if (myDFRobotIRPosition.available()) {
//      if(calibrationStep == CALIBRATING_STEP_MIDDLE){
//         // Read the X and Y coordinates of the four IR LEDs
//        float x0 = myDFRobotIRPosition.readX(0);
//        float y0 = myDFRobotIRPosition.readY(0);
//        float x1 = myDFRobotIRPosition.readX(1);
//        float y1 = myDFRobotIRPosition.readY(1);
//        float x2 = myDFRobotIRPosition.readX(2);
//        float y2 = myDFRobotIRPosition.readY(2);
//        float x3 = myDFRobotIRPosition.readX(3);
//        float y3 = myDFRobotIRPosition.readY(3);
//    
//        // Calculate the center X and Y coordinates
//        xCenter = (x0 + x1 + x2 + x3) / 4.0;
//        yCenter = (y0 + y1 + y2 + y3) / 4.0;
//      }
//
      mySamco.begin(
        myDFRobotIRPosition.readX(0), myDFRobotIRPosition.readY(0), 
        myDFRobotIRPosition.readX(1), myDFRobotIRPosition.readY(1),
        myDFRobotIRPosition.readX(2), myDFRobotIRPosition.readY(2),
        myDFRobotIRPosition.readX(3), myDFRobotIRPosition.readY(3), 
        xCenter, yCenter);
        
      finalX = mySamco.X();
      finalY = mySamco.Y();
    }
    else {
    Serial.println("Device not available!");
    }
}

void PrintResults() {    
  Serial.print("RAW: ");
  Serial.print(finalX);
  Serial.print(", ");
  Serial.print(finalY);
  
  Serial.print(" : ");
  
  Serial.print("MAP MoveAxis: ");
  Serial.print(MoveXAxis);
  Serial.print(", ");
  Serial.print(MoveYAxis);

  Serial.print(" : ");
  
  Serial.print("MAP conMoveAxis: ");
  Serial.print(conMoveXAxis);
  Serial.print(", ");
  Serial.print(conMoveYAxis);

  Serial.print(" : ");
  
  Serial.print("Smoothed: ");
  Serial.print(smoothedX);
  Serial.print(", ");
  Serial.println(smoothedY);

  Serial.println(" ");
}

//void resetArrow() {
//  //todo: clear timedHallTriggers
//  arrowState = ARROW_STATE_RESETTING;
//  delay(250);
//  timePointCounter = 0;
//  
//  Joystick.button(1, 0); // Joystick button 1 indicates fired
//  Joystick.button(3, 0); // Joystick button 3 indicates drawing
//  Joystick.button(4, 0); // Joystick button 4 indicates fireing
//  //digitalWrite(LED_BUILTIN, LOW);
//  Serial.print("Reset event"
//    CR);
//   arrowState = ARROW_STATE_IDLE;
//   lastArrowEvent = 0;
//
//}
//
//void notifyJoyOfArrowState(int localArrowState){
//
//  if(localArrowState == ARROW_STATE_DRAWING){
//      Joystick.button(3, 1); // Joystick button 3 indicates drawing
//      Joystick.button(4, 0); // Joystick button 4 indicates fireing 
//  } else if(localArrowState == ARROW_STATE_FIRING){
//      Joystick.button(4, 1); // Joystick button 4 indicates fireing
//      Joystick.button(3, 0); // Joystick button 3 indicates drawing
//  } else if(localArrowState == ARROW_STATE_CALCULATING){
//      Joystick.button(3, 0); //Joystick button 3 indicates drawing
//      Joystick.button(4, 0); // Joystick button 4 indicates fireing
//  } else if(localArrowState == ARROW_STATE_LAUNCHED){
//      Joystick.button(3, 0); //Joystick button 3 indicates drawing
//      Joystick.button(4, 0); // Joystick button 4 indicates fireing
//      Joystick.button(1, 1); //Joystick button 1 indicates a calculated fire
//  }
//
//      delay(5);
//      Joystick.send_now();
//      
//}
//
//
//void arrowEvent() {
//    
//}


void fireArrowJoystick() {
//  Joystick.button(1, 0);
//  Joystick.button(4, 0);

    Joystick.button(3, 0); // Joystick button 3 indicates drawing
    Joystick.button(4, 0); // Joystick button 4 indicates fireing
    Joystick.button(1, 1); //Joystick button 1 indicates a calculated fire

    Joystick.button(3, 1); // Joystick button 3 indicates drawing
    Joystick.button(4, 0); // Joystick button 4 indicates fireing 

    Joystick.send_now();

  
  Serial.println("Fire Button Pressed ");
}

//void fireArrowMouse() {
////  Joystick.button(1, 0);
////  Joystick.button(4, 0);
//
//  // Simulate a left mouse click
//  Mouse.click(MOUSE_LEFT);
//
//  // Wait for a moment (you can adjust the delay as needed)
//  delay(200); // 1 second
//
//  // Release the left mouse button (optional)
//  Mouse.release(MOUSE_LEFT);
//
//  Serial.println("Fire Button Pressed ");
//}

void setup() {
  Serial.begin(9600);  

//  Mouse.begin();

  // Initialize the previous data points to zero
  for (int i = 0; i < numDataPoints; i++) {
    previousXValues[i] = 0;
    previousYValues[i] = 0;
  }

  myDFRobotIRPosition.begin();

  Joystick.useManualSend(true);

  calibrateButton.begin();
  calibrateButton.onPressed(calibrateOnPressed);
  calibrateButton.onPressedFor(calibrateLongPressTime, calibrateOnPressedForDuration);

//  Mouse.begin();
  firingButton.begin();

  delay(500);
}


void loop() {

  calibrateButton.read();


  if(calibrationState == CALIBRATION_UNKNOWN){
    checkCalibrationState();
  }
  else if (calibrationState == CALIBRATION_DONE){
    firingButton.read();
    
    getPosition();

    
    firingButton.onPressed(fireArrowJoystick);

    int sensitivityReduction = 2;

    smoothedX /= sensitivityReduction;
    smoothedY /= sensitivityReduction;
    
    // Map the smoothed values to screen resolution or apply adjustments
//    MoveXAxis = map (finalX, xLeft, xRight, 100, (res_x - 100));
//    MoveYAxis = map (finalY, yTop, yBottom, 90, (res_y - 100));
    MoveXAxis = map (smoothedX, xLeft, xRight, 100, res_x - 100);
    MoveYAxis = map (smoothedY, yTop, yBottom, 90, res_y - 100);
//    conMoveXAxis = constrain (MoveXAxis, 0, res_x);
//    conMoveYAxis = constrain (MoveYAxis, 0, res_y);


    /*-------------------------*/

    Joystick.X(MoveXAxis);
    Joystick.Y(MoveYAxis);
    Joystick.send_now();

    
    PrintResults();
  }

}
