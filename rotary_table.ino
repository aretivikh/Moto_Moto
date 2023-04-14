#include <AccelStepper.h>
#define DIR_PIN 2
#define STEP_PIN 3

int MOTOR_STEPS=200;
int MICRO_STEPS=2;
int STEPS=MOTOR_STEPS*MICRO_STEPS;
int MAX_STEPS=360*STEPS*20000;
int degree;
int move_to;

// set up stepper motor driver
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);  // STEP pin = 2, DIR pin = 3

void setup() {
  // initialize serial communication
  Serial.begin(9600);

  // set up motor parameters
  stepper.setMaxSpeed(2500);
  stepper.setAcceleration(2000);
  Serial.print("Position ");
  Serial.println(stepper.currentPosition());
  Serial.print("STEPS: ");
  Serial.print(STEPS);
  Serial.println();
}

void calculate_steps(){
  // 360 / (STEPS * MICRO_STETS)
  // need 1600 steps do 360
  // one_degree =
}

void loop() {
  // wait for command from Python program
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("command : ");
    Serial.println(command.substring(3));
    Serial.println(command.substring(3).toInt());

    Serial.print("Current pos ");
    Serial.println(stepper.currentPosition());

    // parse command and execute corresponding action
    if (command.startsWith("cw")) {
      degree = command.substring(3).toInt();
      move_to = (stepper.currentPosition()+command.substring(3).toInt());
      //  Serial.print("degree ");
      Serial.println(stepper.currentPosition()+command.substring(3).toInt());
      Serial.print("move_to ");
      Serial.println(move_to);
      // if (move_to <= MAX_STEPS) {

      stepper.moveTo(command.substring(3).toInt());
      stepper.runToPosition();
      Serial.print("CW after pos ");
      Serial.println(stepper.currentPosition());
      stepper.setCurrentPosition(0.0);
      // }
      // else {
      //   Serial.print("Number of steps more maximum. It will be the second revolution");
      //   }
      }


    else if (command.startsWith("ccw")) {
      int degree = command.substring(4).toInt();
      int move_to = stepper.currentPosition()-(STEPS*degree);
      if (move_to <= MAX_STEPS) {
          Serial.print("move_to ");
          Serial.println(move_to);
          Serial.print("-(STEPS*degree) ");
          Serial.println(-(STEPS*degree));
          stepper.moveTo(move_to);
          stepper.runToPosition();
          Serial.print("runing ");
          Serial.print(stepper.isRunning());

          Serial.print("CW after pos ");
          Serial.println(stepper.currentPosition());
          Serial.print("CCW after pos ");
          Serial.println(stepper.currentPosition());
      }
      else {
        Serial.print("Number of steps reach maximum. It will be the second revolution");
        move_to=(MAX_STEPS - 20);
      }
    }
    else if (command.startsWith("r")) {
      stepper.setCurrentPosition(0);
      Serial.print("Pos set to ");
      Serial.print(stepper.currentPosition());
    }
    else if (command.startsWith("home")) {
      int move_to = stepper.currentPosition(); //it means move to oposit direction but not farther than HOME
      Serial.print("move_to ");
      Serial.println(-move_to);
      // Serial.print("-(STEPS*degree) ");
      // Serial.println(-(STEPS*degree));
      stepper.moveTo(move_to);
      stepper.runToPosition();
    }
  }
}