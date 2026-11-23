#include <SimpleFOC.h>
#include <AccelStepper.h>
#include <MultiStepper.h> 

AccelStepper stepperTurret(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperElbow(AccelStepper::DRIVER, 3, 6);
AccelStepper stepperBase(AccelStepper::DRIVER, 4, 7);

MultiStepper steppers;


MagneticSensorAnalog sensorL = MagneticSensorAnalog(PC1, 14, 1020);
MagneticSensorAnalog sensorR = MagneticSensorAnalog(PA4, 14, 1020);
MagneticSensorAnalog sensorT = MagneticSensorAnalog(PC2, 14, 1020);


BLDCDriver3PWM driverL = BLDCDriver3PWM(PA0, PA1, PA15, PC4); //PA11, PA9, PA10, PA12 //L
BLDCMotor motorL = BLDCMotor(7);

BLDCDriver3PWM driverR = BLDCDriver3PWM(PC6, PC7, PC8, PC5); //PB6, PB8, PB9, PB5 //R
BLDCMotor motorR = BLDCMotor(7);

BLDCDriver3PWM driverT = BLDCDriver3PWM(PB6, PB8, PB9, PB5); //T
BLDCMotor motorT = BLDCMotor(7);

float angleL = 3;
float angleR = 0;
float angleT = 0;

float angleElbow = 0;
float angleBase = 0;
float angleTurret = 0;
Commander command = Commander(Serial);
void doAngleL(char* cmd) { command.scalar(&angleL, cmd); }
void doAngleR(char* cmd) { command.scalar(&angleR, cmd); }
void doAngleT(char* cmd) { command.scalar(&angleT, cmd); }

void doAngleElbow(char* cmd) { command.scalar(&angleElbow, cmd); }
void doAngleBase(char* cmd) { command.scalar(&angleBase, cmd); }
void doAngleTurret(char* cmd) { command.scalar(&angleTurret, cmd); }


bool estop = false;
bool stepperStop = false;


void setup() {
  Serial.begin(115200);

  // Stepper motor setup
  // Configure each stepper
  

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepperTurret);
  steppers.addStepper(stepperBase);
  steppers.addStepper(stepperElbow);

  stepperTurret.setMaxSpeed(1000);
  stepperBase.setMaxSpeed(1000);
  stepperElbow.setMaxSpeed(1000);

  stepperTurret.setAcceleration(250); // Set acceleration value for the stepper
  stepperBase.setAcceleration(250);
  stepperElbow.setAcceleration(250);

  Serial.println(F("Stepper Motors Ready."));

  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensorL.init();
  sensorR.init();
  sensorT.init();
  // // link the motor to the sensor
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);
  motorT.linkSensor(&sensorT);


  // driver config
  //power supply voltage [V]
  driverL.voltage_power_supply = 12;
  driverL.init();
  driverR.voltage_power_supply = 12;
  driverR.init();
  driverT.voltage_power_supply = 12;
  driverT.init();

  // link the motor and the driver
  motorL.linkDriver(&driverL);
  motorR.linkDriver(&driverR);
  motorT.linkDriver(&driverT);

  // // choose FOC modulation (optional)
  motorL.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorR.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorT.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motorL.controller = MotionControlType::angle;
  motorR.controller = MotionControlType::angle;
  motorT.controller = MotionControlType::angle;

  // velocity PI controller parameters
  motorL.PID_velocity.P = 0.1f;
  motorL.PID_velocity.I = 10;
  motorL.PID_velocity.D = 0;

  motorR.PID_velocity.P = 0.1f;
  motorR.PID_velocity.I = 10;
  motorR.PID_velocity.D = 0;

  motorT.PID_velocity.P = 0.1f;
  motorT.PID_velocity.I = 10;
  motorT.PID_velocity.D = 0;

  // maximal voltage to be set to the motor
  motorL.voltage_limit = 6;
  motorR.voltage_limit = 6;
  motorT.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motorL.LPF_velocity.Tf = 0.01f;
  motorR.LPF_velocity.Tf = 0.01f;
  motorT.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motorL.P_angle.P = 15;
  motorR.P_angle.P = 15;
  motorT.P_angle.P = 15;

  // maximal velocity of the position control
  motorL.velocity_limit = 10;
  motorR.velocity_limit = 10;
  motorT.velocity_limit = 10;
  
  // comment out if not needed
  motorL.useMonitoring(Serial);
  motorR.useMonitoring(Serial);
  motorT.useMonitoring(Serial);

  // initialize motor
  motorL.init();
  motorR.init();
  motorT.init();

  // align sensor and start FOC
  motorL.initFOC();
  motorR.initFOC();
  motorT.initFOC();

  // add target command T
  command.add('L', doAngleL, "target angle L");
  command.add('R', doAngleR, "target angle R");
  command.add('T', doAngleT, "target angle T");

  command.add('E', doAngleElbow, "target angle elbow");
  command.add('B', doAngleBase, "target angle base");
  command.add('Y', doAngleTurret, "target angle turret");
  
  Serial.println(F("FOC ready."));

  

  Serial.println("starting Loop.");
  _delay(1000);

}

void loop() {
    //   stepperTurret.moveTo(550); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
    //   // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position
      

    //   stepperElbow.moveTo(1600); // 0 - 3150

    //   stepperElbow.runToPosition();
      
    //   stepperTurret.runToPosition();
      
    //   stepperBase.moveTo(400);
    //   stepperBase.runToPosition();
      
    //   // Move back to position 0, using run() which is non-blocking - both motors will move at the same time
    //   stepperElbow.moveTo(0);
    //   stepperTurret.moveTo(0);
    //   stepperBase.moveTo(0);
      
      // sensorL.update();
      // sensorR.update();
      // sensorT.update();
      // // display the angle and the angular velocity to the terminal
      // Serial.print(sensorL.getAngle());
      // Serial.print(" | ");
      // Serial.print(sensorR.getAngle());
      // Serial.print(" | ");
      // Serial.print(sensorT.getAngle());
      // Serial.println(" | ");

    //   while (stepperBase.currentPosition() != 0 || stepperTurret.currentPosition() != 0) {
    //     stepperBase.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    //     stepperTurret.run();
    //     stepperElbow.run();
    //   }
    // }
    
    
    motorL.loopFOC();
    motorR.loopFOC();
    motorT.loopFOC();

    // motorL.target = angleL;
    motorL.move(angleL);
    motorR.move(angleL);
    motorT.move(angleT);

    command.run();
  

}
