package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by robow_000 on 3/22/2016.
 */
public class RedShort extends OpMode{
    private enum State {
        MOVE_TO_FAR_SIDE,
        TURN_TO_WHITE_LINE,
        MOVE_TO_WHITE_LINE,
        RAISE_ARM,
        DUMP_CLIMBERS,
        LOWER_ARM,
        MOVE_TO_MOUNTAIN,
        TURN_TO_MOUNTAIN_BASE,
        MOVE_TO_MOUNTAIN_BASE,
        TURN_TO_MOUNTAIN,
        MOVE_UP_MOUNTAIN
    }

    private State currentState;
    private void changeToState(State newState){
        timeInState.reset();
        currentState = newState;
    }

    DcMotor motorRoller;
    DcMotor motorHangWinch;
    DcMotor motorTurretRotate;
    DcMotor motorArmUpDown;
    DcMotor motorLeft;
    DcMotor motorRight;
    Servo servoDebrisGate;
    Servo servoHangerElbow;
    Servo servoRedPusher;
    Servo servoBluePusher;
    Servo servoBumper;
    Servo servoBucket;
    GyroSensor sensorGyro;
    ColorSensor sensorColor;

    static final int RED_IS_MINUS1 = -1;// +1 IF BLUE, for turns of robot and turret
    //*** Java assumes DOUBLE PRECISION, must specify FLOAT with #.##f
    static final float DGATE_CLOSE = 0.078f, DGATE_HOLDCLIMBERS = 0.11f, DGATE_OPEN = .3f;// Debris Gate
    static final float HANGER_IN = 0.11f, HANGER_OUT = 0.9f;// Hanger Hook
    static final float RED_UP = 0.83f, RED_MID = 0.15f, RED_DOWN = 0.0f;// Red Pusher Zipline Trigger
    static final float BLUE_UP = 0.15f, BLUE_MID = 0.75f, BLUE_DOWN = .9f;// Blue Pusher Zipline Trigger
    static final float BUMPER_UP = .66f, BUMPER_DOWN = .04f;// Back Bumper
    static final float BUCKET_LOAD = 0.07f, BUCKET_DUMP = 0.63f;// Collector Bucket
    static final int PUSHER_DOWN = 0, PUSHER_HALF = 1, PUSHER_UP = 2;
    static final int PUSHER_READY = 0, PUSHER_NOT_READY = 1;

    static final float ENCODER_CPI = 77.05f;// ~121 counts/inch with Hugo's treads
    float targetInches, targetEncCount, targetFuzzy, oldEncCount;
    int oldHeading, targetHeading, deltaHeading, gyroDrift, wiggleFlag, climbState;
    float currentPower, basePower =0.7f, deltaPower, deltaPowerMax = 1 - basePower;
    float rightPower, leftPower;

    float leftStickY, rightStickY, normalDrive = 0.7f;//slower motors for normal driving
    boolean bumperDown = true;
    int targetDistRed = 400, targetDistBlue = 400;
    int pusherStatus = PUSHER_READY, pusherMode = PUSHER_UP;
    float winchPower = 0, rollerPower, hangerPosition = .1f;
    float turretRotato = 0, bucketPos=.08f, bucketPower = .006f;
    int toggleTimer = 0, senseWhite, senseRed, senseBlue;

    public ElapsedTime timeInRound = new ElapsedTime();//timer for whole autonomous round
    public ElapsedTime timeInState = new ElapsedTime();//timer reset for each state
    public ElapsedTime timeTemp = new ElapsedTime();//timer for miscellaneous
    public float minTime, maxTime;//minimum and maximum times to stay in next state

    private int adjHeading(int heading){//offset for gyro drift
        if (heading > 180) {heading = heading - 360;}
        return heading;
    }// -179 to 0 to +180 instead of 181 to 359 to 0 to 180

    public void resetDriveEncoders() {
        motorLeft.setTargetPosition(0);
        motorRight.setTargetPosition(0);
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //Return true when completed, false if not done yet
    private Boolean moveDistanceWithHeading(int desiredHeading, double desiredDistance){
        telemetry.addData("Cd", motorRight.getCurrentPosition());
        if(motorRight.getCurrentPosition() < desiredDistance){
            motorRight.setPower(1);
            motorLeft.setPower(1);
            if(adjHeading(sensorGyro.getHeading()) > desiredHeading){
                motorRight.setPower(.5);
            }
            if(adjHeading(sensorGyro.getHeading()) < desiredHeading){
                motorLeft.setPower(.5);
            }
            return false;
        }else{
            motorRight.setPower(0);
            motorLeft.setPower(0);
            return true;
        }
    }

    private Boolean turnToHeading(int heading){
        int adjTargetHeading = adjHeading(heading);
        int adjCurrentHeading = adjHeading(sensorGyro.getHeading());
        if(Math.abs(adjCurrentHeading-adjTargetHeading) < 1){
            return true;
        }
        double speedModifier = Math.abs(adjCurrentHeading-adjTargetHeading) / 180;
        speedModifier = Range.clip(speedModifier, .5, 1);
        if(adjTargetHeading - adjCurrentHeading < 0) {
            motorRight.setPower(-1 * speedModifier);
            motorLeft.setPower(speedModifier);
        }else{
            motorRight.setPower(speedModifier);
            motorLeft.setPower(-1 * speedModifier);
        }
        return false;
    }

    @Override
    public void stop() {
    }

    @Override
    public void init() {
        telemetry.addData("0","...... Initializing .....");
        motorRoller = hardwareMap.dcMotor.get(ConstantsConfig.rollerMotorName);
        motorHangWinch = hardwareMap.dcMotor.get(ConstantsConfig.hangWinchName);
        motorTurretRotate = hardwareMap.dcMotor.get(ConstantsConfig.turretRotationMotorName);
        motorArmUpDown = hardwareMap.dcMotor.get(ConstantsConfig.turretUpDownName);
        motorLeft = hardwareMap.dcMotor.get(ConstantsConfig.motorLeftName);
        motorRight = hardwareMap.dcMotor.get(ConstantsConfig.motorRightName);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        servoDebrisGate = hardwareMap.servo.get(ConstantsConfig.debrisGateName);
        servoDebrisGate.setPosition(DGATE_HOLDCLIMBERS);
        servoHangerElbow = hardwareMap.servo.get(ConstantsConfig.hangerName);
        servoHangerElbow.setPosition(HANGER_IN);
        servoRedPusher = hardwareMap.servo.get(ConstantsConfig.redTriggerName);
        servoRedPusher.setPosition(RED_UP);
        servoBluePusher = hardwareMap.servo.get(ConstantsConfig.blueTriggerName);
        servoBluePusher.setPosition(BLUE_UP);
        servoBumper = hardwareMap.servo.get(ConstantsConfig.backBumperName);
        servoBumper.setPosition(BUMPER_DOWN);
        servoBucket = hardwareMap.servo.get(ConstantsConfig.bucketServoName);
        servoBucket.setPosition(bucketPos);

        //sensorOpticalDistance = hardwareMap.opticalDistanceSensor.get("0od");
        sensorGyro = hardwareMap.gyroSensor.get("5gyro");
        sensorGyro.calibrate();
        //turn the LED off then on in the beginning so user will know that the sensor is active.
        sensorColor = hardwareMap.colorSensor.get("3color");
        sensorColor.enableLed(false);
        sensorColor.enableLed(true);
    }

    @Override
    public void init_loop(){//loop while positioning robot, waiting for Start
        resetDriveEncoders();
        telemetry.addData("InitLoop: Ready if sensorGyro is zero here - ", sensorGyro.getHeading());
        //Give the gyro time to calibrate...
        while (sensorGyro.isCalibrating()) {
            telemetry.addData("NOT READY YET", sensorGyro.getHeading());
            timeInState.reset();
            while (timeInState.time() < 0.05){
                telemetry.addData("CALIBRATING GYRO", timeInState.time());
            }
        }
    }

    @Override
    public void start() {
        super.start();
        resetDriveEncoders();
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorArmUpDown.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorArmUpDown.setTargetPosition(0);
        motorArmUpDown.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTurretRotate.setTargetPosition(0);
        motorTurretRotate.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorHangWinch.setTargetPosition(0);
        motorHangWinch.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        gyroDrift = adjHeading(0);//if long wait before Start, gyro may have drift
        timeInRound.reset();
        changeToState(State.MOVE_TO_FAR_SIDE);
    }

    @Override
    public void loop() {
        telemetry.addData("Current State", currentState);
        switch (currentState){
            case MOVE_TO_FAR_SIDE:
                telemetry.addData("Distance (inches)", motorRight.getCurrentPosition() / ENCODER_CPI);
                telemetry.addData("Current Heading", sensorGyro.getHeading());
                if(moveDistanceWithHeading(0, 120 * ENCODER_CPI)){
                    changeToState(State.TURN_TO_WHITE_LINE);
                }
                break;
            case TURN_TO_WHITE_LINE:
                telemetry.addData("Heading",adjHeading(sensorGyro.getHeading()));
                if(turnToHeading(43)){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    changeToState(State.MOVE_TO_WHITE_LINE);
                }
                break;
            case MOVE_TO_WHITE_LINE:
                telemetry.addData("Color", sensorColor.alpha());
                if(sensorColor.alpha() == 1){
                    motorRight.setPower(0);
                    motorLeft.setPower(0);
                    changeToState(State.RAISE_ARM);
                }else{
                    motorLeft.setPower(.5);
                    motorRight.setPower(.5);
                }
                break;
            case RAISE_ARM:
                telemetry.addData("up down ENC", motorArmUpDown.getCurrentPosition());
                telemetry.addData("turn ENC", motorTurretRotate.getCurrentPosition());
                motorArmUpDown.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorTurretRotate.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                boolean readyToDrop = false;
                if(motorArmUpDown.getCurrentPosition() < 500){
                    motorArmUpDown.setPower(1);
                    readyToDrop = false;
                }else if(motorArmUpDown.getCurrentPosition() > 500) {
                    motorArmUpDown.setPower(-.2);
                    readyToDrop = false;
                }else{
                    motorArmUpDown.setPower(0);
                    readyToDrop = true;
                }
                if(motorTurretRotate.getCurrentPosition() < 3000){
                    motorTurretRotate.setPower(1);
                    readyToDrop = false;
                }else{
                    motorTurretRotate.setPower(0);
                    readyToDrop = true;
                }
                if(readyToDrop){
                    motorArmUpDown.setPower(-1);
                    motorTurretRotate.setPower(0);
                    servoBucket.setPosition(BUCKET_DUMP);
                    changeToState(State.DUMP_CLIMBERS);
                }
                break;
            case DUMP_CLIMBERS:
                motorArmUpDown.setPower(0);
                telemetry.addData("Up Down ENC", motorArmUpDown.getCurrentPosition());
                if(motorArmUpDown.getCurrentPosition() > 800) {
                    servoDebrisGate.setPosition(.4);
                    changeToState(State.LOWER_ARM);
                }
                break;
        }
    }
}
