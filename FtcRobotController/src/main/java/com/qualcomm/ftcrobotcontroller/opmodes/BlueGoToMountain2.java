package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by robow_000 on 3/22/2016.
 */
public class BlueGoToMountain2 extends OpMode{
    private enum State {
        DRIVE_TO_BASE,
        TURN_TO_MOUNTAIN,
        LOWER_ARM,
        GO_UP_MOUTAIN
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

    //*** Java assumes DOUBLE PRECISION, must specify FLOAT with #.##f
    static final int RED_IS_MINUS1 = -1;// +1 IF BLUE, for turns of robot and turret
    static final float  DGATE_HOLDCLIMBERS = ConstantsConfig.DGATE_HOLDCLIMBERS;
    static final double DGATE_CLOSE = ConstantsConfig.DGATE_CLOSE, DGATE_OPEN = ConstantsConfig.DGATE_OPEN;
    static final double HANGER_IN = ConstantsConfig.HANGER_IN, HANGER_OUT = ConstantsConfig.HANGER_OUT;
    static final float RED_UP = ConstantsConfig.RED_UP, RED_MID = ConstantsConfig.RED_MID, RED_DOWN = ConstantsConfig.RED_DOWN;// Red Pusher Zipline Trigger
    static final float BLUE_UP = ConstantsConfig.BLUE_UP, BLUE_MID = ConstantsConfig.BLUE_MID, BLUE_DOWN = ConstantsConfig.BLUE_DOWN;// Blue Pusher Zipline Trigger
    static final float BUMPER_UP = ConstantsConfig.BUMPER_UP, BUMPER_DOWN = ConstantsConfig.BUMPER_DOWN;// Back Bumper
    static final float BUCKET_LOAD = ConstantsConfig.BUCKET_LOAD, BUCKET_DUMP = ConstantsConfig.BUCKET_DUMP;// Collector Bucket
    static final int PUSHER_DOWN = ConstantsConfig.PUSHER_DOWN, PUSHER_HALF = ConstantsConfig.PUSHER_HALF, PUSHER_UP = ConstantsConfig.PUSHER_UP;
    static final int PUSHER_READY = ConstantsConfig.PUSHER_READY, PUSHER_NOT_READY = ConstantsConfig.PUSHER_NOT_READY;
    static final float ENCODER_CPI = ConstantsConfig.ENCODER_CPI;

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
                motorRight.setPower(.2);
            }
            if(adjHeading(sensorGyro.getHeading()) < desiredHeading){
                motorLeft.setPower(.2);
            }
            return false;
        }else{
            motorRight.setPower(0);
            motorLeft.setPower(0);
            return true;
        }
    }

    private Boolean turnToHeading(int heading){
        if(sensorGyro.getHeading() != heading){
            if(Math.abs(adjHeading(sensorGyro.getHeading()) - heading) > 40){
                motorLeft.setPower(-1);
                motorRight.setPower(1);
            }else{
                motorLeft.setPower(-.4);
                motorRight.setPower(.4);
            }
            return false;
        }
        return true;
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
        changeToState(State.DRIVE_TO_BASE);
    }

    @Override
    public void loop() {
        switch (currentState){
            case DRIVE_TO_BASE:
                telemetry.addData("Distace (inches)", motorRight.getCurrentPosition() / ENCODER_CPI);
                telemetry.addData("Current Heading", sensorGyro.getHeading());
                if(moveDistanceWithHeading(0, 52 * ENCODER_CPI)){
                    changeToState(State.TURN_TO_MOUNTAIN);
                }

                break;
            case TURN_TO_MOUNTAIN:
                telemetry.addData("Heading",adjHeading(sensorGyro.getHeading()));
                if(turnToHeading(85)){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    changeToState(State.GO_UP_MOUTAIN);
                }
                break;
            case GO_UP_MOUTAIN:
                servoBumper.setPosition(1);
                telemetry.addData("Distace (inches)", motorRight.getCurrentPosition() / ENCODER_CPI);
                if(motorRight.getCurrentPosition() < (120 * ENCODER_CPI)){
                    motorRight.setPower(1);
                    motorLeft.setPower(1);
                }else{
                    telemetry.addData("Done", "");
                    motorRight.setPower(0);
                    motorLeft.setPower(0);
                }
                break;
        }
    }
}
