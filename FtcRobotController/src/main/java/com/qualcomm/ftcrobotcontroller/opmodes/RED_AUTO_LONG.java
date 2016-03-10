package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by robow_000 on 3/9/2016.
 */
public class RED_AUTO_LONG extends OpMode{

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
    static final float HANGER_IN = 0.07f, HANGER_OUT = 0.9f;// Hanger Hook
    static final float RED_UP = 0.83f, RED_MID = 0.15f, RED_DOWN = 0.0f;// Red Pusher Zipline Trigger
    static final float BLUE_UP = 0.15f, BLUE_MID = 0.75f, BLUE_DOWN = .9f;// Blue Pusher Zipline Trigger
    static final float BUMPER_UP = .66f, BUMPER_DOWN = .04f;// Back Bumper
    static final float BUCKET_LOAD = 0.07f, BUCKET_DUMP = 0.63f;// Collector Bucket
    static final int PUSHER_DOWN = 0, PUSHER_HALF = 1, PUSHER_UP = 2;
    static final int PUSHER_READY = 0, PUSHER_NOT_READY = 1;

    public void resetDriveEncoders() {
        motorLeft.setTargetPosition(0);
        motorRight.setTargetPosition(0);
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void goToPos(DcMotor m, double encCount, double power) {
        telemetry.addData("Gyro:", sensorGyro.getHeading());
        m.setTargetPosition((int) encCount);
        m.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        m.setPower(power);
    }

    private int adjHeading(int offset){
        int heading;
        heading = sensorGyro.getHeading() - offset;
        if (heading > 180) {heading = heading - 360;}
        return heading;
    }

    public void turnToHeading(int desiredHeading, float desiredPower){
        //sets targetHeading, deltaHeading, leftPower, rightPower, but NOT basePower
        //int targetHeading = desiredHeading;
        //int deltaHeading = desiredHeading - adjHeading(sensorGyro.getHeading());
        int deltaHeading = 50;
        double leftPower;
        double rightPower;

        while (Math.abs(deltaHeading) > 5) {
            //deltaHeading = desiredHeading - adjHeading(sensorGyro.getHeading());
            deltaHeading = 50;
            //basePower = desiredPower;

            if (deltaHeading > 1) {//turn right
                leftPower = -desiredPower * .75f;
                rightPower = desiredPower * .75f;
            } else if (deltaHeading < -1) {//turn left
                leftPower = desiredPower * .75f;
                rightPower = -desiredPower * .75f;
            } else {//already at correct heading
                leftPower = 0;
                rightPower = 0;
            }//setting left & rightPower at 0.75 for slowing the turn as approach desiredHeading
            motorLeft.setPower(leftPower / 0.75f);
            motorRight.setPower(rightPower / 0.75f);//start turn with full desiredPower
        }
    }

    public double inchesToEncoderTicks(double inches){
        return inches * 120.689655;
    }

    @Override
    public void init() {
        telemetry.addData("0","...... Initializing .....");
        motorRoller = hardwareMap.dcMotor.get("qq1roll");
        motorHangWinch = hardwareMap.dcMotor.get("qq2winch");
        motorTurretRotate = hardwareMap.dcMotor.get("ue1turr");
        motorArmUpDown = hardwareMap.dcMotor.get("ue2updn");
        motorLeft = hardwareMap.dcMotor.get("mo1left");
        motorRight = hardwareMap.dcMotor.get("mo2right");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        servoDebrisGate = hardwareMap.servo.get("6gate");
        servoDebrisGate.setPosition(DGATE_HOLDCLIMBERS);
        servoHangerElbow = hardwareMap.servo.get("5hang");
        servoHangerElbow.setPosition(HANGER_IN);
        servoRedPusher = hardwareMap.servo.get("4red");
        servoRedPusher.setPosition(RED_UP);
        servoBluePusher = hardwareMap.servo.get("3blu");
        servoBluePusher.setPosition(BLUE_UP);
        servoBumper = hardwareMap.servo.get("2bump");
        servoBumper.setPosition(BUMPER_DOWN);
        servoBucket = hardwareMap.servo.get("1bkt");
        servoBucket.setPosition(.08);

        //sensorOpticalDistance = hardwareMap.opticalDistanceSensor.get("0od");
        sensorGyro = hardwareMap.gyroSensor.get("5gyro");
        sensorGyro.calibrate();
        //turn the LED off then on in the beginning so user will know that the sensor is active.
        sensorColor = hardwareMap.colorSensor.get("3color");
        sensorColor.enableLed(false);
        sensorColor.enableLed(true);
        resetDriveEncoders();
    }



    @Override
    public void loop() {
        motorLeft.setPower(1);
       // motorRight.setPower(-1);//start turn with full desiredPower
    }
}
