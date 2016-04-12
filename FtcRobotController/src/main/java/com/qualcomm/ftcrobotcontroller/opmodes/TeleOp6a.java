package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.currentThreadTimeMillis;



/**************************************************
 *
 *   Robowties 6137
 *
 *   Teleop v6
 *
 **************************************************/

public class TeleOp6a extends OpMode {
    DcMotor rollerMotor;
    DcMotor hangWinch;
    DcMotor turretRotationMotor;
    DcMotor turretUpDown;
    DcMotor motorLeft;
    DcMotor motorRight;

    Servo debrisGate;
    Servo hanger;
    Servo redTrigger;
    Servo blueTrigger;
    Servo backBumper;
    Servo bucketServo;

    GyroSensor gyro;
    OpticalDistanceSensor opticalDistance;

    static final double DGATE_CLOSE = ConstantsConfig.DGATE_CLOSE, DGATE_OPEN = ConstantsConfig.DGATE_OPEN;
    static final double HANGER_IN = ConstantsConfig.HANGER_IN, HANGER_OUT = ConstantsConfig.HANGER_OUT;
    static final double RED_UP = ConstantsConfig.RED_UP, RED_MID = ConstantsConfig.RED_MID, RED_DOWN = ConstantsConfig.RED_DOWN;
    static final double BLUE_UP = ConstantsConfig.BLUE_UP, BLUE_MID = ConstantsConfig.BLUE_MID, BLUE_DOWN = ConstantsConfig.BLUE_DOWN;
    static final double BUMPER_UP = ConstantsConfig.BUMPER_UP, BUMPER_DOWN = ConstantsConfig.BUMPER_DOWN;
    static final double BUCKET_LOAD = ConstantsConfig.BUCKET_LOAD, BUCKET_DUMP = ConstantsConfig.BUCKET_DUMP; //change bucket_load to fix problem

    static final int FULL_DOWN = ConstantsConfig.FULL_DOWN;
    static final int HALF_DOWN = ConstantsConfig.HALF_DOWN;
    static final int UP = ConstantsConfig.UP;

    static final int READY = 0;
    static final int NOT_READY = 1;

    static final int RED = 0;
    static final int BLUE = 0;
 
    float percent = 70;
    float turretRotato = 0;
    float turretRotatoMiddle = 0;
    float turretUPDOWN = 0;
    double bucketPos=.07;
    double bucketSpeed = .006;
    double hangerPosition = 0.1;
    double winchPower = 0;
    boolean backBumperDown = true;
    boolean turretgodown = false;
    int toggleTimer = 0;
    float rollerSpeed;

    int trigger = READY;
    int triggerMode = UP;

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void init() {
        rollerMotor = hardwareMap.dcMotor.get(ConstantsConfig.rollerMotorName);
        hangWinch = hardwareMap.dcMotor.get(ConstantsConfig.hangWinchName);
        turretRotationMotor = hardwareMap.dcMotor.get(ConstantsConfig.turretRotationMotorName);
        turretUpDown = hardwareMap.dcMotor.get(ConstantsConfig.turretUpDownName);
        motorLeft = hardwareMap.dcMotor.get(ConstantsConfig.motorLeftName);
        motorRight = hardwareMap.dcMotor.get(ConstantsConfig.motorRightName);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        debrisGate = hardwareMap.servo.get(ConstantsConfig.debrisGateName);
        hanger = hardwareMap.servo.get(ConstantsConfig.hangerName);
        redTrigger = hardwareMap.servo.get(ConstantsConfig.redTriggerName);
        blueTrigger = hardwareMap.servo.get(ConstantsConfig.blueTriggerName);
        backBumper = hardwareMap.servo.get(ConstantsConfig.backBumperName);
        bucketServo = hardwareMap.servo.get(ConstantsConfig.bucketServoName);

        gyro = (GyroSensor) hardwareMap.gyroSensor.get(ConstantsConfig.gyroName);
        opticalDistance = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get(ConstantsConfig.opticalDistanceName);
        bucketServo.setPosition(0);
    }

    @Override
    public void loop() {
        float left;
        float right;

        if (gamepad1.right_bumper) {
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
        } else {
            left = gamepad1.left_stick_y * (percent / 100);
            right = gamepad1.right_stick_y * (percent / 100);
        }

        //moving red zipline trigger
        if (gamepad1.left_trigger > 0 && gamepad1.left_trigger < .5) {
            redTrigger.setPosition(RED_MID);
        } else if (gamepad1.left_trigger >= .5) {
            redTrigger.setPosition(RED_DOWN);
        } else redTrigger.setPosition(RED_UP);

        //moving blue zipline trigger (wouldn't work when combined with red)
        if (gamepad1.left_trigger >= .9) {
            triggerMode = FULL_DOWN;
            trigger = NOT_READY;
        } else if (gamepad1.left_trigger > .45 && gamepad1.left_trigger < .9) {
            if (trigger == READY || triggerMode == UP) {
                triggerMode = HALF_DOWN;
                trigger = NOT_READY;
            }
        } else if (gamepad1.left_trigger <= .45 && gamepad1.left_trigger > .01) {
            if (trigger == READY) {
                triggerMode = UP;
                trigger = NOT_READY;
            }
        } else trigger = READY;

        if(triggerMode == UP){
            redTrigger.setPosition(RED_UP);
            blueTrigger.setPosition(BLUE_UP);
            telemetry.addData("UP", triggerMode);
        }
        else if(triggerMode == HALF_DOWN){
            redTrigger.setPosition(RED_MID);
            blueTrigger.setPosition(BLUE_MID);
            telemetry.addData("HALF", triggerMode);

        }
        else if(triggerMode == FULL_DOWN){
            redTrigger.setPosition(RED_DOWN);
            blueTrigger.setPosition(BLUE_DOWN);
            telemetry.addData("DOWN", triggerMode);

        }
        //intake or output of foam roller
        if (gamepad2.dpad_right) {
            rollerSpeed = .7f;
        } else if (gamepad2.dpad_left) {
            rollerSpeed = -.7f;
        } else {
            rollerSpeed -= .2 * (rollerSpeed); //delays the stop of the roller stopping
        }

        //open and close debris gate
        if (gamepad2.right_bumper) {
            debrisGate.setPosition(DGATE_OPEN);
        } else debrisGate.setPosition(DGATE_CLOSE);

        telemetry.addData("bucket pos", bucketPos);
        bucketPos = Range.clip(bucketPos, BUCKET_LOAD, BUCKET_DUMP);
        bucketServo.setPosition(bucketPos);

        //back bumper up or down
        if (gamepad1.left_bumper && toggleTimer == 0) {
            backBumperDown = !backBumperDown;
            toggleTimer = 30;
        }

        if (toggleTimer > 0)
            toggleTimer -= 1;

        if (!backBumperDown) {
            backBumper.setPosition(BUMPER_UP);
        } else {
            backBumper.setPosition(BUMPER_DOWN);
        }

        //moving hang arm up and down
        if (gamepad2.dpad_up) {
            hangerPosition += 0.01;
        } else if (gamepad2.dpad_down) {
            hangerPosition -= 0.01;
        }

        //move hang arm to full out position
        if (gamepad2.left_bumper) {
            double t = currentThreadTimeMillis();
            hangerPosition = HANGER_OUT;
            if(currentThreadTimeMillis() > t + 500){
                triggerMode = HALF_DOWN;
            }
        }

        telemetry.addData("hangerPosition", hangerPosition);
        hangerPosition = Range.clip(hangerPosition, HANGER_IN, HANGER_OUT);
        hanger.setPosition(hangerPosition);

        //reel in hanging string or let out
        if (gamepad1.right_trigger > .5) {
            winchPower = 1;
        } else if (gamepad1.y) {
            winchPower = -2;
        } else {
            winchPower = 0;
        }

        //turret rotation control
        if(gamepad2.left_trigger > .9){
            turretRotato = 0;
        }
        turretRotato -= gamepad2.left_stick_x * 20 * pctFromTarget(turretRotationMotor.getCurrentPosition()); //changed from 30
        telemetry.addData("ARM UP DOWN", gamepad2.right_stick_y);
        int targetDistred = 100;
        if(gamepad2.b){ //red pos
            goToPos(turretUpDown, targetDistred, 1);
            if(turretUpDown.getCurrentPosition() > targetDistred * .8) {
                turretRotato = turretRotatoMiddle - 4850;
            }
        }

        int targetDistblue = 100;
        if(gamepad2.x){ //go to blue pos
            goToPos(turretUpDown, targetDistblue, 1);
            if(turretUpDown.getCurrentPosition() > targetDistred * .8) {
                turretRotato = turretRotatoMiddle + 4850;
            }
        }

        telemetry.addData("ARM MOTOR", turretUpDown.getCurrentPosition());

        if (Math.abs(gamepad2.right_stick_y) > .1) {
            goDistance(turretUpDown, -100 * gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y), 1);
        } else {

            if (turretUpDown.getCurrentPosition() > -100) { //deadzone for turret up/down
                goDistance(turretUpDown, -100 * gamepad2.right_stick_y * Math.abs(gamepad2.right_stick_y), .5);
            } else{
                if(Math.abs(turretRotato) < 250) {
                    turretUpDown.setPower(0);
                }
            }

        }

        //when moving bucket to score, also moves foam roller
        if (gamepad2.y) {
            if (bucketPos < .995) { //.995 is the upper limit
                bucketPos += bucketSpeed;
            }
            //moving bucket to loading position
        } else if (gamepad2.a) {
            if (bucketPos > .005) { //.005 is the lower limit
                rollerSpeed = -0.3f;
                bucketPos -= bucketSpeed;
            }
        }

        telemetry.addData("TurretPosition", turretRotato);
        telemetry.addData("power pct: ", pctFromTarget(turretRotationMotor.getCurrentPosition()));

        //push values to motors
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        winchPower = Range.clip(winchPower, -1, 1);
        //turretRotato = Range.clip(turretRotato, -15000, 15000);
        left = (float) scaleInput(left);
        right = (float) scaleInput(right);
        motorRight.setPower(right);
        motorLeft.setPower(left);
        hangWinch.setPower(winchPower);
        goToPos(turretRotationMotor, turretRotato, 1);
        rollerMotor.setPower(rollerSpeed);
    }

    private double pctFromTarget(int heading){
//        int targetColor;
//        if((Math.abs(heading - 4850)) > Math.abs(heading + 4850)){
//            targetColor = BLUE;
//        }
//        else targetColor = RED;
//
//        if(targetColor == BLUE){
//
//        }
        double h;
        if(heading != 0){
            h = Math.abs(heading);
        }
        else h = 1;
        double pct = .05 + Math.abs((((4850 - h)) / 4850)); // 1 = 100% difference
        telemetry.addData("raw pct: ", pct);
        pct += .2;
        if(pct > 1){
            pct = 1;
        }
        return pct;
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }

    public void goDistance(DcMotor m, double encCount, double speed) {
        m.setTargetPosition((int) (encCount + m.getCurrentPosition()));
        m.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        m.setPower(speed);
    }

    public void goToPos(DcMotor m, double encCount, double speed) {
        m.setTargetPosition((int) encCount);
        m.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        m.setPower(speed);
    }
}

