package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*** Autonomous OpMode    */

public class Blue7a_LONG extends OpMode {

    private enum State {
        AT_START_GO_FAR,
        AT_FAR_GO_BACKUP,
        AT_BACKUP_GO_TURN_PARALLEL,
        AT_TURN_PARALLEL_GO_ARM_VERTICAL,
        AT_ARM_VERTICAL_GO_TURRET90,
        AT_TURRET90_GO_WHITE_TAPE,
        AT_WHITE_TAPE_GO_ADJ_PARALLEL,
        AT_ADJ_PARALLEL_GO_ARM_ON_BEACON,
        AT_ARM_ON_BEACON_GO_OPEN_GATE,
        AT_OPEN_GATE_GO_ARM_VERTICAL,
        AT_ARM_VERTICAL_GO_TURRET0,
        AT_TURRET0_GO_LOWER_ARM,
        AT_LOWER_ARM_GO_TURN_2MTN,
        AT_TURN_2MTN_GO_HALT_TURN,
        AT_HALT_TURN_GO_CLEAR_MTNBASE,
        AT_CLEAR_MTNBASE_GO_BACKUP,
        AT_BACKUP_GO_FACE_MTN,
        AT_FACE_MTN_GO_PREP_CLIMB,
        AT_PREP_CLIMB_GO_CLIMBING,
        AT_CLIMBING_GO_END_STATE,
        AT_END_STATE,
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
    OpticalDistanceSensor sensorOpticalDistance;

    static final int RED_IS_MINUS1 = 1;// +1 IF BLUE, for turns of robot and turret
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
    int oldHeading, targetHeading, deltaHeading, gyroDrift, wiggleFlag;
    float currentPower, basePower = 0.5f, deltaPower, deltaPowerMax = 1 - basePower;
    float rightPower, leftPower;

    float leftStickY, rightStickY, normalDrive = 0.7f;//slower motors for normal driving
    boolean bumperDown = true;
    int targetDistRed = 400, targetDistBlue = 400;
    int pusherStatus = PUSHER_READY, pusherMode = PUSHER_UP;
    float winchPower = 0;
    float turretRotato = 0, bucketPos=.08f, bucketPower = .006f, rollerPower, hangerPosition = .1f;
    int toggleTimer = 0;
    int surfaceColor;

    public ElapsedTime timeInRound = new ElapsedTime();//clock for whole autonomous round
    public ElapsedTime timeInState = new ElapsedTime();//clock reset for each state
    public ElapsedTime timeInPause = new ElapsedTime();//clock reset for pauses
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

//    public void goDistance(DcMotor m, double encCount, double power) {
//        m.setTargetPosition((int) (encCount + m.getCurrentPosition()));
//        m.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//        m.setPower(power);
//    }

    public void goToPos(DcMotor m, double encCount, double power) {
        m.setTargetPosition((int) encCount);
        m.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        m.setPower(power);
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

    @Override
    public void stop() {
    }

    @Override
    public void init() {
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
                telemetry.addData("calibrating gyro", timeInState.time());
            }
        }
    }

    @Override
    public void start() {
        super.start();
        resetDriveEncoders();
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorArmUpDown.setTargetPosition(0);
        motorArmUpDown.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorTurretRotate.setTargetPosition(0);
        motorTurretRotate.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorHangWinch.setTargetPosition(0);
        motorHangWinch.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //currentHeading = 0;
        gyroDrift = adjHeading(0);//if long wait before Start, gyro may have drift
        timeInRound.reset();
        changeToState(State.AT_START_GO_FAR);
    }

    @Override
    public void loop()
    {
        telemetry.addData("t",String.format("%4.1f  %4.1f >",
                timeInRound.time(),timeInState.time())+currentState.toString());
//        telemetry.addData("0",String.format("CASE currentState   state %4.1f   TIME %4.1f",
//                currentState, timeInState.time(), timeInRound.time()) );
        switch (currentState)
        {
            case AT_START_GO_FAR://from start, set target distance and heading, turn on motors
                telemetry.addData("Distace (inches)", motorRight.getCurrentPosition() / ENCODER_CPI);
                telemetry.addData("Current Heading", sensorGyro.getHeading());
                if(moveDistanceWithHeading(0, 80 * ENCODER_CPI)){
                    changeToState(State.AT_FAR_GO_BACKUP);
                }
                break;

            case AT_FAR_GO_BACKUP://correct heading with gyro, then backup an inch so can freely turn
                telemetry.addData("Current Heading", adjHeading(gyroDrift));
                if (motorLeft.getCurrentPosition() < targetEncCount &&
                        motorRight.getCurrentPosition() < targetEncCount &&
                        timeInState.time() < maxTime ) {//if by time, prob hit wall, then go State 2
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    deltaPower = deltaHeading * .15f;
                    // going straight, deltaHeading hopefully not > 2 degrees, clip just in case
                    deltaPower = Range.clip(deltaPower, -deltaPowerMax, deltaPowerMax);
                    motorLeft.setPower(basePower - deltaPower);
                    motorRight.setPower(basePower + deltaPower);
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
                    targetInches = -1;//so can freely turn
                    targetEncCount = targetInches * ENCODER_CPI + motorLeft.getCurrentPosition();
                    //targetHeading = 0;// needs to be same as in previous case
                    basePower = -0.6f;
                    if (basePower > 0) {
                        deltaPowerMax = 1 - basePower;
                    } else {
                        deltaPowerMax = -1 - basePower;
                    }
                    motorLeft.setPower(basePower); motorRight.setPower(basePower);
                    minTime =0f; maxTime = .6f;
                    changeToState(State.AT_BACKUP_GO_TURN_PARALLEL);
                }
                break;

            case AT_BACKUP_GO_TURN_PARALLEL://after back up, set up for turn parallel to wall and Rescue Beacon
                if (motorLeft.getCurrentPosition() > targetEncCount &&
                        timeInState.time() < maxTime ) {//if by time, *****
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    deltaPower = deltaHeading * .15f; // going straight, deltaHeading hopefully not > 2 degrees
                    deltaPower = Range.clip(deltaPower, -deltaPowerMax, deltaPowerMax);// clip just in case
                    motorLeft.setPower(basePower - deltaPower);
                    motorRight.setPower(basePower + deltaPower);
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
                    if(turnToHeading(125 * RED_IS_MINUS1)) { //135
                        minTime = 0f;
                        maxTime = 3f;
                        changeToState(State.AT_TURN_PARALLEL_GO_ARM_VERTICAL);
                    }
                }
                break;

            case AT_TURN_PARALLEL_GO_ARM_VERTICAL://turn until reach target heading, then prep arm
                if (Math.abs(deltaHeading) > 2){
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    telemetry.addData("Current Heading", adjHeading(gyroDrift));
                    telemetry.addData("delta", deltaHeading);
                    telemetry.addData("gyroDrift", gyroDrift);
                    telemetry.addData("power", motorLeft.getPower());
                    if (Math.abs(deltaHeading) < 25 ){//reduce turning power
                        motorLeft.setPower(leftPower);//left & rightPower from previous case
                        motorRight.setPower(rightPower);
                    }
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);

                    servoHangerElbow.setPosition(HANGER_IN);// hold in place so doesn't get loose
                    targetEncCount = 450;//raise arm vertical 400, slowly so climbers don't fall
                    targetFuzzy = .05f * targetEncCount;
                    goToPos(motorArmUpDown, targetEncCount, .4);
                    bucketPos = BUCKET_DUMP;              //turn bucket to up position BUCKET_DUMP
                    servoBucket.setPosition(bucketPos);
                    minTime =.1f; maxTime = 2f;
                    changeToState(State.AT_ARM_VERTICAL_GO_TURRET90);
                }
                break;

            case AT_ARM_VERTICAL_GO_TURRET90://when arm and bucket raised vertical, rotate turret
                telemetry.addData("Arm target", targetEncCount);
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                if (timeInState.time() > minTime){//wait
                    if (motorArmUpDown.getCurrentPosition() < targetEncCount - targetFuzzy){
                        //
                    } else {
                        targetEncCount = 2600 * RED_IS_MINUS1;//-3000
                        goToPos(motorTurretRotate, targetEncCount, .6);//turn arm to face beacon, slowly
                        minTime =1f; maxTime = 3f;
                        changeToState(State.AT_TURRET90_GO_WHITE_TAPE);
                    }
                }
                break;

            case AT_TURRET90_GO_WHITE_TAPE://when turret rotated to Rescue Beacon,
                telemetry.addData("turret ENC", motorTurretRotate.getCurrentPosition());
                telemetry.addData("turret target", targetEncCount);
                if (Math.abs(motorTurretRotate.getCurrentPosition() - targetEncCount) > 50 ) {
                    //wait for 1 sec  &&
                    //timeInState.time() > minTime
                } else {
                    surfaceColor = sensorColor.alpha();
                    oldEncCount = motorLeft.getCurrentPosition();//track forward progress
                    wiggleFlag = 0;//if stopped by ziptie, then track wiggle process
                    basePower = 0.45f;
                    motorLeft.setPower(basePower);
                    motorRight.setPower(basePower);
                    minTime =0.5f; maxTime = 2f;//~1.8 sec to hit white tape
                    changeToState(State.AT_WHITE_TAPE_GO_ADJ_PARALLEL);
                }
                break;

            case AT_WHITE_TAPE_GO_ADJ_PARALLEL://update color sensor reading until stop at white tape, then adjust turn a bit
                if (surfaceColor == 0 && timeInState.time() < maxTime){//black = 0 (not white = 1)
                    surfaceColor = sensorColor.alpha();
                    telemetry.addData("color: ", surfaceColor);
                    // how to handle getting stuck at ziptie
                    if (timeInState.time() > minTime) {
                        telemetry.addData("wiggle", wiggleFlag);
                        switch (wiggleFlag){
                            case 0://looking for no forward movement by Encoder
                                if (motorLeft.getCurrentPosition() - oldEncCount < 200) {
                                    wiggleFlag = 1;
                                    servoRedPusher.setPosition(RED_MID);
                                }
                                break;
                            case 1://start turn away from wall and ziptie-blocked debris
                                oldHeading = targetHeading;
    //                            startTurnToHeading(oldHeading + 15 * RED_IS_MINUS1, 0.5f);
                                wiggleFlag = wiggleFlag + 1;
                                break;
                            case 2://stop turn at target
                                if (Math.abs(deltaHeading) > 2){
                                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                                    if (Math.abs(deltaHeading) < 7){
                                        motorLeft.setPower(leftPower);
                                        motorRight.setPower(rightPower);
                                    }
                                } else {
                                    motorLeft.setPower(0);
                                    motorRight.setPower(0);
        //                            startTurnToHeading(oldHeading, 0.5f);
                                    wiggleFlag = wiggleFlag + 1;
                                }
                                break;
                            case 3://stop turn at original heading
                                if (Math.abs(deltaHeading) > 2){
                                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                                    if (Math.abs(deltaHeading) < 7){
                                        motorLeft.setPower(leftPower);
                                        motorRight.setPower(rightPower);
                                    }
                                } else {
                                    motorLeft.setPower(0);
                                    motorRight.setPower(0);
                                    wiggleFlag = wiggleFlag + 1;
                                }
                                break;
                            case 4://continue on our way forward, scanning for white line
                                surfaceColor = sensorColor.alpha();
                                servoRedPusher.setPosition(RED_UP);
                                motorLeft.setPower(basePower);//was 0.45f
                                motorRight.setPower(basePower);
                                wiggleFlag = wiggleFlag + 1;
                                break;
                            case 5:
                                break;
                        }
                    }

                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
                    //startTurnToHeading(135 * RED_IS_MINUS1, 0.5f);
                    minTime =0f; maxTime = 9.5f;
                    changeToState(State.AT_ADJ_PARALLEL_GO_ARM_ON_BEACON);
                }
                break;

            case AT_ADJ_PARALLEL_GO_ARM_ON_BEACON://when adjust parallel to Beacon, then raise arm
                if (Math.abs(deltaHeading) > 2){
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    telemetry.addData("Current Heading", adjHeading(gyroDrift));
                    telemetry.addData("delta", deltaHeading);
                    telemetry.addData("gyroDrift", gyroDrift);
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);

                    targetEncCount = 800;//absolute Encoder Count 800
                    goToPos(motorArmUpDown, targetEncCount, .2);
                    //.2 further rotate arm to rest on beacon
                    minTime =1f; maxTime = 5f;
                    changeToState(State.AT_ARM_ON_BEACON_GO_OPEN_GATE);
                }
                break;

            case AT_ARM_ON_BEACON_GO_OPEN_GATE://when arm resting onto Beacon, open Debris Gate to drop climbers
                telemetry.addData("arm ENC, open gate", motorArmUpDown.getCurrentPosition());
                telemetry.addData("arm target", targetEncCount);

                if (timeInState.time() > minTime) {
                    if (motorArmUpDown.getCurrentPosition() < targetEncCount )
                    {//wait for 1 sec,
                    } else {
                        servoDebrisGate.setPosition(DGATE_OPEN);//dump climbers
                        minTime =.4f; maxTime = 5f;
                        changeToState(State.AT_OPEN_GATE_GO_ARM_VERTICAL);
                    }
                }

                break;

            case AT_OPEN_GATE_GO_ARM_VERTICAL://close Debris Gate, then raise arm off the Rescue Beacon
                telemetry.addData("closegate, ARM ENC", motorArmUpDown.getCurrentPosition());
                telemetry.addData("arm target", targetEncCount);
                if (timeInState.time() > minTime){//wait for Climbers to drop
                    servoDebrisGate.setPosition(DGATE_CLOSE);//close gate
                    targetEncCount = 450;
                    targetFuzzy = .05f * targetEncCount;
                    goToPos(motorArmUpDown, targetEncCount, .3);
                    minTime =0f; maxTime = 5f;
                    changeToState(State.AT_ARM_VERTICAL_GO_TURRET0);
                }
                break;

            case AT_ARM_VERTICAL_GO_TURRET0://when arm off the Rescue Beacon, rotate Turret back to home position
                telemetry.addData("arm enc", motorArmUpDown.getCurrentPosition());
                telemetry.addData("arm target", targetEncCount + targetFuzzy);
                //if (timeInState.time() > 0.2){
                if (motorArmUpDown.getCurrentPosition() > targetEncCount + targetFuzzy){
                    //
                } else {
                    targetEncCount = 0;
                    goToPos(motorTurretRotate, targetEncCount, .6);
                    servoBucket.setPosition(BUCKET_LOAD);
                    minTime =0f; maxTime = 5f;
                    changeToState(State.AT_TURRET0_GO_LOWER_ARM);
                }
                //}
                break;

            case AT_TURRET0_GO_LOWER_ARM://when Turret back to rest, then reset arm and bucket
                telemetry.addData("turret ENC", motorTurretRotate.getCurrentPosition());
                telemetry.addData("turret Power", motorTurretRotate.getPower());
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                if (Math.abs(motorTurretRotate.getCurrentPosition() - targetEncCount) > 100 &&
                        timeInState.time()>minTime && timeInState.time()<maxTime){
                    if (Math.abs(motorTurretRotate.getCurrentPosition() - targetEncCount) < 600){
                        motorTurretRotate.setPower(.4);//slow down closer to target, was .3
                    };
                } else {
                    targetEncCount = -200;
                    //targetFuzzy = .05f * targetEncCount;
                    goToPos(motorArmUpDown, targetEncCount, .4);
                    //arm back down for loading
                    minTime =.5f; maxTime = 2f;
                    changeToState(State.AT_LOWER_ARM_GO_TURN_2MTN);
                }
                break;

            case AT_LOWER_ARM_GO_TURN_2MTN://when arm down, turn 45 deg to base of mtn
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                if (timeInState.time() > minTime) {
                    if (motorArmUpDown.getCurrentPosition() > targetEncCount) {
                    } else {
     //                   startTurnToHeading(155 * RED_IS_MINUS1, 0.8f);
                        minTime = 0f; maxTime = 2.f;
                        changeToState(State.AT_TURN_2MTN_GO_HALT_TURN);
                    }
                }
                break;

            case AT_TURN_2MTN_GO_HALT_TURN://when done with turn, stop motors
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                telemetry.addData("Heading", adjHeading(gyroDrift));
                telemetry.addData("delta", deltaHeading);
                if (Math.abs(deltaHeading) > 2 && timeInState.time() < maxTime) {
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    if (Math.abs(deltaHeading) < 16) {//reduce power when close
                        motorLeft.setPower(leftPower); motorRight.setPower(rightPower);
                    }
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
                    minTime = 0f; maxTime = 4f;
                    changeToState(State.AT_HALT_TURN_GO_CLEAR_MTNBASE);
                }
                break;

            case AT_HALT_TURN_GO_CLEAR_MTNBASE://go forward to base of mtn
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                if (timeInState.time() > minTime){
                    targetInches = 48;
                    targetEncCount = targetInches * ENCODER_CPI + motorLeft.getCurrentPosition();
                    //targetHeading = -160;// needs to be same as in previous case
                    basePower = 0.85f; deltaPowerMax = 1 - basePower;
                    motorLeft.setPower(basePower); motorRight.setPower(basePower);
                    minTime =0f; maxTime = 4f;
                    changeToState(State.AT_CLEAR_MTNBASE_GO_BACKUP);
                }
                break;

            case AT_CLEAR_MTNBASE_GO_BACKUP://when clear at base, back up
                telemetry.addData("Current Heading", adjHeading(gyroDrift));
                telemetry.addData("delta", deltaHeading);
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                if (motorLeft.getCurrentPosition() < targetEncCount &&
                        timeInState.time() < maxTime ) {//if by time, *****
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    deltaPower = deltaHeading * .15f; // going straight, deltaHeading hopefully not > 2 degrees
                    deltaPower = Range.clip(deltaPower, -deltaPowerMax, deltaPowerMax);// clip just in case
                    motorLeft.setPower(basePower - deltaPower);
                    motorRight.setPower(basePower + deltaPower);
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
                    targetInches = -4;
                    targetEncCount = targetInches * ENCODER_CPI + motorLeft.getCurrentPosition();
                    //targetHeading = -160;// needs to be same as in previous case
                    basePower = -0.6f;
                    if (basePower > 0) {
                        deltaPowerMax = 1 - basePower;
                    } else {
                        deltaPowerMax = -1 - basePower;
                    }
                    motorLeft.setPower(basePower); motorRight.setPower(basePower);
                    minTime =0f; maxTime = 2f;
                    changeToState(State.AT_BACKUP_GO_FACE_MTN);
                }
                break;

            case AT_BACKUP_GO_FACE_MTN://after back up, turn to mountain
                telemetry.addData("ENC-target", targetEncCount);
                telemetry.addData("motorLeft", motorLeft.getCurrentPosition());
                telemetry.addData("motorRight", motorRight.getCurrentPosition());
                if (motorLeft.getCurrentPosition() > targetEncCount &&
                        //motorRight.getCurrentPosition() < targetEncCount &&
                        timeInState.time() < maxTime ) {//if by time, *****
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    deltaPower = deltaHeading * .15f; // going straight, deltaHeading hopefully not > 2 degrees
                    deltaPower = Range.clip(deltaPower, -deltaPowerMax, deltaPowerMax);// clip just in case
                    motorLeft.setPower(basePower - deltaPower);
                    motorRight.setPower(basePower + deltaPower);
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
     //              startTurnToHeading(90 * RED_IS_MINUS1, 0.7f);
                    motorLeft.setPower(leftPower/.75f);//start turn with full basePower
                    motorRight.setPower(rightPower/.75f);
                    minTime =0f; maxTime = 3f;
                    changeToState(State.AT_FACE_MTN_GO_PREP_CLIMB);
                }
                break;

            case AT_FACE_MTN_GO_PREP_CLIMB://when done turning, prep for climb
                telemetry.addData("15 Current Heading", adjHeading(gyroDrift));
                telemetry.addData("delta", deltaHeading);
                telemetry.addData("gyrodrift", gyroDrift);
                if (Math.abs(deltaHeading) > 2 &&
                        timeInState.time() > minTime && timeInState.time() < maxTime){
                    deltaHeading = targetHeading - adjHeading(gyroDrift);
                    if (Math.abs(deltaHeading) < 25) {//reduce power when close
                        motorLeft.setPower(leftPower); motorRight.setPower(rightPower);
                    }
                } else {
                    motorLeft.setPower(0); motorRight.setPower(0);
                    //get ready for climbing ramp
                    servoBumper.setPosition(BUMPER_UP); servoBucket.setPosition(BUCKET_LOAD);
                    servoRedPusher.setPosition(RED_UP); servoBluePusher.setPosition(BLUE_UP);
                    //goToPos(motorArmUpDown, 100, .4);
                    minTime =0f; maxTime = 2f;
                    changeToState(State.AT_PREP_CLIMB_GO_CLIMBING);
                }
                break;

            case AT_PREP_CLIMB_GO_CLIMBING://when stopped after turn and ready for climb, go up!
                telemetry.addData("arm ENC", motorArmUpDown.getCurrentPosition());
                //if (timeInState.time() > minTime){
                targetInches = 80;
                targetEncCount = targetInches * ENCODER_CPI + motorRight.getCurrentPosition();
                basePower =1f; deltaPowerMax = 1 - basePower;
                motorLeft.setPower(basePower); motorRight.setPower(basePower);
//                    goToPos(motorArmUpDown, 300, .4);
                minTime =.25f; maxTime = 9.5f;
                changeToState(State.AT_CLIMBING_GO_END_STATE);
                //}
                break;

            case AT_CLIMBING_GO_END_STATE://climb up trying to get to targetInches 60 til end of autonomous
                telemetry.addData("17 X", sensorGyro.rawX());
                telemetry.addData("17 Y", sensorGyro.rawY());
                telemetry.addData("17 Z", sensorGyro.rawZ());
                if (timeInState.time() > minTime){
                    if (motorRight.getCurrentPosition() < targetEncCount)
                    {
                        deltaHeading = targetHeading - adjHeading(gyroDrift);
                        deltaPower = deltaHeading * .15f;
                        // going straight, deltaHeading hopefully not > 2 degrees, clip just in case
                        deltaPower = Range.clip(deltaPower, -deltaPowerMax, deltaPowerMax);
                        motorLeft.setPower(basePower - deltaPower);
                        motorRight.setPower(basePower + deltaPower);
//                        if (timeInState.time() > 2){
//                            //goToPose(motorArmUpDown, 500, .4);
//                            servoBucket.setPosition(BUCKET_LOAD);
//                        }
                        if (Math.abs(sensorGyro.rawX()) > 1000){//if going up slope, try zip pushers
                            servoBluePusher.setPosition(BLUE_DOWN);
                            servoRedPusher.setPosition(RED_DOWN);
                        }
                    } else {
                        motorLeft.setPower(0); motorRight.setPower(0);
                        servoBluePusher.setPosition(BLUE_UP); servoRedPusher.setPosition(RED_UP);
                        minTime =0f; maxTime = 9.5f;
                        changeToState(State.AT_END_STATE);
                    }
                }
                break;

            case AT_END_STATE://
//                if (timeInState.time() > minTime && timeInState.time() < maxTime){
//                    //
//                } else {
//                    //
//                    minTime =0f; maxTime = 9.5f;
//                    //changeToState(currentState+1);
//                }
                break;
        }
    }
}
