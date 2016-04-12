package com.qualcomm.ftcrobotcontroller.opmodes;

public class ConstantsConfig {
    static final float  DGATE_HOLDCLIMBERS = 0.229f;
    static final double DGATE_CLOSE = 0.227, DGATE_OPEN = .66;
    static final double HANGER_IN = 0.1, HANGER_OUT = 0.9;
    static final float RED_UP = 0.83f, RED_MID = 0.15f, RED_DOWN = 0.0f;// Red Pusher Zipline Trigger
    static final float BLUE_UP = 0.15f, BLUE_MID = 0.75f, BLUE_DOWN = .9f;// Blue Pusher Zipline Trigger
    static final float BUMPER_UP = .66f, BUMPER_DOWN = .04f;// Back Bumper
    static final float BUCKET_LOAD = 0.07f, BUCKET_DUMP = 0.63f;// Collector Bucket
    static final int PUSHER_DOWN = 0, PUSHER_HALF = 1, PUSHER_UP = 2;
    static final int PUSHER_READY = 0, PUSHER_NOT_READY = 1;
    static final float ENCODER_CPI = 77.07f;// ~121 counts/inch with Hugo's treads
    static final int FULL_DOWN = 0;
    static final int HALF_DOWN = 1;
    static final int UP = 2;
    static final int READY = 0;
    static final int NOT_READY = 1;
    static final String rollerMotorName = "qq1roll";
    static final String hangWinchName = "mo1winch";
    static final String turretRotationMotorName = "ue1turr";
    static final String turretUpDownName = "ue2updn";
    static final String motorLeftName = "qq2left";
    static final String motorRightName = "mo2right";
    static final String debrisGateName = "6gate";
    static final String hangerName = "5hang";
    static final String blueTriggerName = "3blu";
    static final String backBumperName = "2bump";
    static final String bucketServoName = "1bkt";
    static final String gyroName = "5gyro";
    static final String opticalDistanceName = "0od";
    static final String redTriggerName = "4red";
}
