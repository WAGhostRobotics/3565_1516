package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 7876 on 12/9/2015.
 *
 * Authors: Lili,
 *
 * This class has all the Standard Drive methods.
 * ALL AUTONOMOUS OP MODES SHOULD EXTEND THIS CLASS.
 *
 * Versions: (ADD AS YOU EDIT)
 * (Big changes.small changes)
 * 1.0: Added first versions of methods, untested. Completed 12/17/15. Contributors: Lili
 * 1.1  Redid DriveInches to deal with read/write problem. Doesn't work since we can't read the encoders.
 */
public abstract class StandardDrive extends LinearOpMode{
    //Declare motors and servos
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;
    protected DcMotorController motorController;
    protected Servo buttonLeft;
    protected Servo buttonRight;
    protected Servo dump;
    protected Servo grab;

    //Declare constants
    private final double CIRCUMFERENCE = 4.75 * Math.PI; //Circumference of robot wheels
    private final double ROBOT_RADIUS = 15 / Math.PI; //Radius of robot (?)
    private final boolean LEFT = true; //Direction

    public abstract void runOpMode();

    public void begin() {
        //Initialize motors and servos
        leftMotor = hardwareMap.dcMotor.get("motor_1");
        rightMotor = hardwareMap.dcMotor.get("motor_2");
        motorController = hardwareMap.dcMotorController.get("wheels");
        buttonLeft = hardwareMap.servo.get("buttonl"); // channel 3
        buttonRight = hardwareMap.servo.get("buttonr"); // channel 2

       // grab = hardwareMap.servo.get("servo_4"); //channel 4
        dump = hardwareMap.servo.get("dump"); // channel 1
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    /**
     * Drive a certain number of inches
     * @param inches: number of inches to drive
     * @param l: left power
     * @param r: right power
     */
    public void driveInches(double inches, int l, int r){
        if(inches < 0){ //Reverse direction
            inches *= -1;
            l *= -1;
            r *= -1;
        }

        //Get encoder value
        double encoderValue = inchesToEncoder(inches);
        telemetry.addData("Encoder: ", encoderValue);
        int state = 0;
        int end = 0;
        int currentPostion = 0;
        int count = 0;

        /*leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftMotor.setTargetPosition((int) encoderValue);
        leftMotor.setPower(r);*/

        while(end == 0){
            switch(state) {
                case 0:
                    count++;
                    while (motorController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY)
                        motorController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                    telemetry.clearData();
                    telemetry.addData("Device Mode (0): ", motorController.getMotorControllerDeviceMode());
                    telemetry.addData("Current Encoder (0): ", leftMotor.getCurrentPosition());
                    telemetry.addData("Count (0): ",count);
                    currentPostion = leftMotor.getCurrentPosition();
                    state = 1;
                    break;
                case 1:
                    count++;
                    while (motorController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY)
                        motorController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    telemetry.clearData();
                    telemetry.addData("Device Mode (1): ", motorController.getMotorControllerDeviceMode());
                    telemetry.addData("Count (1): ",count);
                    if (currentPostion < encoderValue) {
                        //rightMotor.setPower(r);
                        leftMotor.setPower(l);
                        state = 0;
                    } else {
                        state = 2;
                    }
                    break;
                case 2:
                    count++;
                    while (motorController.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY)
                        motorController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                    telemetry.clearData();
                    telemetry.addData("Device Mode (2): ", motorController.getMotorControllerDeviceMode());
                    telemetry.addData("Count (2): ",count);
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                    /*while (leftMotor.getMode() != DcMotorController.RunMode.RESET_ENCODERS)
                        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    while (leftMotor.getMode() != DcMotorController.RunMode.RUN_USING_ENCODERS)
                        leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
                    end = 1;
                    break;
            }
        }
        telemetry.addData("end","end");
    }

    /**
     * Turn the robot a specified amount of degrees
     * @param direction: which way to turn, left = true, right = false
     * @param degrees: number of degrees to turn
     * @param power: power of left and right wheels
     */
    public void turn(boolean direction, int degrees, int power){
        //Calculate number of inches to move
        double amount = ROBOT_RADIUS * Math.toRadians(degrees) * 2;

        //Drive specified amount of inches in specified direction
        if(direction == LEFT)
            driveInches(amount, -1*power, power);
        else
            driveInches(amount, power, -1*power);
    }

    /**
     * Stop the motors
     */
    public void stopMotors(){
        motorController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        try{ waitForNextHardwareCycle(); } catch(InterruptedException e) {}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**
     * Convert an inches value to an encoder value
     * @param inches: number of inches
     * @return: encoder value
     */
    private double inchesToEncoder(double inches){
        return inches / CIRCUMFERENCE * 1440;
    }

    /**
     * Convert an encoder value to inches
     * @param encoder: encoder value
     * @return: inches
     */
    private double encoderToInches(double encoder){
        return encoder * CIRCUMFERENCE / 1440;
    }
}
