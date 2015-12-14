package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Lili on 12/8/2015.
 */
public class TeleOp1 extends OpMode {
    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor grabLeft;
    DcMotor grabRight;
    Servo buttonLeft;
    Servo buttonRight;
    Servo dump;

    double buttonLeftPostion = 0;
    double buttonRightPosition = 0;
    double dumpPostion = 0;
    double buttonDelta = 0.01;
    double dumpDelta = 0.01;

    public void loop(){
        double y2 = gamepad1.left_stick_y;
        // double x2 = gamepad1.left_stick_x;
        double y = gamepad1.right_stick_y;
        // double x = gamepad1.left_stick_x;

        motorLeft.setPower(y);
        motorRight.setPower(y2);

        if(gamepad1.left_bumper){
            buttonLeftPostion += buttonDelta;
        }

        if(gamepad1.right_bumper){
            buttonLeftPostion -= buttonDelta;
        }

        if(gamepad1.a){
            buttonRightPosition += buttonDelta;
        }

        if(gamepad1.b){
            buttonRightPosition -= buttonDelta;
        }

        if(gamepad2.right_bumper) {
            dumpPostion += dumpDelta;
        }

        if(gamepad2.left_bumper) {
            dumpPostion -= dumpDelta;
        }

        buttonLeftPostion = Range.clip(buttonLeftPostion, 0, 1);
        buttonRightPosition = Range.clip(buttonRightPosition, 0, 1);
        dumpPostion = Range.clip(dumpPostion, 0, 1);

        telemetry.addData("dump position",  ": " + String.format("%.2f", dumpPostion));

        //  buttonLeft.setPosition(buttonLeftPostion);
        //buttonRight.setPosition(buttonRightPosition);
        dump.setPosition(dumpPostion);

    }

    public void init(){
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        //grabLeft = hardwareMap.dcMotor.get("grabLeft");
        //grabRight = hardwareMap.dcMotor.get("grabRight");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //buttonLeft = hardwareMap.servo.get("servo_3"); // channel 3
        //buttonRight = hardwareMap.servo.get("servo_2"); // channel 2
        dump = hardwareMap.servo.get("dump"); // channel 1
    }

}
