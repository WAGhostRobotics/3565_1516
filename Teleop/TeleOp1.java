package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 7876 on 12/9/2015.
 *
 * Authors: Lili,
 *
 * Teleop 3565
 *
 * Versions: (ADD AS YOU EDIT)
 * (Big changes.small changes)
 * 1.0: Added first versions of methods, untested. Completed 12/17/15. Contributors: Lili
 */
public class TeleOp1 extends OpMode {
    //Declare motors and servos
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private Servo buttonLeft;
    private Servo buttonRight;
    private Servo dump;
    private Servo grab;

    //Declare variables
    double buttonLeftPostion;
    double buttonRightPosition;
    double dumpPostion;
    double buttonDelta;
    double dumpDelta;

    public void loop(){
        //Get joystick values
        double y2 = gamepad1.left_stick_y;
        double y = gamepad1.right_stick_y;

        //Set the motor power
        motorLeft.setPower(y);
        motorRight.setPower(y2);

        if(gamepad1.left_bumper){ //Button left forward
            buttonLeftPostion += buttonDelta;
        }

        if(gamepad1.right_bumper){ //Button left backward
            buttonLeftPostion -= buttonDelta;
        }

        if(gamepad1.a){ //Button right forward
            buttonRightPosition += buttonDelta;
        }

        if(gamepad1.b){ //Button right backward
            buttonRightPosition -= buttonDelta;
        }

        //DOESN'T WORK
        if(gamepad2.right_bumper) { //Dump forward
            dumpPostion += dumpDelta;
            dumpPostion = Range.clip(dumpPostion, 0, 1);
            telemetry.addData("dump position",  ": " + String.format("%.2f", dumpPostion));
            dump.setPosition(dumpPostion);
            telemetry.addData("dump position actual", ": " + String.format("%.2f", dump.getPosition()));
        }

        if(gamepad2.left_bumper) { //Dump backward
            dumpPostion = Range.clip(dumpPostion, 0, 1);
            telemetry.addData("dump position",  ": " + String.format("%.2f", dumpPostion));
            dump.setPosition(dumpPostion);
            telemetry.addData("dump position actual", ": " + String.format("%.2f", dump.getPosition()));
        }

        //Clip range
       // buttonLeftPostion = Range.clip(buttonLeftPostion, 0, 1);
       // buttonRightPosition = Range.clip(buttonRightPosition, 0, 1);
       // dumpPostion = Range.clip(dumpPostion, 0, 1);

        //(error checking :( )
        //telemetry.addData("dump position",  ": " + String.format("%.2f", dumpPostion));

        //Set servo positions
//        buttonLeft.setPosition(buttonLeftPostion);
  //      buttonRight.setPosition(buttonRightPosition);
    //    dump.setPosition(dumpPostion);

    }

    public void init(){
        //Initialize motors
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        //Initialize servos
        buttonLeft = hardwareMap.servo.get("buttonl"); // channel 3
        buttonRight = hardwareMap.servo.get("buttonr"); // channel 2
        dump = hardwareMap.servo.get("dump"); // channel 1
        //grab = hardwareMap.servo.get("servo_4"); //channel 4

        //Initialize variables
        buttonLeftPostion = 0;
        buttonRightPosition = 0;
        dumpPostion = 0;
        buttonDelta = 0.01;
        dumpDelta = 0.01;
    }

}
