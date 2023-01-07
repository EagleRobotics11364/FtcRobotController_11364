package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TestServor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    DcMotorEx armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
    Servo gripServoLeft = hardwareMap.get(Servo.class,"gripServoLeft");
//        Servo gripServoRight = hardwareMap.get(Servo.class,"gripServoRight");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x) {
                gripServoLeft.setPosition(0.95);
//            gripServoRight.setPosition(0.15);
            }
            if (gamepad2.y) {
                gripServoLeft.setPosition(0.75);
//            gripServoRight.setPosition(0);
            }
        }


    }


}
