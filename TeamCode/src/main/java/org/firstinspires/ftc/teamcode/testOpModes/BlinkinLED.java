package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


@TeleOp
public class BlinkinLED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RevBlinkinLedDriver ledLights = hardwareMap.get(RevBlinkinLedDriver.class,"ledLights");
        DcMotorEx armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");

        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            double value = gamepad2.right_stick_y;
            if(armExtension.getCurrentPosition() > 0) {
                if(value >= 0) {
                    armExtension.setPower((value * 0.5) + 0.15);
                } else {
                    armExtension.setPower((value * 0.3) + 0.1);
                }
            } else {
                if(value >= 0) {
                    armExtension.setPower((value * 0.5));
                } else {
                    armExtension.setPower((value * 0.3));
                }
            }
            if (armExtension.getCurrentPosition() > 150) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (armExtension.getCurrentPosition() < 150 && armExtension.getCurrentPosition() > -50) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

            telemetry.addData("Position: ",armExtension.getCurrentPosition());
            telemetry.addData("Target: ",armExtension.getTargetPosition());
            telemetry.addData("Power: ",armExtension.getPower());
            telemetry.update();
        }
        armExtension.setPower(0.0);
    }


}
