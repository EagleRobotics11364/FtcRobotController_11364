package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class AdvTestOdometry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            DcMotorEx armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
            Servo gripServo = hardwareMap.get(Servo.class,"gripServo");

            armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtension.setDirection(DcMotor.Direction.REVERSE);
            armExtension.setTargetPosition(700);
            armExtension.setPower(.5);


            Pose2d startPose = new Pose2d(0, 0, 0);

            drive.setPoseEstimate(startPose);

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripServo.setPosition(.15))
                    .waitSeconds(1)
                    .forward(27)
                    .turn(Math.toRadians(60))
                    .forward(5)

//                               .UNSTABLE_addTemporalMarkerOffset(0, () -> armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION))
//                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> armExtension.setTargetPosition(700))
                    .waitSeconds(.5)

                    .forward(5)
//                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> gripServo.setPosition(0.0))
                    .waitSeconds(1)

                    .back(8)
//                                .UNSTABLE_addTemporalMarkerOffset(5, () -> armExtension.setTargetPosition(0))
                    .waitSeconds(1)

                    .turn(Math.toRadians(45))
                    .strafeRight(22.5)
                    .forward(22.5)

                    .waitSeconds(.5)

                    .back(5)
                    .turn(Math.toRadians(135))

                    .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(trajSeq);
    }
}
