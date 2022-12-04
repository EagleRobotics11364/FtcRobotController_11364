/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(preselectTeleOp = "Beta_TeleOp")
public class VisionWithAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");

        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setDirection(DcMotor.Direction.REVERSE);
        armExtension.setTargetPosition(700);
        armExtension.setPower(.5);


        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);
        TrajectorySequence base;

        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            base = drive.trajectorySequenceBuilder(startPose)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripServo.setPosition(.15))
                    .waitSeconds(1)
                    .forward(25)
                    .turn(Math.toRadians(68.5))
                    .forward(5)

                    .UNSTABLE_addTemporalMarkerOffset(1, () -> armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> armExtension.setTargetPosition(700))
                    .waitSeconds(2)

                    .forward(6)
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> gripServo.setPosition(0.0))
                    .UNSTABLE_addTemporalMarkerOffset(5, () -> armExtension.setTargetPosition(0))
                    .waitSeconds(5)

                    .back(10)
                    .turn(Math.toRadians(55))
                    .forward(20)
                    .build();


        } else if(tagOfInterest.id == MIDDLE){
            base = drive.trajectorySequenceBuilder(startPose)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripServo.setPosition(.15))
                    .waitSeconds(1)
                    .forward(25)
                    .turn(Math.toRadians(68.5))
                    .forward(5)

                    .UNSTABLE_addTemporalMarkerOffset(1, () -> armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> armExtension.setTargetPosition(700))
                    .waitSeconds(2)

                    .forward(6)
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> gripServo.setPosition(0.0))
                    .UNSTABLE_addTemporalMarkerOffset(5, () -> armExtension.setTargetPosition(0))
                    .waitSeconds(5)

                    .back(10)
                    .turn(Math.toRadians(55))
                    .build();

        } else {
            base = drive.trajectorySequenceBuilder(startPose)
                   .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripServo.setPosition(.15))
                    .waitSeconds(1)
                    .forward(25)
                    .turn(Math.toRadians(68.5))
                    .forward(5)

                    .UNSTABLE_addTemporalMarkerOffset(1, () -> armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> armExtension.setTargetPosition(700))
                    .waitSeconds(2)

                    .forward(6)
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> gripServo.setPosition(0.0))
                    .UNSTABLE_addTemporalMarkerOffset(5, () -> armExtension.setTargetPosition(0))
                    .waitSeconds(5)

                    .back(10)
                    .turn(Math.toRadians(55))
                    .back(17)

                    .build();
        }



        waitForStart();
        drive.followTrajectorySequence(base);





    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
