package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripServo.setPosition(.15))
                                .waitSeconds(1)
                                .forward(24)
                                .turn(Math.toRadians(45))
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

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}