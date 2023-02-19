package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61.5, PI))
                                .strafeTo(new Vector2d(35, -70 + 17*sqrt(2)/2))
                                .turn(PI)
                                .lineTo(new Vector2d(35, -15))
                                .lineToLinearHeading(new Pose2d(32, -5, -PI/6))

                                .setReversed(false)
                                .splineTo(new Vector2d(56, -12), 0)
                                .setReversed(true)
                                .splineTo(new Vector2d(32, -5), 5*PI/6)

                                .setReversed(false)
                                .splineTo(new Vector2d(56, -12), 0)
                                .setReversed(true)
                                .splineTo(new Vector2d(32, -5), 5*PI/6).setReversed(false)
                                .splineTo(new Vector2d(56, -12), 0)
                                .setReversed(true)
                                .splineTo(new Vector2d(32, -5), 5*PI/6).setReversed(false)
                                .splineTo(new Vector2d(56, -12), 0)
                                .setReversed(true)
                                .splineTo(new Vector2d(32, -5), 5*PI/6).setReversed(false)
                                .splineTo(new Vector2d(56, -12), 0)
                                .setReversed(true)
                                .splineTo(new Vector2d(32, -5), 5*PI/6)

                                .lineToLinearHeading(new Pose2d(35, -12, 0))
                                .lineTo(new Vector2d(35, -34))
                                .lineTo(new Vector2d(11, -34))

                                .build()
                );
        RoadRunnerBotEntity yourBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, PI))
                                .lineTo(new Vector2d(-35, -15))
                                .lineToLinearHeading(new Pose2d(-32, -5, 7*PI/6))

                                .setReversed(false)
                                .splineTo(new Vector2d(-56, -12), PI)
                                .setReversed(true)
                                .splineTo(new Vector2d(-32, -5), PI/6)

                                .setReversed(false)
                                .splineTo(new Vector2d(-56, -12), PI)
                                .setReversed(true)
                                .splineTo(new Vector2d(-32, -5), PI/6)
                                .setReversed(false)
                                .splineTo(new Vector2d(-56, -12), PI)
                                .setReversed(true)
                                .splineTo(new Vector2d(-32, -5), PI/6)
                                .setReversed(false)
                                .splineTo(new Vector2d(-56, -12), PI)
                                .setReversed(true)
                                .splineTo(new Vector2d(-32, -5), PI/6)
                                .setReversed(false)
                                .splineTo(new Vector2d(-56, -12), PI)
                                .setReversed(true)
                                .splineTo(new Vector2d(-32, -5), PI/6)

                                .lineToLinearHeading(new Pose2d(-35, -12, PI))
                                .lineTo(new Vector2d(-35, -34))
                                .lineTo(new Vector2d(-11, -34))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(yourBot)
                .start();
    }
}