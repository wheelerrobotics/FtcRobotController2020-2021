package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double conePos = 3;

        SplineInterpolator si = new SplineInterpolator(7*PI/6, PI, -0.1, 0.01, 0., -0.00);
        QuinticSpline s = new QuinticSpline(new QuinticSpline.Knot(-28, -4, -30, -33, 30, -6),
                new QuinticSpline.Knot(-60, -12, 10, -0, -0.1, -0), 0.001, 1000, 1000);
        PathSegment seg = new PathSegment(s, si);
        Path p = new Path(seg);
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(60, 10, 0),
                30,
                30,
                100
        );


        SplineInterpolator si2 = new SplineInterpolator( PI,7*PI/6, 0., 0.00001, -0.001, -0.01);
        QuinticSpline s2 = new QuinticSpline(new QuinticSpline.Knot(-62, -12, 10, -0, -0.1, -0),
                new QuinticSpline.Knot(-28, -4, 0, 33, 30, -6), 0.001, 1000, 1000);
        PathSegment seg2 = new PathSegment(s2, si2);
        Path p2 = new Path(seg2);
        MotionProfile profile2 = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(60, 0, 0),
                30,
                30,
                100
        );

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(20),
                new AngularVelocityConstraint(1)
        ));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(40);
        //List<TrajectoryMarker> marks = Arrays.asList(new TrajectoryMarker(2, ()->{}));
        Trajectory t = new Trajectory(p, profile);
        Trajectory t2 = new Trajectory(p2, profile2);

        TrajectorySequence t4 = new TrajectorySequenceBuilder(new Pose2d(-62, -12, 7*PI/6), velConstraint, accelConstraint, 15, 15)
                .addTrajectory(t)
                .lineToLinearHeading(new Pose2d(0, 0,0))
                .build();


        RoadRunnerBotEntity yBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(255), Math.toRadians(273), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, PI))
                                .lineToConstantHeading(new Vector2d(-35, -8)) // drive from start to in front of junction
                                .addDisplacementMarker(20, () -> { // move armature to appropriate heights/poses
                                })
                                .turn(PI/6)// turn to face junction
                                .lineToConstantHeading(new Vector2d(-28, -4)) // move toward junction so cone catches it

                                .build()
                );

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(255), Math.toRadians(273), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, PI))
                                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {})
                                .addTrajectory(t)
/*
                                // NoTurnAround
                                .lineTo(new Vector2d(-35, -15))
                                .setTangent(PI/2)
                                .splineToLinearHeading(new Pose2d(-28, -4, 7*PI/6), PI/2)
                                .addSpatialMarker(new Vector2d(-35, -24), () -> {
                                    //bot.setSlideTarget(Heights.highMin + 200);
                                    //bot.setArmTarget(Heights.upSlantArmPlace + 0.05);
                                    //bot.setWristTarget(Heights.upSlantWristPlace + 0.1);
                                })
                                .addDisplacementMarker(10, () -> {
                                    //bot.setClawTarget(Heights.clawOpen);
                                })


                                // cycleSplineToStack
                                .setReversed(false)


                                .addSpatialMarker(new Vector2d(-34, -12), () -> { // maybe make spatial in future
                                    //bot.setClawTarget(Heights.clawClosed);
                                    //bot.setWristTarget(Heights.levelWristPickup);
                                    //bot.setArmTarget(Heights.levelArmPickup);
                                    //bot.setSlideNextCone();
                                })
                                .addSpatialMarker(new Vector2d(-48, -12), () -> {
                                    //bot.setClawTarget(Heights.clawOpen);
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {})
                                .addTrajectory(t)
                                .lineToLinearHeading(t.end())


                                // cycleSplineToJunction
                                .lineToConstantHeading(new Vector2d(-62, -12))
                                .addSpatialMarker(new Vector2d(-60, -12), () -> {
                                    //bot.setClawTarget(Heights.clawClosed);
                                })
                                .waitSeconds(0.5)
                                .addSpatialMarker(new Vector2d(-60, -12), () -> {
                                    //bot.setSlideTargetDelay(Heights.lowMax, 500)
                                })
                                .setReversed(true)
                                .addTrajectory(t2)
                                .lineToLinearHeading(t2.end())
                                //.lineToConstantHeading(new Vector2d(-36, -12))
                                //.lineToLinearHeading(new Pose2d(-32, -9, 7*PI/6))
                                //.lineToConstantHeading(new Vector2d(-37, 4))
                                .addSpatialMarker(new Vector2d(-48, -12),() -> {
                                    //bot.setSlideTarget(Heights.highMin);
                                    //bot.setClawTarget(Heights.clawClosed);
                                    //bot.setArmTarget(Heights.upSlantArmPlace);
                                })
                                .addSpatialMarker(new Vector2d(-33, -10), () -> {
                                    //bot.setWristTarget(Heights.upSlantWristPlace + 0.05);
                                })
                                .addSpatialMarker(new Vector2d(-28, 10), () -> {
                                    //bot.setClawTarget(Heights.clawOpen);
                                })


                                // cycleSplineToStack
                                .setReversed(false)

                                .setTangent(-PI/2)
                                .splineToLinearHeading(new Pose2d(-60, -11, PI), PI)
                                .addSpatialMarker(new Vector2d(-34, -12), () -> { // maybe make spatial in future
                                    //bot.setClawTarget(Heights.clawClosed);
                                    //bot.setWristTarget(Heights.levelWristPickup);
                                    //bot.setArmTarget(Heights.levelArmPickup);
                                    //bot.setSlideNextCone();
                                })
                                .addSpatialMarker(new Vector2d(-48, -12), () -> {
                                    //bot.setClawTarget(Heights.clawOpen);
                                })


                                // cycleSplineToJunction
                                .lineToConstantHeading(new Vector2d(-62, -12))
                                .addSpatialMarker(new Vector2d(-60, -12), () -> {
                                    //bot.setClawTarget(Heights.clawClosed);
                                })
                                .waitSeconds(0.5)
                                .addSpatialMarker(new Vector2d(-60, -12), () -> {
                                    //bot.setSlideTargetDelay(Heights.lowMax, 500)
                                })
                                .setReversed(true)
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-28, -4, 7*PI/6), PI/2)
                                //.lineToConstantHeading(new Vector2d(-36, -12))
                                //.lineToLinearHeading(new Pose2d(-32, -9, 7*PI/6))
                                //.lineToConstantHeading(new Vector2d(-37, 4))
                                .addSpatialMarker(new Vector2d(-48, -12),() -> {
                                    //bot.setSlideTarget(Heights.highMin);
                                    //bot.setClawTarget(Heights.clawClosed);
                                    //bot.setArmTarget(Heights.upSlantArmPlace);
                                })
                                .addSpatialMarker(new Vector2d(-33, -10), () -> {
                                    //bot.setWristTarget(Heights.upSlantWristPlace + 0.05);
                                })
                                .addSpatialMarker(new Vector2d(-28, 10), () -> {
                                    //bot.setClawTarget(Heights.clawOpen);
                                })


                                // park
                                .addTemporalMarker(0.2, () -> { // maybe make spatial in future
                                    //bot.setClawTarget(Heights.clawClosed);
                                    //bot.setWristTarget(Heights.levelWristPickup);
                                    //bot.setArmTarget(Heights.levelArmPickup);
                                    //bot.setSlideTarget(0);
                                })
                                .lineToLinearHeading(new Pose2d(-35, -12, PI))
                                .lineTo(new Vector2d((conePos == 1 ? -61.5 : (conePos == 2 ? -35.5 : -11)), -12))
                                //default to closest pos to terminal if cone det doesnt work
                                .addDisplacementMarker( () -> {
                                    //bot.setClawTarget(Heights.clawOpen);
                                })
*/

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(yBot)
                .addEntity(myBot)
                .start();
    }
}