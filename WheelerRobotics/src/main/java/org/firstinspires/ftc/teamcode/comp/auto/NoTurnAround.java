package org.firstinspires.ftc.teamcode.comp.auto;

import static java.lang.Math.PI;
import static java.lang.Math.cos;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.helpers.Heights;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous(name = "normal auto")
@Config
public class NoTurnAround extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    public int currentMovementID = -2;
    public int conePos = 0;
    public TrajectorySequence cycleSplineToJunction, park, cycleSplineToStack, noTurnAround = null;

    boolean NEW_MOVEMENT = true;
    boolean DISTANCE_CORRECT = false;
    boolean FIRST_CORRECT = false;


    Trajectory toStack = null;
    Trajectory toJunction = null;


    public static boolean movement = true;
    Lenny bot = new Lenny();
    SampleMecanumDrive drive = null;
    public boolean buttonUnpressed = false;
    public void runOpMode() {

        bot.init(hardwareMap);
        bot.slideinit();
        bot.cawtinit();
        //bot.detinit("Webcam 1");
        while (bot.getDistance(DistanceUnit.INCH) > 90);

        drive = new SampleMecanumDrive(hardwareMap);

        //correctPos(70);


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
                new TranslationalVelocityConstraint(30),
                new AngularVelocityConstraint(1)
        ));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(30);

        toStack = new Trajectory(p, profile);
        toJunction = new Trajectory(p2, profile2);

        noTurnAround = drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, PI))
                .addDisplacementMarker(20, () -> { // move armature to appropriate heights/poses
                    bot.setSlideTarget(Heights.highMin + 200);
                    bot.setArmTarget(Heights.levelArmPlace);
                    bot.setWristTarget(Heights.upSlantWristPlace + 0.1);
                })
                .lineToConstantHeading(new Vector2d(-32, -12)) // drive from start to in front of junction
                .turn(PI/6)// turn to face junction
                .lineToConstantHeading(new Vector2d(-28, -4)) // move toward junction so cone catches it
                .build();

        cycleSplineToStack = drive.trajectorySequenceBuilder(noTurnAround.end())
                .addTemporalMarker(0, ()->{
                    bot.setClawTarget(Heights.clawOpen); // drop cone
                })
                .waitSeconds(0.4) // wait for cone to fall
                .setReversed(false) // stop reversage if coming from tojun
                .UNSTABLE_addDisplacementMarkerOffset(3, () -> { // reset arm/wrist/claw/slides to pickup poses
                    bot.setClawTarget(Heights.clawClosed);
                    bot.setWristTarget(Heights.levelWristPickup);
                    bot.setArmTarget(Heights.levelArmPickup);
                    bot.setSlideNextCone();
                })
                .UNSTABLE_addDisplacementMarkerOffset(15, () -> { // open claw to get ready to pickup cone from stack
                    bot.setClawTarget(Heights.clawOpen);
                })
                .addTrajectory(toStack) // move in custom spline towards stack
                .build();

        cycleSplineToJunction = drive.trajectorySequenceBuilder(toStack.end()) // might be better as cycleSplineToStack.end
                .lineToConstantHeading(new Vector2d(-62, -12)) // move towards stack from detected position
                .addSpatialMarker(new Vector2d(-62, -12), () -> { // grab cone
                    bot.setClawTarget(Heights.clawClosed);
                })
                .waitSeconds(0.6) // wait so that the claw closes before lifting the cone
                .setReversed(true)

                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    bot.setSlideTarget(Heights.lowMax); // lift cone off stack
                })
                .UNSTABLE_addDisplacementMarkerOffset(3,() -> { // put slides/wrist to place position
                    bot.setSlideTarget(Heights.highMin);
                    bot.setClawTarget(Heights.clawClosed);
                    bot.setWristTarget(Heights.levelWristPlace);
                })
                .UNSTABLE_addDisplacementMarkerOffset(20, () -> { // put arm in place position
                    bot.setArmTarget(Heights.levelArmPlace);
                })
                .UNSTABLE_addDisplacementMarkerOffset(40, () -> { // slant wrist (couldnt do before because would have hit slides when arm moved)
                    bot.setWristTarget(Heights.upSlantWristPlace + 0.05);
                })
                .addTrajectory(toJunction) // move in custom spline to junction
                // NOTE: might need to manually update position after this custom spline
                .build();


        ElapsedTime cooldown = new ElapsedTime();
        telemetry.addLine(String.format("inited in %f seconds", cooldown.seconds()));
        telemetry.update();

        waitForStart();
        cooldown.reset();
        bot.setWristTarget(Heights.levelWristPickup); // set wrist to pickup position
        bot.setArmTarget(Heights.levelArmPickup); // set arm to pickup position
        bot.setClawTarget(Heights.clawClosed); // close claw around preload cone
        while (cooldown.milliseconds() < 400) bot.tick(); // tick while waiting for precious actions to complete
        bot.setWristTarget(Heights.levelWristPlace); // rotate cone so it is ready to place
        bot.setSlideTarget(Heights.lowMin); // move cone up so it doesnt hit ground when rotating
        while (cooldown.milliseconds() < 1300) bot.tick(); // tick while waiting for precious actions to complete
        bot.setArmTarget(Heights.upSlantArmPlace); // move arm to place position
        bot.tick(); // tick

        cooldown.reset(); // reset cooldown (used for vision timing)

        while (conePos == 0 && cooldown.milliseconds() < 3000) conePos = bot.getPrincipalTag(); // scan for tag
        conePos = conePos == 0 ? 3 : conePos; // if no tag detected, default to position closest to terminal

        telemetry.addLine(String.format("detected cone in %f seconds", cooldown.seconds())); // print stuff
        telemetry.addLine(String.format("detected cone position %f seconds", (float) conePos)); // print stuff
        telemetry.update(); // print stuff

        park = drive.trajectorySequenceBuilder(cycleSplineToJunction.end())
                .addTemporalMarker(1, () -> { // put everything in "teleop ready" mode
                    bot.setClawTarget(Heights.clawClosed);
                    bot.setWristTarget(Heights.levelWristPickup);
                    bot.setArmTarget(Heights.levelArmPickup);
                    bot.setSlideTarget(0);
                })
                .lineToLinearHeading(new Pose2d(-35, -12, PI)) // go to parking space 2 and turn so doesnt hit junctions when parking
                .lineTo(new Vector2d((conePos == 1 ? -61.5 : (conePos == 2 ? -35.5 : -11)), -12)) // go to parking space

                .build();


        //drive.followTrajectorySequence(noTurnAround);
        drive.setPoseEstimate(noTurnAround.start());

        drive.followTrajectorySequenceAsync(noTurnAround);

        while (opModeIsActive()) {
            drive.update(); // tick roadrunner async
            bot.tick(); // tick ds (daniel system) async

            if (!drive.isBusy()){ // update movement if done with prev
                currentMovementID++;
                NEW_MOVEMENT = true;
            }
            if (NEW_MOVEMENT) {
                //cool alternative booleans:
                // "13579".contains(valueOf(currentMovementID)) && currentMovementID < 10
                // "2468".contains(valueOf(currentMovementID)) || currentMovementID == 10 && currentMovementID < 20

                if (DISTANCE_CORRECT && false) {
                    correctPos(20); // updates x based on distance sensor reads
                }
                if (currentMovementID == 0) drive.followTrajectorySequenceAsync(noTurnAround);

                if (currentMovementID % 2 == 1 &&
                        currentMovementID > 0 && // redundant for readability
                        currentMovementID <= 4) {
                    drive.followTrajectorySequenceAsync(cycleSplineToStack);
                    DISTANCE_CORRECT = true;
                }
                if (currentMovementID % 2 == 0 &&
                        currentMovementID > 0 &&
                        currentMovementID < 4) drive.followTrajectorySequenceAsync(cycleSplineToJunction);
                if (currentMovementID == 10) {
                    updateJunCycle(5);
                    drive.followTrajectorySequenceAsync(cycleSplineToJunction);
                }
                if (currentMovementID == 5) drive.followTrajectorySequenceAsync(park);
                if (currentMovementID == 6) break;
            }
            NEW_MOVEMENT = false;

            telemetry.addData("Current Movement ID: ", currentMovementID);
            telemetry.update();
        }
    }
    public void correctPos(int x) {
        DISTANCE_CORRECT = false;
        double distance = 0;
        for (int i = 0; i< x; i++) {
            double td = 100;
            while (td > 50) td = bot.getDistance(DistanceUnit.INCH);
            distance += td;
        }
        distance /= x;
        telemetry.addData("dist", distance);
        telemetry.update();

        double rot = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + PI;
        double coser = cos(rot);

        drive.setPoseEstimate(new Pose2d(-72 + (distance) + (FIRST_CORRECT ? 5.8 : 1.5), drive.getPoseEstimate().getY(), rot));
        updateJunCycle(0);
        FIRST_CORRECT = false;
    }
    public void updateJunCycle(double n) {
        if (n ==0) {

            cycleSplineToJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-62, -12))
                    .addSpatialMarker(new Vector2d(-62, -12), () -> {
                        bot.setClawTarget(Heights.clawClosed);
                    })
                    .waitSeconds(1)
                    .setReversed(true)

                    .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                        bot.setSlideTarget(Heights.lowMax);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(3,() -> {
                        bot.setSlideTarget(Heights.highMin);
                        bot.setClawTarget(Heights.clawClosed);
                        bot.setWristTarget(Heights.levelWristPlace);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(20, () -> {
                        bot.setArmTarget(Heights.upSlantArmPlace);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(40, () -> {
                        bot.setWristTarget(Heights.upSlantWristPlace + 0.05);
                    })
                    .addTrajectory(toJunction)
                    // NOTE: might need to manually update position after this custom spline
                    .build();
        }
        if (n == 5) {
            cycleSplineToJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-62, -12))
                    .addSpatialMarker(new Vector2d(-62, -12), () -> {
                        bot.setClawTarget(Heights.clawClosed);
                    })
                    .waitSeconds(1)
                    .setReversed(true)

                    .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                        bot.setSlideTarget(Heights.lowMax);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(3,() -> {
                        bot.setSlideTarget(Heights.highMin);
                        bot.setClawTarget(Heights.clawClosed);
                        bot.setWristTarget(Heights.levelWristPlace);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(20, () -> {
                        bot.setArmTarget(Heights.upSlantArmPlace);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(40, () -> {
                        bot.setWristTarget(Heights.upSlantWristPlace + 0.05);
                    })
                    .addTrajectory(toJunction)
                    // NOTE: might need to manually update position after this custom spline
                    .build();
        }

    }

}
