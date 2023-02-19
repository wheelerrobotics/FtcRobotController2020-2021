package org.firstinspires.ftc.teamcode.comp.auto;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.comp.helpers.Heights;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class TurnAround extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    public int currentMovementID = 0;
    public int conePos = 2;



    public static boolean movement = true;
    Lenny bot = new Lenny();
    SampleMecanumDrive drive = null;
    public boolean buttonUnpressed = false;
    public void runOpMode() {

        // sequence:
        /*

        START SIDEWAYS WITH CAM ON SIDE
        1 SETUP
            1 Scan signal
            2 Grab preload
                1 close claw
                2 rotate arm up to get cone through slides
            2.5 Move Toward signal and Turn (if necessary)
                1 move forward
                2 turn 180
            3 Move to in front of high junction
            3 rotate to start orientation of spline to cones (if not too wide in rotation to do over 3, do on 4)
            3 Move preload cone to apropriate location
                1 slides up to high
                1 wrist wrist not all the way level (slant for easier placing at found angle)
                1.5 (when slides above mid) arm down to not all the way level (slant for easier placing at found angle)
            5 Place Preload Cone On high junction
                1 open claw
        2 CYCLE
            1 Put Claw Into Pickup Pos
                (might want to close claw in this step to make less error prone)
                1 rotate wrist to vertical
                2 rotate arm to pickup (level)
                2 rotate wrist to pickup (level)
                3 drop slides to corresponding cone height
            1 Drive spline from junction to stack
            2 Grab cone from stack
                1 close claw
                2 move slides up (not all the way)
            3 Drive spline to junction from stack
            3 cone to appropriate position
                1 move slides to high height
                1 arm over to not all the way level in place side (slant for easier placing)
                2 wrist to not all the way level (slant for easier placing)
            4 place cone
                1 open claw
        3 PARK
            1 park!
                1 move everything to pickup position
                    see 1 above
                2 drive to indicated value



         */

        // Move Toward Signal

        // strafe a bit
        TrajectorySequence oneSixTwoFive = drive.trajectorySequenceBuilder(new Pose2d(35, -61.5, 0))
                .strafeTo(new Vector2d(35, -70 + 17*sqrt(2)/2))
                .turn(PI)
                .lineTo(new Vector2d(35, -15))
                .addSpatialMarker(new Vector2d(35, -24), () -> {
                    bot.setSlideTarget(Heights.high);
                    bot.setArmTarget(Heights.upSlantArmPlace);
                })
                .lineToLinearHeading(new Pose2d(32, -5, 5*PI/6))
                .addSpatialMarker(new Vector2d(32, -5), () -> {})
                .build();

        TrajectorySequence cycleSplineToStackTurnAround = drive.trajectorySequenceBuilder(new Pose2d(32, -5, 5*PI/6))
                .setReversed(true)

                .addTemporalMarker(0.2, () -> { // maybe make spatial in future
                    bot.setClawTarget(Heights.clawClosed);
                    bot.setWristTarget(Heights.levelWristPickup);
                    bot.setArmTarget(Heights.levelArmPickup);
                    bot.setSlideTarget(Heights.cone3);
                })

                .addSpatialMarker(new Vector2d(40, -12), () -> {
                    bot.setClawTarget(Heights.clawOpen);
                })
                .splineTo(new Vector2d(56, -12), 0)
                .addSpatialMarker(new Vector2d(56, -12), () -> {
                    bot.setClawTarget(Heights.clawClosed);
                })
                .build();

        TrajectorySequence cycleSplineToJunctionTurnAround = drive.trajectorySequenceBuilder(cycleSplineToStackTurnAround.end())
                .addTemporalMarker(0.2, () -> {
                    bot.setSlideTarget(Heights.low);
                })
                .waitSeconds(0.6) // wait for slide to get to height
                .setReversed(false)
                .splineTo(new Vector2d(32, -5), 5*PI/6)
                .addSpatialMarker(new Vector2d(40, -12), () -> {
                    bot.setSlideTarget(Heights.high);
                    bot.setWristTarget(Heights.levelWristPlace);
                    bot.setArmTarget(Heights.upSlantArmPlace);
                })
                .addSpatialMarker(new Vector2d(32, -5), () -> {
                    bot.setClawTarget(Heights.clawOpen);
                })
                .build();


        drive = new SampleMecanumDrive(hardwareMap);
        bot.init(hardwareMap);
        bot.slideinit();
        bot.cawtinit();
        bot.detinit();
        ElapsedTime cooldown = new ElapsedTime();

        waitForStart();
        cooldown.reset();
        while (conePos == 0 && cooldown.milliseconds() < 3000) conePos = bot.getPrincipalTag();

        TrajectorySequence parkTurnAround = drive.trajectorySequenceBuilder(new Pose2d(32, -5, 5*PI/6))
                .addTemporalMarker(0.2, () -> { // maybe make spatial in future
                    bot.setClawTarget(Heights.clawClosed);
                    bot.setWristTarget(Heights.levelWristPickup);
                    bot.setArmTarget(Heights.levelArmPickup);
                    bot.setSlideTarget(Heights.cone3);
                })
                .lineToLinearHeading(new Pose2d(35, -12, PI))
                .lineTo(new Vector2d(35, -34))
                .lineTo(new Vector2d((conePos == 1 ? 11 : (conePos == 2 ? 35 : 61.5)), -34))
                .addDisplacementMarker(() -> {
                    bot.setClawTarget(Heights.clawOpen);
                })
                .build();


        bot.setClawTarget(Heights.clawClosed);
        bot.setWristTarget(Heights.levelWristPlace); // TODO: MAKE SURE CAW LOGIC IN PLACE BEFORE RUNNING
        bot.setArmTarget(Heights.middleSlidesArm);

        while (opModeIsActive()) {
            drive.update();
            if (!drive.isBusy() && !bot.isBusy()) currentMovementID++;
            if (currentMovementID == 0) drive.followTrajectorySequenceAsync(oneSixTwoFive);
            //cool alternative booleans:
            // "13579".contains(valueOf(currentMovementID)) && currentMovementID < 10
            // "2468".contains(valueOf(currentMovementID)) || currentMovementID == 10 && currentMovementID < 20
            if (currentMovementID == 1 ||
                    currentMovementID == 3 ||
                    currentMovementID == 5 ||
                    currentMovementID == 7 ||
                    currentMovementID == 9) drive.followTrajectorySequenceAsync(cycleSplineToStackTurnAround);
            if (currentMovementID == 2 ||
                    currentMovementID == 4 ||
                    currentMovementID == 6 ||
                    currentMovementID == 8 ||
                    currentMovementID == 10) drive.followTrajectorySequenceAsync(cycleSplineToJunctionTurnAround);
            if (currentMovementID == 11) drive.followTrajectorySequenceAsync(parkTurnAround);
        }
    }

}
