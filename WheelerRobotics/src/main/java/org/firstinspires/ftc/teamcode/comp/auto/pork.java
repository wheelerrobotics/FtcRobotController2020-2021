package org.firstinspires.ftc.teamcode.comp.auto;

import static java.lang.Math.PI;
import static java.lang.Math.cos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.helpers.Heights;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "pork")
@Config
public class pork extends LinearOpMode {
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
        bot.detinit("Webcam 2");
        //while (bot.getDistance(DistanceUnit.INCH) > 90);

        drive = new SampleMecanumDrive(hardwareMap);

        //correctPos(70);

        ElapsedTime cooldown = new ElapsedTime();
        telemetry.addLine(String.format("inited in %f seconds", cooldown.seconds()));
        telemetry.update();

        while (opModeInInit()); // do nothing

        cooldown.reset(); // reset cooldown (used for vision timing)

        // if no tag detected, default to position closest to terminal
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        tele.addLine(String.format("detected cone in %f seconds", cooldown.seconds())); // print stuff // print stuff // print stuff


        while (conePos == 0 && cooldown.milliseconds() < 3000) conePos = bot.getPrincipalTag(); // scan for tag
        tele.addLine(String.format("detected cone position %f", (float) conePos));
        tele.update();
        conePos = conePos == 0 ? 3 : conePos;


        park = drive.trajectorySequenceBuilder(new Pose2d(0, 0, PI))
                .lineTo(new Vector2d(0, 28))
                .lineTo(new Vector2d(conePos == 1 ? -24 : (conePos == 2 ? 1 : 24), 28))
                .build();


        //drive.followTrajectorySequence(noTurnAround);
        drive.setPoseEstimate(park.start());
        drive.followTrajectorySequence(park);
    }
    public void correctPos(int x) {
        DISTANCE_CORRECT = false;
        double distance = 0;
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        for (int i = 0; i< x; i++) {
            double td = 100;
            while (td > 20 || timeout.milliseconds() > 2000) td = bot.getDistance(DistanceUnit.INCH);
            distance += td;
        }
        if (timeout.milliseconds() > 2000) return;
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
                    //.lineToConstantHeading(new Vector2d(-62, -8))
                    .lineToConstantHeading(new Vector2d(-62.3, -11))
                    .addTemporalMarker(0.3, () -> {
                        bot.setClawTarget(Heights.clawClosed);
                    })
                    .waitSeconds(0.5)
                    .setReversed(true)

                    .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                        bot.setSlideTarget(Heights.lowMax);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(3,() -> {
                        bot.setSlideTarget(Heights.highMin);
                        bot.setClawTarget(Heights.clawClosed);
                        bot.setArmTarget(Heights.upSlantArmPlace);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(20, () -> {
                        bot.setWristTarget(Heights.levelWristPlace);

                    })
                    .lineToConstantHeading(new Vector2d(-35, -12))
                    .turn(PI/4)
                    .lineToConstantHeading(new Vector2d(-32, -1.5))
                    // NOTE: might need to manually update position after this custom spline
                    .build();
        }
        if (n == 5) {
            cycleSplineToJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    //.lineToConstantHeading(new Vector2d(-62, -8))
                    .lineToConstantHeading(new Vector2d(-64.3, -8))
                    .addTemporalMarker(0.6, () -> {
                        bot.setClawTarget(Heights.clawClosed);
                    })
                    .addTemporalMarker(0.9, () -> {
                        bot.setSlideTarget(Heights.lowMax);
                    })
                    .waitSeconds(0.8)
                    .setReversed(true)
                    .UNSTABLE_addDisplacementMarkerOffset(3,() -> {
                        bot.setSlideTarget(Heights.highMax - 800);
                        bot.setClawTarget(Heights.clawClosed);
                        bot.setArmTarget(Heights.upSlantArmPlace);
                    })
                    .UNSTABLE_addDisplacementMarkerOffset(20, () -> {
                        bot.setWristTarget(0); // 0.1 on other side

                    })
                    .lineToConstantHeading(new Vector2d(-35, -12))
                    .lineToLinearHeading(new Pose2d(-31.5, -4.4, 5*PI/4 + 0.15))
                    // NOTE: might need to manually update position after this custom spline
                    .build();
        }

    }

}
