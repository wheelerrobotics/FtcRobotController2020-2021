package org.firstinspires.ftc.teamcode.comp.controller.Odo;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.controller.ControllerMap;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// a "simple" class for mapping 2p controller settings

public class ControllerMapOdo implements ControllerMap {
    Odo bot = null;
    SampleMecanumDrive drive = null;
    Gamepad gamepad1 = null;
    Gamepad gamepad2 = null;
    boolean notOrbiting = true;
    Trajectory traj = null;
    Trajectory stop = null;
    volatile boolean trajing = false;

    public void init(Robot robot, SampleMecanumDrive d, Gamepad gp1, Gamepad gp2){
        bot = (Odo) robot;
        drive = d;
        gamepad1 = gp1;
        gamepad2 = gp2;
        traj = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(66, -2, PI/2))
                .build();
        stop = drive.trajectoryBuilder(new Pose2d()).forward(0.01).build();
    }

    @Override
    public void init(Robot robot, Gamepad gp1, Gamepad gp2) {

    }

    @Override
    public void leftBumper(boolean on) {

    }

    @Override
    public void rightBumper() {

    }

    @Override
    public void buttonB() {

    }

    @Override
    public void buttonY() {

    }

    @Override
    public void buttonX() {

    }

    @Override
    public void buttonA() {
        notOrbiting = false;
        // robot-radius = 20cm
        //
        // orbit distance = 1ft or
        double newRight = 0;
        double newLeft = gamepad1.left_stick_y;
        bot.motorDrive(newLeft, newLeft, newRight, newRight);
    }

    @Override
    public void leftTrigger() {

    }

    @Override
    public void rightTrigger() {

    }

    @Override
    public void dpadLeft() {

    }

    @Override
    public void dpadRight() {

    }

    @Override
    public void dpadUp() {

    }

    @Override
    public void dpadDown() {

    }

    @Override
    public void dpadLeft2() {

    }

    @Override
    public void dpadRight2() {

    }

    @Override
    public void dpadUp2() {

    }

    @Override
    public void dpadDown2() {

    }

    @Override
    public void leftBumper2() {

    }

    @Override
    public void rightBumper2() {

    }

    @Override
    public void leftTrigger2() {

    }

    @Override
    public void rightTrigger2() {

    }

    @Override
    public void buttonX2() {

    }

    @Override
    public void buttonB2() {

    }

    @Override
    public void buttonA2() {

    }

    @Override
    public void buttonY2() {
        drive.followTrajectoryAsync(traj);
        trajing = true;
    }

    @Override
    public void checkSpecialControls() {

    }

    public void spin180() {

    }

    @Override
    public void checkButtons() {
        if (gamepad2.a) buttonA2();
        if (gamepad2.b) buttonB2();
        if (gamepad2.x) buttonX2();
        if (gamepad2.y) buttonY2();
        if (gamepad1.a) buttonA();
        else notOrbiting = true;
        if (gamepad1.b) buttonB();
        if (gamepad1.x) buttonX();
        if (gamepad1.y) buttonY();
        if (gamepad2.dpad_down) dpadDown2();
        if (gamepad2.dpad_up) dpadUp2();
        if (gamepad2.dpad_left) dpadLeft2();
        if (gamepad2.dpad_right) dpadRight2();
        if (gamepad1.dpad_down) dpadDown();
        if (gamepad1.dpad_up) dpadUp();
        if (gamepad1.dpad_left) dpadLeft();
        if (gamepad1.dpad_right) dpadRight();
    }

    @Override
    public void checkJoysticks() {
        if ((gamepad1.left_stick_x != 0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0) && trajing) {
            trajing = false;
        }
        if (trajing) {
            drive.update();
            return;
        }
        bot.motorDriveXYVectors(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

    }

}
