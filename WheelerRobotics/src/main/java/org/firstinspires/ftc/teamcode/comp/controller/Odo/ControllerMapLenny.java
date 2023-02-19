package org.firstinspires.ftc.teamcode.comp.controller.Odo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.controller.ControllerMap;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// a "simple" class for mapping 2p controller settings

public class ControllerMapLenny implements ControllerMap {
    //CONCEPT
    /*
    GP1
        Joysticks:
            either standard mecanum or field centric, driver decides
        ABXY DPAD TRIGGERS(zl,zr) BUMPERS(rb,lb) JS-BUTTONS:
            rr paths?
            speed scale mod button
                or norm scale down and speed ramp button
                or both

    GP2
        Joysticks:
            Left:
                y: arm rot (could be x if pref)
                // need a arm place var for cur pos to add to at js scaled speed
            Right:
                y: slide up/down
        Bumpers:
            l: flip wrist
            r: toggle claw
        DPAD:
            up: arm to up position
            left: arm to place position
            right: arm to pickup level
            down: arm to furthest down
        ABXY zl zr jsy jsx:
            preset heights for slides
            auto grab flip place button (maybe dpad)



     */

    Lenny bot = null;
    SampleMecanumDrive drive = null;
    Gamepad gamepad1 = null;
    Gamepad gamepad2 = null;
    Trajectory stop = null;
    volatile boolean trajing = false;

    public void init(Robot robot, SampleMecanumDrive d, Gamepad gp1, Gamepad gp2){
        bot = (Lenny) robot;
        drive = d;
        gamepad1 = gp1;
        gamepad2 = gp2;
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
    }

    @Override
    public void checkSpecialControls() {

    }

    @Override
    public void checkButtons() {
        if (gamepad2.a) buttonA2();
        if (gamepad2.b) buttonB2();
        if (gamepad2.x) buttonX2();
        if (gamepad2.y) buttonY2();
        if (gamepad1.a) buttonA();
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
