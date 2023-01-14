package org.firstinspires.ftc.teamcode.comp.controller.Odo;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.controller.ControllerMap;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;

// a "simple" class for mapping 2p controller settings

public class ControllerMapFrantFC implements ControllerMap {
    Frant bot = null;
    Gamepad gamepad1 = null;
    Gamepad gamepad2 = null;
    boolean notOrbiting = true;
    boolean clawToggledAlready = false;

    public void init(Robot robot, Gamepad gp1, Gamepad gp2){
        bot = (Frant) robot;
        gamepad1 = gp1;
        gamepad2 = gp2;
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

        bot.setClaw(!bot.getClaw());
        clawToggledAlready = true;

    }

    @Override
    public void buttonY2() {

    }

    @Override
    public void checkSpecialControls() {

    }

    public void spin180() {

    }

    @Override
    public void checkButtons() {
        if (gamepad2.a && !clawToggledAlready) buttonA2();
        else if (!gamepad2.a) clawToggledAlready = false;
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
        bot.motorDriveFieldCentricVectors(-gamepad1.left_stick_x * 1.1, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        bot.armDrive(-gamepad2.left_stick_y);
    }

}
