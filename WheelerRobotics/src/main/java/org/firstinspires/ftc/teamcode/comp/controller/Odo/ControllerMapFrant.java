package org.firstinspires.ftc.teamcode.comp.controller.Odo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.controller.ControllerMap;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;

// a "simple" class for mapping 2p controller settings
@Config
public class ControllerMapFrant implements ControllerMap {
    Frant bot = null;
    Gamepad gamepad1 = null;
    Gamepad gamepad2 = null;
    boolean notOrbiting = true;
    boolean clawToggledAlready = false;
    public static double highHeight = 79.0;
    public static double medHeight = 59.0;
    public static double lowHeight = 38.0;

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

    // 5 16.5 +- 0.2
    // 4 13.5 +- 0.2
    // 3 10.0 +- 0.2
    // 2 6.0  +- 0.2
    @Override
    public void dpadLeft2() {
        bot.setTargetArmHeight(6.0); // 2

        bot.driverOperating = false;

    }

    @Override
    public void dpadRight2() {
        bot.setTargetArmHeight(13.5); // 4

        bot.driverOperating = false;

    }

    @Override
    public void dpadUp2() {
        bot.setTargetArmHeight(16.5); // 5

        bot.driverOperating = false;
    }

    @Override
    public void dpadDown2() {
        bot.setTargetArmHeight(10.0); // 3

        bot.driverOperating = false;

    }

    @Override
    public void leftBumper2() {
        bot.setTargetArmHeight(highHeight); // 3

        bot.driverOperating = false;

    }

    @Override
    public void rightBumper2() {
        bot.setTargetArmHeight(medHeight); // 3

        bot.driverOperating = false;


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
        bot.setTargetArmHeight(lowHeight); // 3

        bot.driverOperating = false;

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

        if (gamepad2.right_bumper) rightBumper2();
        if (gamepad2.left_bumper) leftBumper2();

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
        bot.motorDriveXYVectors(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        if (gamepad2.left_stick_y != 0) bot.driverOperating = true;
        if (bot.driverOperating) bot.armDrive(-gamepad2.left_stick_y);
    }

}
