package org.firstinspires.ftc.teamcode.comp.controller.Odo;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.controller.ControllerMap;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;

// a "simple" class for mapping 2p controller settings

public class ControllerMapOdo implements ControllerMap {
    private Robot bot;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    @Override
    public void init(Robot robot, Gamepad gp1, Gamepad gp2){
        bot = (Odo) robot;
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

    }

    @Override
    public void buttonY2() {

    }

    @Override
    public void checkSpecialControls() {

    }

    @Override
    public void checkButtons() {

    }

    @Override
    public void checkJoysticks() {

    }

}
