package org.firstinspires.ftc.teamcode.comp.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.robot.Robot;

// a "simple" class for mapping 2p controller settings

public interface ControllerMap {

    void init(Robot robot, Gamepad gp1, Gamepad gp2);
    void leftBumper(boolean on);
    void rightBumper();
    void buttonB();
    void buttonY();
    void buttonX();
    void buttonA();
    void leftTrigger();
    void rightTrigger();
    void dpadLeft();
    void dpadRight();
    void dpadUp();
    void dpadDown();
    void dpadLeft2();
    void dpadRight2();
    void dpadUp2();
    void dpadDown2();
    void leftBumper2();
    void rightBumper2();
    void leftTrigger2();
    void rightTrigger2();
    void buttonX2();
    void buttonB2();
    void buttonA2();
    void buttonY2();

    default void checkControls(){
        checkSpecialControls();
        checkButtons();
        checkJoysticks();
    }
    void checkSpecialControls();
    void checkButtons();
    void checkJoysticks();


}
