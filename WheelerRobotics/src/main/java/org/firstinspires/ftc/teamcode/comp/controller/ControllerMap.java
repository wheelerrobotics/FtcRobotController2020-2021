package org.firstinspires.ftc.teamcode.comp.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.robot.Robot;

// a "simple" class for mapping 2p controller settings

public interface ControllerMap {

    void init(Robot robot, Gamepad gp1, Gamepad gp2);
    void leftBumper(boolean on);
    void rightBumper();
    void button_b();
    void button_y();
    void button_x();
    void button_a();
    void left_trigger();
    void right_trigger();
    void dpad_left();
    void dpad_right();
    void dpad_up();
    void dpad_down();
    void dpad_left_2();
    void dpad_right_2();
    void dpad_up_2();
    void dpad_down_2();
    void left_bumper_2();
    void right_bumper_2();
    void left_trigger_2();
    void right_trigger_2();
    void button_x_2();
    void button_b_2();
    void button_a_2();
    void button_y_2();

    default void check_controls(){
        check_special_controls();
        check_buttons();
        check_joysticks();
    }
    void check_special_controls();
    void check_buttons();
    void check_joysticks();


}
