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
    public void button_b() {

    }

    @Override
    public void button_y() {

    }

    @Override
    public void button_x() {

    }

    @Override
    public void button_a() {

    }

    @Override
    public void left_trigger() {

    }

    @Override
    public void right_trigger() {

    }

    @Override
    public void dpad_left() {

    }

    @Override
    public void dpad_right() {

    }

    @Override
    public void dpad_up() {

    }

    @Override
    public void dpad_down() {

    }

    @Override
    public void dpad_left_2() {

    }

    @Override
    public void dpad_right_2() {

    }

    @Override
    public void dpad_up_2() {

    }

    @Override
    public void dpad_down_2() {

    }

    @Override
    public void left_bumper_2() {

    }

    @Override
    public void right_bumper_2() {

    }

    @Override
    public void left_trigger_2() {

    }

    @Override
    public void right_trigger_2() {

    }

    @Override
    public void button_x_2() {

    }

    @Override
    public void button_b_2() {

    }

    @Override
    public void button_a_2() {

    }

    @Override
    public void button_y_2() {

    }

    @Override
    public void check_special_controls() {

    }

    @Override
    public void check_buttons() {

    }

    @Override
    public void check_joysticks() {

    }
}
