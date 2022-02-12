package org.firstinspires.ftc.teamcode.comp.controller;

import static java.lang.Double.max;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

public class ControllerMap {

    private Meccanum meccanum;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private boolean gp2ldown;
    private boolean gp2lup;

    public void init(Meccanum mec, Gamepad gp1, Gamepad gp2){
        meccanum = mec;
        gamepad1 = gp1;
        gamepad2 = gp2;
    }
    private void leftBumper(boolean on){
        if (on) gamepad2.rumble(10);
        else gamepad2.stopRumble();
    }
    private void rightBumper(){}
    private void buttonB(){ meccanum.playSound("scree"); }
    private void buttonY(){ meccanum.playSound("car"); }
    private void buttonX(){}
    private void buttonA(){}
    private void leftTrigger(){}
    private void rightTrigger(){}
    private void dpadLeft(){}
    private void dpadRight(){}
    private void dpadUp(){}
    private void dpadDown(){}
    private void rumble(int time){ gamepad2.rumble(time); }
    private void joystickDriver(double scale){
        meccanum.motorDriveXYVectors(gamepad1.left_stick_x * scale, gamepad1.left_stick_y * scale, gamepad1.right_stick_x * scale);
    }

    private void dpadLeft2(){}
    private void dpadRight2(){}
    private void dpadUp2(){}
    private void dpadDown2() { meccanum.setServo(meccanum.BACK_SERVO_ANGLE); }
    private void leftBumper2(){  }
    private void rightBumper2(){ meccanum.spinnySpin(meccanum.OPTIMAL_SPINNER_POWER); }
    private void buttonY2(){}
    private void leftTrigger2(){}
    private void rightTrigger2(){ meccanum.spinnySpin(meccanum.HIGH_SPINNER_POWER); }
    private void buttonX2(){}
    private void buttonB2(){ meccanum.closeServoFull(); }
    private void buttonA2(){ meccanum.openServoFull(); }
    private void rumble2(int time){ gamepad2.rumble(time); }
    private void joystickDriver2(double scale){
        meccanum.moveArm(gamepad2.left_stick_y * scale);
        /* THIS IS DISABLED< BUT CONCEPT FOR FINE TURNING OF TURNS N STUFF meccanum.rx += gamepad2.right_stick_x/8; */}

    public void checkControls(){

        if (gamepad1.right_trigger > 0.1) joystickDriver(0.2);//
        else joystickDriver(1); //

        if (gamepad1.left_bumper) leftBumper(true);
        else leftBumper(false);

        if (gamepad1.right_bumper) rightBumper();
        if (gamepad1.x) buttonX();
        if (gamepad1.b) buttonB();
        if(gamepad1.dpad_down) dpadDown();
        if(gamepad1.dpad_up) dpadUp();
        if(gamepad1.dpad_right) dpadRight();
        if(gamepad1.dpad_left) dpadLeft();
        if (gamepad1.y) buttonY();
        if (gamepad1.a) buttonA();
    }
    public void checkControls2(){
        if (gamepad2.left_trigger > 0.01) joystickDriver2(0.3); //
        else joystickDriver2(0.8); //


        if(gamepad2.left_bumper) leftBumper2(); //
        if(gamepad2.x) buttonX2(); //
        if(gamepad2.b) buttonB2(); //
        if(gamepad2.a) buttonA2(); //
        if(gamepad2.dpad_down) dpadDown2(); //
        if(gamepad2.dpad_up) dpadUp2(); //
        if(gamepad2.dpad_right) dpadRight2(); //
        if(gamepad2.dpad_left) dpadLeft2(); //

        if (gamepad2.right_trigger > 0.01) rightTrigger2(); //
        else if (gamepad2.right_bumper) rightBumper2(); //
        else meccanum.spinnyStop(); //

    }


}
