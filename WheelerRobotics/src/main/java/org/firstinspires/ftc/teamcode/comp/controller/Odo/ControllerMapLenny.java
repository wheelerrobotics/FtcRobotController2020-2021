package org.firstinspires.ftc.teamcode.comp.controller.Odo;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.comp.controller.ControllerMap;
import org.firstinspires.ftc.teamcode.comp.helpers.Heights;
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
    }
    public void init(Robot robot, Gamepad gp1, Gamepad gp2){
        bot = (Lenny) robot;
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

    boolean TARGET_SLIDE = false;
    @Override
    public void dpadLeft2() {

        bot.setSlideTarget(coneStackPresets ? Heights.cone3 : Heights.lowMin);
        TARGET_SLIDE = true;
    }

    @Override
    public void dpadRight2() {
        bot.setSlideTarget(coneStackPresets ? Heights.cone5 : Heights.midMin);
        TARGET_SLIDE = true;

    }

    @Override
    public void dpadUp2() {
        bot.setSlideTarget(coneStackPresets ? Heights.cone4 : Heights.highMin);
        TARGET_SLIDE = true;
    }

    @Override
    public void dpadDown2() {

        bot.setSlideTarget(Heights.cone2);
        TARGET_SLIDE = true;
    }

    @Override
    public void leftBumper2() {
        slantArm = !slantArm;

    }

    boolean armPlace = false;
    boolean slantArm = false;
    @Override
    public void rightBumper2() {
        if (clawOpen) {
            clawOpen = false;
            sinceClawMove.reset();
        }
        armPlace = !armPlace;
        sinceArmMove.reset();


    }

    @Override
    public void leftTrigger2() {

    }

    boolean botZeroing = false;
    boolean botWasZeroing = botZeroing;

    public void leftTrigger2(double amount) {
        if (amount > 0.9) {
            bot.setSlideMinMax(-1000, 4000);
            botZeroing = true;
        }
        else bot.setSlideMinMax(0, 3900);
        if (!botZeroing && botWasZeroing)  {
            bot.zeroSlidePos();
        }
        botWasZeroing = botZeroing;
    }
    boolean coneStackPresets = false;

    @Override
    public void rightTrigger2() {
        coneStackPresets = true;
    }

    @Override
    public void buttonX2() {
        clawOpen = !clawOpen;
        sinceClawMove.reset();

    }


    ElapsedTime sinceWristRot = new ElapsedTime();
    ElapsedTime sinceClawMove = new ElapsedTime();
    ElapsedTime sinceArmMove = new ElapsedTime();
    ElapsedTime bigMove = new ElapsedTime();
    boolean wristPlace = false;
    boolean queueBigMove = false;
    @Override
    public void buttonB2() {
        wristPlace = !wristPlace;
        sinceWristRot.reset();
        if (bot.getSlideHeight() < Heights.flipMin) {
            TARGET_SLIDE = true;
            bot.setSlideTarget(Heights.flipMin);
        }
    }

    boolean clawOpen = false;
    @Override
    public void buttonA2() {
        clawOpen = !clawOpen;
        sinceClawMove.reset();
        if (clawOpen && armPlace) queueBigMove = true;
    }

    @Override
    public void buttonY2() {
        queueBigMove = false;
        if (clawOpen){
            clawOpen = false;
            sinceClawMove.reset();
        }
        if (bot.getSlideHeight() < Heights.flipMin) {
            TARGET_SLIDE = true;
            bot.setSlideTarget(Heights.flipMin);
        }

        wristPlace = !wristPlace;
        armPlace = !armPlace;

        sinceWristRot.reset();
        sinceArmMove.reset();

        bigMove.reset();


    }
    boolean wristRoting = false;
    boolean clawMoving = false;
    boolean armMoving = false;
    boolean bigMovin = false;
    @Override
    public void checkSpecialControls() {

        wristRoting = sinceWristRot.milliseconds() < 650;
        clawMoving = sinceClawMove.milliseconds() < 650;
        if (!clawMoving && queueBigMove) buttonY2();
        armMoving = sinceArmMove.milliseconds() < 650;


        if (!clawMoving) {
            if (armPlace && !slantArm) bot.setArmTarget(Heights.levelArmPlace);
            else if (armPlace) bot.setArmTarget(Heights.upSlantArmPlace);
            else if (slantArm) bot.setArmTarget(Heights.lowArmPickup);
            else if (coneStackPresets) bot.setArmTarget(Heights.levelArmPickup);
            else bot.setArmTarget(Heights.lowArmPickup);
        }

        if (clawOpen) bot.setClawTarget(Heights.clawOpen);
        else bot.setClawTarget(Heights.clawClosedNoStrain);

        if (!armMoving || clawMoving) {
            if (wristPlace) bot.setWristTarget(Heights.levelWristPlace);
            else bot.setWristTarget(Heights.levelWristPickup);
        }



    }

    boolean A2_DOWN = false;
    boolean B2_DOWN = false;
    boolean UP2_DOWN = false;
    boolean LEFT2_DOWN = false;
    boolean RIGHT2_DOWN = false;
    boolean RB2_DOWN = false;
    boolean LB2_DOWN = false;
    boolean X2_DOWN = false;
    boolean Y2_DOWN = false;
    boolean DOWN2_DOWN = false;
    @Override
    public void checkButtons() {
        if (gamepad2.a && !A2_DOWN) buttonA2();
        if (gamepad2.b && !B2_DOWN) buttonB2();
        if (gamepad2.x && !X2_DOWN) buttonX2();
        if (gamepad2.y && !Y2_DOWN) buttonY2();
        if (gamepad1.a) buttonA();
        if (gamepad1.b) buttonB();
        if (gamepad1.x) buttonX();
        if (gamepad1.y) buttonY();
        if (gamepad2.dpad_down && !DOWN2_DOWN) dpadDown2();
        if (gamepad2.dpad_up && !UP2_DOWN) dpadUp2();
        if (gamepad2.dpad_left && !LEFT2_DOWN) dpadLeft2();
        if (gamepad2.dpad_right && !RIGHT2_DOWN) dpadRight2();
        if (gamepad1.dpad_down) dpadDown();
        if (gamepad1.dpad_up) dpadUp();
        if (gamepad1.dpad_left) dpadLeft();
        if (gamepad1.dpad_right) dpadRight();

        if (gamepad2.left_bumper && !LB2_DOWN) leftBumper2();
        if (gamepad2.right_bumper && !RB2_DOWN) rightBumper2();
        leftTrigger2(gamepad2.left_trigger);

        if (gamepad2.right_trigger > 0) rightTrigger2();
        else coneStackPresets = false;

        A2_DOWN = gamepad2.a;
        B2_DOWN = gamepad2.b;
        X2_DOWN = gamepad2.x;
        Y2_DOWN = gamepad2.y;

        RB2_DOWN = gamepad2.right_bumper;
        LB2_DOWN = gamepad2.left_bumper;

        UP2_DOWN = gamepad2.dpad_up;
        RIGHT2_DOWN = gamepad2.dpad_right;
        DOWN2_DOWN = gamepad2.dpad_left;
        LEFT2_DOWN = gamepad2.dpad_left;
    }



    @Override
    public void checkJoysticks() {
        bot.tick();

        bot.motorDriveXYVectors((armPlace?1:-1) * gamepad1.left_stick_x, (armPlace?-1:1) * gamepad1.left_stick_y,-1 * gamepad1.right_stick_x);

        if (!TARGET_SLIDE || gamepad2.left_stick_y != 0) bot.driveSlides(-gamepad2.left_stick_y);
        if (gamepad2.left_stick_y != 0) TARGET_SLIDE = false;
    }

}
