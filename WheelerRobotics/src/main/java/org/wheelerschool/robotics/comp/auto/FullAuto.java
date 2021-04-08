package org.wheelerschool.robotics.comp.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.comp.CompBot;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous
public class FullAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    CompBot bot;

    private void barrage(int attempts) {
        for (int i=0; i<attempts; i++) {
            bot.launchPush(true);
            runtime.reset();
            while (runtime.seconds() < 1 && opModeIsActive()) {}
            bot.launchPush(false);
            runtime.reset();
            while (runtime.seconds() < 1 && opModeIsActive()) {}
        }
    }

    private void jiggle() {
        runtime.reset();
        while (runtime.seconds() < 1) {
            bot.jiggle();
        }
        bot.setDriveDirect(0,0,0,0);
    }

    private void encoderWait() {
        while (!bot.atDriveTarget() && opModeIsActive()) {}
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new CompBot(hardwareMap);
        BotNav nav = new BotNav(bot);

        nav.activate();

        String ringStatus = null;
        while (!isStarted()) {
            String newStatus = nav.botVis.ringDetect();
            if (newStatus != null) {
                ringStatus = newStatus;
            }
            telemetry.addData("Ring Status", ringStatus);
            telemetry.update();
        }

        bot.setDriveEncTranslate(0.75f, 0, 500);

        while (!bot.atDriveTarget() && opModeIsActive()) {

        }

        bot.setDriveEncTranslate(0.75f, 1300, 0);

        while (!bot.atDriveTarget() && opModeIsActive()) {

        }

        bot.setDriveEncRotate(0.5f, -300);

        while (!bot.atDriveTarget() && opModeIsActive()) {

        }

        bot.launchLeft.setPower(0.8f);
        bot.launchRight.setPower(0.9f);

        bot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(
                !nav.moveTowardsTarget(new VectorF(-200, 1100, 0),
                new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) (Math.PI/2), 0))
                && opModeIsActive()
        ) {}

        bot.setDriveDirect(0,0,0,0);

        barrage(1);
        jiggle();

        barrage(1);
        jiggle();

        barrage(1);

        if (ringStatus == null) {
            while(
                    !nav.moveTowardsTarget(new VectorF(600, 1000, 0),
                            new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) (Math.PI/2), 0))
                            && opModeIsActive()
            ) {}
            bot.setDriveEncRotate(1.f, 3000);
            encoderWait();
        } else if  (ringStatus.equals("Single")) {
            while(
                    !nav.moveTowardsTarget(new VectorF(600, 1000, 0),
                            new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) (Math.PI/2), 0))
                            && opModeIsActive()
            ) {}
            bot.setDriveEncRotate(1.f, 1500);
            encoderWait();
        } else if  (ringStatus.equals("Quad")) {
            while(
                    !nav.moveTowardsTarget(new VectorF(1600, 1000, 0),
                            new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) (Math.PI/2), 0))
                            && opModeIsActive()
            ) {}
            bot.setDriveEncRotate(1.f, 3000);
            encoderWait();
        }

        bot.setWobbleArm(CompBot.WobblePosition.GRAB);
        while (bot.wobbleArm.isBusy() && opModeIsActive()) {}
        bot.setWobbleGrab(false);
        //bot.setWobbleArm(CompBot.WobblePosition.UP);

        /*
        while(
                !nav.moveTowardsTarget(new VectorF(500, 1100, 0),
                        new Orientation(EXTRINSIC, XYZ, RADIANS, 0, 0, (float) (Math.PI/2), 0))
                        && opModeIsActive()
        ) {}
         */

        bot.setDriveDirect(0,0,0,0);
    }
}
