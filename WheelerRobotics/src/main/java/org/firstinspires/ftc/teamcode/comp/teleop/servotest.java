package org.firstinspires.ftc.teamcode.comp.teleop;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.controller.Odo.ControllerMapOdo;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.DummyCVPipeline;

@Disabled
@TeleOp
public class servotest extends LinearOpMode {
    // Declare OpMode members.
    Odo bot = new Odo();
    ControllerMapOdo cm = new ControllerMapOdo();
    BotVision bv = new BotVision();

    FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        bv.init(hardwareMap, new DummyCVPipeline());
        // internal IMU setup
        bot.init(hardwareMap);

        cm.init(bot, gamepad1, gamepad2);

        int startupID = hardwareMap.appContext.getResources().getIdentifier("startup", "raw", hardwareMap.appContext.getPackageName());
        Context appContext = hardwareMap.appContext;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        SoundPlayer.getInstance().startPlaying(appContext, startupID);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            cm.checkControls();

            telemetry.update();
        }

    }
}
