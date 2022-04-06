package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;

@Config
@Autonomous
public class nto extends LinearOpMode {
    public static double AMP = 10;
    public static double PH = 90;
    public static double FREQ = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard db = FtcDashboard.getInstance();
        telemetry = db.getTelemetry();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            telemetry.addData("x", AMP * Math.sin(2*Math.PI * FREQ * getRuntime() + Math.toRadians(PH)));
            telemetry.update();
        }

    }
}
