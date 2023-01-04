package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;

@Autonomous
@Config
public class FakeAuto extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    Frant bot = new Frant();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        bot.autoinit();

        waitForStart();

        while (opModeIsActive()) {
            bot.pidActive = true;
            bot.pidDrive(x, y, r);
        }
        bot.opModeIsActive = false;


    }

}
