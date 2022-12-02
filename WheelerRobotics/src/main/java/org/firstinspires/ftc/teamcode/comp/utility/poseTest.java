package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@TeleOp
@Disabled
@Config
public class poseTest extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    Odo bot = new Odo();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            bot.pidActive = true;
            bot.pidDrive(x, y, r);


        }
        bot.opModeIsActive = false;


    }


}
