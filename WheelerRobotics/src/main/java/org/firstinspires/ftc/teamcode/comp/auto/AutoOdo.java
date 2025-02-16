package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@Config
@TeleOp(name="Griffin's Muddy Buddies")
@Disabled
public class AutoOdo extends LinearOpMode {
    Odo bot = new Odo();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public static double p = 0;
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        waitForStart();
        bot.opModeIsActive = false;
        while (opModeIsActive()){
            bot.motorDrive(p,p,p,p);
        }
    }
}
