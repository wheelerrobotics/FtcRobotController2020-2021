package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@Autonomous
@Config
public class RealAuto extends LinearOpMode {
    public static double x = 50;
    public static double y = 10;
    Odo bot = new Odo();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);

        waitForStart();

        int conePosition = bot.getPrincipalTag();
        tele.addData("Cone Position", conePosition );
        tele.update();
        bot.pidDrive(x, y);

    }
}
