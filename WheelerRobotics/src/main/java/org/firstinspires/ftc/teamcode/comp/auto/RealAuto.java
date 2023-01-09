package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;

@Autonomous
@Disabled
@Config
public class RealAuto extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    Frant bot = new Frant();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public static int currentMovementID = 0;
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);

        waitForStart();


        int conePosition = 0;
        while (opModeIsActive() && conePosition == 0) conePosition = bot.getPrincipalTag();
        ElapsedTime cooldown = new ElapsedTime();
        cooldown.reset();
        while (opModeIsActive()) {
            bot.pidActive = true;


            if (currentMovementID == 0) {
                bot.pidDrive(47, 0, 0);}

            if (currentMovementID == 1) {
                bot.pidDrive(47, 100, 0);}

            if (currentMovementID == 2) {
                bot.pidDrive(((conePosition == 1) ? 48 : ((conePosition == 2) ? 0 : -48)), 100, 0);
            }


            if (currentMovementID == 3) break;

            if (bot.isDone()[0] == 1 && bot.isDone()[1] == 1 && bot.isDone()[2] == 1 && cooldown.milliseconds() > 500){
                currentMovementID++;
                cooldown.reset();
            }




        }
        bot.pidActive = false;
        bot.opModeIsActive = false;


    }


}
