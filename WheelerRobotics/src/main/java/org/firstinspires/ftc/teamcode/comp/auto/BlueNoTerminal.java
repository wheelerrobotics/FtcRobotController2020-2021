package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;
import org.firstinspires.ftc.teamcode.comp.utility.Pose;

@Autonomous
@Config
public class BlueNoTerminal extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    Frant bot = null;
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public int currentMovementID = 0;
    @Override
    public void runOpMode() {
        bot = new Frant();
        bot.init(hardwareMap);
        bot.autoinit();
        waitForStart();


        int conePosition = 0;
        ElapsedTime cooldown = new ElapsedTime();
        while (opModeIsActive() && conePosition == 0 && cooldown.milliseconds() < 3000) conePosition = bot.getPrincipalTag();
        cooldown.reset();
        currentMovementID = 0;
        bot.pt.pose = new Pose(0, 0, 0);
        while (opModeIsActive()) {
            bot.setPIDActive(true);


            if (currentMovementID == 0) {
                bot.pidDrive(0, 52, 0);}

            if (currentMovementID == 1) {
                bot.pidDrive(((conePosition == 1) ? 50 : ((conePosition == 2) ? 0 : -50)), 53, 0);
            }

            if (currentMovementID == 4) {

                break;
            }

            if (bot.isDone()[0] == 1 && bot.isDone()[1] == 1 && bot.isDone()[2] == 1 && cooldown.milliseconds() > 500){
                currentMovementID++;
                cooldown.reset();
            }


            telemetry.addData("x", bot.getPose().x);
            telemetry.addData("y", bot.getPose().y);
            telemetry.update();




        }
        bot.setPIDTActive(false);
        bot.setPIDActive(false);

        bot.pt.interrupt();

    }


}
