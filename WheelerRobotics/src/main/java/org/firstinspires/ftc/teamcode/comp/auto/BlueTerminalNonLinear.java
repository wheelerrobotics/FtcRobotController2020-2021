package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;

@Autonomous
@Config
public class BlueTerminalNonLinear extends OpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    Frant bot = null;
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public int currentMovementID = 0;
    public int conePosition = 0;
    ElapsedTime cooldown = new ElapsedTime();

    @Override
    public void init() {
        bot = new Frant();
        bot.init(hardwareMap);
        bot.autoinit();
    }
    public void start() {
        cooldown.reset();
        conePosition = 0;
        while (conePosition == 0 && cooldown.milliseconds() < 3000) conePosition = bot.getPrincipalTag();
        cooldown.reset();
        currentMovementID = 0;
    }
    @Override
    public void loop() {
        if (!bot.done()) return;
        bot.setPIDActive(true);

        if (currentMovementID == 0) {
            bot.pidDrive(-45, 0, 0);}

        if (currentMovementID == 1) {
            bot.pidDrive(5, 0, 0);}

        if (currentMovementID == 2) {
            bot.pidDrive(5, 50, 0);}


        if (currentMovementID == 3) {
            bot.pidDrive(((conePosition == 1) ? 55 : ((conePosition == 2) ? 0 : -50)), 54, 0);
        }


        telemetry.addData("x", bot.getPose().x);
        telemetry.addData("y", bot.getPose().y);
        telemetry.update();

        if (currentMovementID == 4) {

            this.stop();
        }

        if (bot.isDone()[0] == 1 && bot.isDone()[1] == 1 && bot.isDone()[2] == 1 && cooldown.milliseconds() > 500){
            currentMovementID++;
            cooldown.reset();
        }

    }

    @Override
    public void stop() {
        super.stop();
        bot.setPIDTActive(false);
        bot.setPIDActive(false);
        bot.pt.interrupt();

    }
}
