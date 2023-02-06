package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@Autonomous
@Config
public class RedTerminal extends LinearOpMode {
    public static double conePosition = 0;
    Odo bot = new Odo();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public int currentMovementID = 0;
    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        bot.autoinit();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setConstant(0xFF0000);
        }
        waitForStart();


        //int conePosition = 0;
        ElapsedTime cooldown = new ElapsedTime();
        //while (opModeIsActive() && conePosition == 0 && cooldown.milliseconds() < 3000) conePosition = bot.getPrincipalTag();
        cooldown.reset();
        while (opModeIsActive()) {
            bot.pidActive = true;


            if (currentMovementID == 0) bot.pidDrive(0, 24, 0);
            if (currentMovementID == 1) bot.pidDrive(0, 64, 1.57);
            if (currentMovementID == 2) bot.pidDrive(0, 51, 1.57);
            if (currentMovementID == 3) bot.pidDrive(27, 51, 1.57);
            if (currentMovementID == 4) bot.pidDrive(0, 51, 1.57);
            if (currentMovementID == 5) bot.pidDrive(0, 64, 1.57);
            if (currentMovementID == 6) bot.pidDrive(0, 27, 1.57);
            if (currentMovementID == 7) bot.pidDrive((conePosition == 1 ? -24 : (conePosition == 2 ? 0 : 24)), 27, 1.57);
            if (currentMovementID == 8) break;

            if (bot.isDone()[0] == 1 && bot.isDone()[1] == 1 && bot.isDone()[2] == 1 && cooldown.milliseconds() > 500){
                currentMovementID++;
                cooldown.reset();
            }

            bot.tickPID();


        }
        bot.opModeIsActive = false;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setConstant(0x9900BB);
        }


    }


}
