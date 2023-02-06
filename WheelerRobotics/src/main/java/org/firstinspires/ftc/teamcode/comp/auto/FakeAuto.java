package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@Autonomous
@Config
public class FakeAuto extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    public int currentMovementID = 0;
    public static boolean movement = true;
    Odo bot = new Odo();

    public void runOpMode() {
        bot.init(hardwareMap);
        bot.setMovement(movement);

        waitForStart();
        ElapsedTime cooldown = new ElapsedTime();

        cooldown.reset();
        while (opModeIsActive()) {
            bot.setMovement(movement);
            bot.pidActive = true;
            if (currentMovementID % 2 == 0) bot.pidDrive(0, 0, 0);
            if (currentMovementID % 2 == 1) bot.pidDrive(x, y, r);

            if (bot.isDone()[0] == 1 && bot.isDone()[1] == 1 && bot.isDone()[2] == 1 && cooldown.milliseconds() > 500){
                currentMovementID++;
                cooldown.reset();
                while (cooldown.milliseconds() < 500);
                cooldown.reset();
            }
            bot.tickPID();

        }


    }

}
