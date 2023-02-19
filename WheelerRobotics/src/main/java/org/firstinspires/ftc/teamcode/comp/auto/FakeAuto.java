package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;

@Autonomous
@Config
public class FakeAuto extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double r = 0;
    public int currentMovementID = 0;
    public static boolean movement = true;
    Lenny bot = new Lenny();

    public void runOpMode() {
        bot.init(hardwareMap);
        bot.slideinit();

        waitForStart();
        ElapsedTime cooldown = new ElapsedTime();

        cooldown.reset();
        while (opModeIsActive()) {
            bot.tick();
        }


    }

}
