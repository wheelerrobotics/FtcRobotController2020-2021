package org.firstinspires.ftc.teamcode.old.lib;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.wheelerschool.robotics.old.lib.CompBot;

@Autonomous
public class EncoderTranslateExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompBot bot = new CompBot(hardwareMap);
        BotNav nav = new BotNav(bot);

        nav.activate();

        waitForStart();

        bot.setDriveEncTranslate(0.5f, 0, 10000);

        while (opModeIsActive() && !bot.atDriveTarget()) {
            telemetry.addData("Status", "driving");
            telemetry.update();
        }
    }
}
