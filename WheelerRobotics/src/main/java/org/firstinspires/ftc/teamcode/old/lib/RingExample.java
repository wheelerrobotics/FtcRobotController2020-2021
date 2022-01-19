package org.firstinspires.ftc.teamcode.old.lib;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous
public class RingExample extends OpMode {
    org.wheelerschool.robotics.old.lib.CompBot bot;
    BotNav nav;

    @Override
    public void init() {
        bot = new org.wheelerschool.robotics.old.lib.CompBot(hardwareMap);
        nav = new BotNav(bot);
    }

    @Override
    public void start() {
        nav.activate();
    }

    @Override
    public void loop() {
        telemetry.addData("rings", nav.botVis.ringDetect());
    }
}
