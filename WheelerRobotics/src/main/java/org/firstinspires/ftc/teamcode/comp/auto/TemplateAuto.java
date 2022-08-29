package org.firstinspires.ftc.teamcode.comp.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.ColorIsolationPipeline;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.DummyCVPipeline;

@Autonomous( name = "Warehouse Blue Nav")
public class TemplateAuto extends LinearOpMode {
    // for non next to caurousel
    Odo bot = new Odo();
    BotVision bv = new BotVision();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();

    public void runOpMode() {
        bot.init(hardwareMap);
        bv.init(hardwareMap, new DummyCVPipeline());

        waitForStart();
        executeAutomaticSequence1();

    }

    private void executeAutomaticSequence1() {
        // make the robot score points :)

    }

}
