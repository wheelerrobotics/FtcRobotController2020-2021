package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.DummyCVPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name="Griffin's Muddy Buddies")
public class AutoOdo extends LinearOpMode {
    Odo bot = new Odo();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            //int conePosition = bot.getPrincipalTag();
            tele.addData("Cone Position", bot.detections );
            tele.update();
        }
    }
}
