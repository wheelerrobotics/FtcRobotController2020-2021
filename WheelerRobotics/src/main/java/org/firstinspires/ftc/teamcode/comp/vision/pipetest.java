package org.firstinspires.ftc.teamcode.comp.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.vision.pipelines.SuperCoolPipeline;

@TeleOp
public class pipetest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BotVision bv = new BotVision();
        bv.init(hardwareMap, new SuperCoolPipeline());

        waitForStart();
        while (opModeIsActive()){
            // stall
        }
    }
}
