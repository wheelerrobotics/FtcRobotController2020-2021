package org.firstinspires.ftc.teamcode.comp.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="bv")
public class supaAuto extends LinearOpMode {
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        bv.init(hardwareMap);
        while(opModeIsActive()){
        }
    }
}
