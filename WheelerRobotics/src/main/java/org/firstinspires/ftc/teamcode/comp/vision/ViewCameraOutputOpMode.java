package org.firstinspires.ftc.teamcode.comp.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="View Camera Output")
@Disabled
public class ViewCameraOutputOpMode extends LinearOpMode {
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        // simple class for viewing camera feed and testing processors
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
