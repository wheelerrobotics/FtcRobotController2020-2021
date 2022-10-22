package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("test", "test");
            telemetry.update();
        }

    }
}
