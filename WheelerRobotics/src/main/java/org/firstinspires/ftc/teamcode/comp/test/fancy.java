package org.firstinspires.ftc.teamcode.comp.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.nav;

@Autonomous
public class fancy extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        nav navi = new nav();
        waitForStart();
        while (opModeIsActive()){
            navi.doTheThing(100, 100, 90);
            break;
        }

    }

}
