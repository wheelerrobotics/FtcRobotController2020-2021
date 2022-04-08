package org.firstinspires.ftc.teamcode.comp.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.chassis.nav;

@TeleOp
public class fancy extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        nav navi = new nav();
        navi.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            navi.doTheThing(35, 35, 2);
            break;
        }

    }

}
