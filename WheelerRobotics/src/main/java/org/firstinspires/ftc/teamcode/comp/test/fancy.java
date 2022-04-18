package org.firstinspires.ftc.teamcode.comp.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.chassis.nav;

@TeleOp
@Disabled
public class fancy extends LinearOpMode {
    public static double firstL = 5;
    public static double firstB = 5;
    public static double secondL = 65;
    public static double secondB = 5;
    public static double firstR = 0;
    public static double secondR = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        nav navi = new nav();
        navi.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            break;
        }

    }

}
