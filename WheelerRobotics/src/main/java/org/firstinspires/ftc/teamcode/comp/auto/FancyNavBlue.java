package org.firstinspires.ftc.teamcode.comp.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous
public class FancyNavBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Meccanum mec = new Meccanum();

        mec.opModeOn(opModeIsActive());
    }

}
