package org.firstinspires.ftc.teamcode.comp.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous
public class AutoNav5 extends LinearOpMode {
    // for next to carousel
    Meccanum meccanum = new Meccanum();

    @Override
    public void runOpMode() throws InterruptedException {
        meccanum.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence2();

    }

    private void executeAutomaticSequence2() {
        // test autos


    }

}
