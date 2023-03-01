package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;

@Autonomous
@Config
public class FakeAuto extends LinearOpMode {
    Lenny bot = new Lenny();

    public void runOpMode() {
        bot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("dost", bot.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }


    }

}
