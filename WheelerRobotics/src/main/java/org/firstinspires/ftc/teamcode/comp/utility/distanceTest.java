package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

@TeleOp
@Disabled
public class distanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odo m = new Odo();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        m.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            tele.addData("back", m.getDistances(DistanceUnit.MM).back);
            tele.addData("left", m.getDistances(DistanceUnit.MM).left);
            tele.addData("right", m.getDistances(DistanceUnit.MM).right);
            tele.addData("front", m.getDistances(DistanceUnit.MM).front);
            tele.update();
        }
    }
}
