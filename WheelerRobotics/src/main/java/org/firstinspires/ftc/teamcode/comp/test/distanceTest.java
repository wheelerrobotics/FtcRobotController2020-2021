package org.firstinspires.ftc.teamcode.comp.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@TeleOp
public class distanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Meccanum m = new Meccanum();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        m.init(hardwareMap);
        DistanceSensor db = m.distanceBack;
        DistanceSensor dl = m.distanceLeft;
        DistanceSensor dr = m.distanceRight;

        waitForStart();

        while (opModeIsActive()){

            if(db.getDistance(DistanceUnit.MM) < 8000) tele.addData("db", db.getDistance(DistanceUnit.MM));
            if(dl.getDistance(DistanceUnit.MM) < 8000) tele.addData("dl", dl.getDistance(DistanceUnit.MM));
            if(dr.getDistance(DistanceUnit.MM) < 8000) tele.addData("dr", dr.getDistance(DistanceUnit.MM));
            tele.update();
        }
    }
}
