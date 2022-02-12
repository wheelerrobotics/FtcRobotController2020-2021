package org.firstinspires.ftc.teamcode.comp.test.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

@Disabled
public class FieldPosTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dash = FtcDashboard.getInstance();

        SensorREV2mDistance distanceBack = hardwareMap.get(SensorREV2mDistance.class, "distanceBack");
        SensorREV2mDistance distanceRight = hardwareMap.get(SensorREV2mDistance.class, "distanceRight");
        SensorREV2mDistance distanceLeft = hardwareMap.get(SensorREV2mDistance.class, "distanceLeft");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double x = 0;
        double y = 0;

        waitForStart();
        while(opModeIsActive()){



            updateFieldPos(x, y, dash);
        }
    }
    public void updateFieldPos(double x, double y, FtcDashboard dashboard){
        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().fillCircle(x, y, 10);

        dashboard.sendTelemetryPacket(packet);

    }
}
