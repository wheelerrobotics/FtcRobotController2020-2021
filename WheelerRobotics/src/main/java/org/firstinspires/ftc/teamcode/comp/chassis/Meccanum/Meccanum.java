package org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;


import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.chassis.Chassis;
import org.firstinspires.ftc.teamcode.comp.helpers.Distances;
import org.firstinspires.ftc.teamcode.comp.helpers.PID;

// robot driving and motion class

public class Meccanum implements Chassis {

    double[] left = {
            1,  -1,
            -1,  1
    };
    double[] back = {
            -1, -1,
            -1, -1
    };
    double[] clock = {
            -1,  1,
            -1,  1
    };

    protected BNO055IMU imu = null;
    protected DistanceSensor distanceBack = null;
    protected DistanceSensor distanceFront = null;
    protected DistanceSensor distanceLeft = null;
    protected DistanceSensor distanceRight = null;

    protected DcMotor motorFrontRight = null;
    protected DcMotor motorBackRight = null;
    protected DcMotor motorFrontLeft = null;
    protected DcMotor motorBackLeft = null;

    HardwareMap hw = null;
    public void init(HardwareMap hardwareMap){
        hw = hardwareMap;
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //distace sensors (unused for now)


        distanceBack = tryDeclareDistanceSensor("distanceBack", hw);
        distanceRight = tryDeclareDistanceSensor("distanceRight", hw);
        distanceLeft = tryDeclareDistanceSensor("distanceLeft", hw);
        distanceFront = tryDeclareDistanceSensor("distanceFront", hw);

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;
    }

    // SUPPORT METHODS
    public void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        // drive the motors at custom powers for each
        // used for every other drive class

        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    public void motorDriveXYVectors(double xvec, double yvec, double spinvec){
        // this class drives the robot in the direction of vectors from a joystick and a spin value
        // used for teleop mode driving wheels with joysticks


        double y = -pow(yvec,1); // Remember, this is reversed!
        double x = pow(xvec * 1.1,1); // Counteract imperfect strafing
        double rx = pow(spinvec,1);


        //denominator is the largest motor power (absolute value) or 1
        //this ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorDrive(frontLeftPower,backLeftPower, frontRightPower, backRightPower);

    }

    /**
     * Runs pid and updates the motors to keep the robot some distance/rotation from walls and stuff.
     * @param l the distance from the left distance sensor to try to maintain.
     * @param b the distance from the back distance sensor to try to maintain.
     * @param r the distance from the right distance sensor to try to maintain.
     * @param breakTime the time until the funtion should timeout (if it doesn't reach the position its going for)
     * @param scale a scalar for the speed of the motors.
     */
    public void doTheThing(double l, double b, double r, double breakTime, double scale) {
        ElapsedTime et = new ElapsedTime();
        et.reset();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        double thresh = 0.1;
        double rthresh = 0.1;
        double dl = distanceLeft.getDistance(DistanceUnit.CM);
        double db = distanceBack.getDistance(DistanceUnit.CM);
        double dri = distanceRight.getDistance(DistanceUnit.CM);
        double dr = getAngles().firstAngle;
        PID pb = new PID(-0.025, 0 ,0); // -0.025, -0.00008, -0.2
        pb.init(db);
        pb.setTarget(b);

        PID pl = new PID(-0.030, 0 ,0);
        pl.init(dl);
        pl.setTarget(l);

        /*
        Pidata pri = new Pidata(-0.025, 0 ,0);
        pri.init(dri);
        pri.setTarget(ri);
        */


        PID pr = new PID(0.005, 0, 0);
        pr.init(dr);
        pr.setTarget(r);
        double dthresh = 0.05;



        ElapsedTime start = new ElapsedTime();
        start.reset();
        while (true){
            //motorDriveForwardEncoded(0.5, 100);
            if (dl < 800) tele.addData("dl", dl);
            if (db < 800) tele.addData("db", db);
            if (dr < 800) tele.addData("dr", dr);
            //if (dri < 800) tele.addData("dri", dri);

            tele.addData("sl", pl.getDerivative());
            tele.addData("sb", pb.getDerivative());
            tele.addData("sr", pr.getDerivative());
            //            tele.addData("sri", pri.getDerivative());

            double[] out = {0, 0, 0, 0};
            //if(Math.random() > 0.6) camServo.setPosition(Math.random()); // really does nothing, but DONT DELETE
            double el = -pl.tick(dl);
            double er = pr.tick(dr); // error rotation
            double eb = pb.tick(db);
            //double eri = (ri == -1) ? pri.tick(dri) : 0;

            tele.addData("el", el);
            tele.addData("eb", eb);
            tele.addData("er", er);
            //tele.addData("eri", eri);

            for (int i = 0; i<out.length; i++) { // add the individual vectors
                out[i] = el * this.left[i] + eb * this.back[i] + er * this.clock[i]; //+ eri * -this.left[i];
            }

            double abc = absmac(out); // get max value for scaling
            if (abc > 1){
                for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                    out[i] /= abs(abc);
                }
            }

            for (int i = 0; i<out.length; i++){ // scale down
                out[i] *= scale;
                //if (out[i] < 0.05) out[i] = 0;
            }
            driveVector(out);

            // update vals
            dl = distanceLeft.getDistance(DistanceUnit.CM);
            db = distanceBack.getDistance(DistanceUnit.CM);
            //dri = distanceRight.getDistance(DistanceUnit.CM);
            dr = getAngles().firstAngle;

            tele.addData("o", out);
            tele.update();

            if(
                //abs(eri) < thresh &&
                    abs(eb) < thresh &&
                            abs(el) < thresh &&
                            abs(er) < rthresh &&
                            abs(pb.getDerivative()) > dthresh &&
                            //abs(pri.getDerivative()) > dthresh &&
                            abs(pl.getDerivative()) > dthresh &&
                            abs(pr.getDerivative()) > dthresh
            ) break;

            if(start.milliseconds() > breakTime) break;
        }

        motorStop();

    }

    /**
     * Runs pid and updates the motors to keep the robot some distance/rotation from walls and stuff.
     * @param ri th rotation heading to try to maintain.
     * @param b the distance from the back distance sensor to try to maintain.
     * @param r the distance from the right distance sensor to try to maintain.
     * @param breakTime the time until the funtion should timeout (if it doesn't reach the position its going for)
     * @param scale a scalar for the speed of the motors.
     */
    public void doTheThingy(double ri, double b, double r, double breakTime, double scale) {
        ElapsedTime et = new ElapsedTime();
        et.reset();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        double thresh = 0.1;
        double rthresh = 0.1;
        double dri = distanceRight.getDistance(DistanceUnit.CM);
        double db = distanceBack.getDistance(DistanceUnit.CM);
        double dr = getAngles().firstAngle;
        PID pb = new PID(-0.025, 0 ,0); // -0.025, -0.00008, -0.2
        pb.init(db);
        pb.setTarget(b);

        PID pri = new PID(-0.035, 0 ,0);
        pri.init(dri);
        pri.setTarget(ri);

        /*
        Pidata pri = new Pidata(-0.025, 0 ,0);
        pri.init(dri);
        pri.setTarget(ri);
        */


        PID pr = new PID(0.005, 0, 0);
        pr.init(dr);
        pr.setTarget(r);
        double dthresh = 0.05;



        ElapsedTime start = new ElapsedTime();
        start.reset();
        while (true){
            //motorDriveForwardEncoded(0.5, 100);
            if (dri < 800) tele.addData("dri", dri);
            if (db < 800) tele.addData("db", db);
            if (dr < 800) tele.addData("dr", dr);
            //if (dri < 800) tele.addData("dri", dri);

            tele.addData("sri", pri.getDerivative());
            tele.addData("sb", pb.getDerivative());
            tele.addData("sr", pr.getDerivative());
            //            tele.addData("sri", pri.getDerivative());

            double[] out = {0, 0, 0, 0};
            //if(Math.random() > 0.6) camServo.setPosition(Math.random()); // really does nothing, but DONT DELETE
            double eri = -pri.tick(dri);
            double er = pr.tick(dr); // error rotation
            double eb = pb.tick(db);
            //double eri = (ri == -1) ? pri.tick(dri) : 0;

            tele.addData("el", eri);
            tele.addData("eb", eb);
            tele.addData("er", er);
            //tele.addData("eri", eri);

            for (int i = 0; i<out.length; i++) { // add the individual vectors
                out[i] = eri * -this.left[i] + eb * this.back[i] + er * this.clock[i]; //+ eri * -this.left[i];
            }

            double abc = absmac(out); // get max value for scaling
            if (abc > 1){
                for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                    out[i] /= abs(abc);
                }
            }


            for (int i = 0; i<out.length; i++){ // scale down
                out[i] *= scale;
                //if (out[i] < 0.05) out[i] = 0;
            }
            driveVector(out);

            // update vals
            dri = distanceRight.getDistance(DistanceUnit.CM);
            db = distanceBack.getDistance(DistanceUnit.CM);
            //dri = distanceRight.getDistance(DistanceUnit.CM);
            dr = getAngles().firstAngle;

            tele.addData("o", out);
            tele.update();

            if(
                //abs(eri) < thresh &&
                    abs(eb) < thresh &&
                            abs(eri) < thresh &&
                            abs(er) < rthresh &&
                            abs(pb.getDerivative()) > dthresh &&
                            //abs(pri.getDerivative()) > dthresh &&
                            abs(pri.getDerivative()) > dthresh &&
                            abs(pr.getDerivative()) > dthresh
            ) break;

            if(start.milliseconds() > breakTime) break;
        }

        motorStop();

    }

    /**
     * Takes a 1x4 array representing wheel speeds from -1 to 1 and runs the motors at those powers.
     * @param arr 1x4 array which serves as a grid for the motor wheels.
     */
    void driveVector(double[] arr){
        motorFrontLeft.setPower(arr[0]);
        motorFrontRight.setPower(arr[1]);
        motorBackLeft.setPower(arr[2]);
        motorBackRight.setPower(arr[3]);
    }
    /**
     * Takes an array and returns the value furthest from 0.
     * @param arr the array to act upon.
     * @return The absolute maximum of the array.
     */
    double absmac(double[] arr){

        int outi = 0; // index of max
        for (int i = 0; i<arr.length; i++){
            if( abs(arr[i]) > abs(arr[outi])){
                outi = i;
            }
        }
        return arr[outi];

    }
    /**
     * Stops all robot motors
     */
    public void motorStop(){
        // stop all the motors
        // used at the end of all movement functions
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }

    /**
     * Gets an orientation which reports all robot rotation
     * @return Orientation object with angle unit degrees
     */
    public Orientation getAngles() {
        // gets current angle on field [-180, 180]               i just used interval notation :)
        // useful for angle methods
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Gets a normalized (0-360) rotation of the robot on the field
     * @return rotation of robot
     */
    public double getAngle360() {
        // gets normalized angle on field [0, 360]              i just used interval notation :)
        // useful for angle methods
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + 360;
    }
    public Distances getDistances(DistanceUnit unit){
        return new Distances(
                distanceLeft.getDistance(unit),
                distanceRight.getDistance(unit),
                distanceBack.getDistance(unit),
                distanceFront.getDistance(unit)
        );
    }
}
