package org.firstinspires.ftc.teamcode.comp.robot.Odo;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.comp.helpers.PID;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;
import org.firstinspires.ftc.teamcode.comp.utility.Encoders;
import org.firstinspires.ftc.teamcode.comp.utility.Pose;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
public class Frant extends Meccanum implements Robot {
    protected Telemetry tele = tele = FtcDashboard.getInstance().getTelemetry();
    protected HardwareMap hw = null;
    public static double OPEN_POSITION = 0.3;
    public static double CLOSED_POSITION = 0;
    public static double ai = 0.0001;
    public static double ap = 0.1;
    public static double ad = 0.1;
    public static double xp = 0.00005;
    public static double xd = 0.0004;
    public static double yp = 0.00005;
    public static double yd = 0.0004;
    public static double rp = 0.000;
    public static double rd = 0.000;
    public static double dthresh = 0.001;
    public boolean driverOperating = true;
    public double scaleFactor = 0.5;

    public double stalPower = 0.08;
    protected Servo claw = null;
    protected DistanceSensor armHeight = null;
    protected DcMotor arm = null;

    double xTarget = 0;
    double yTarget = 0;
    double rTarget = 0;
    double aTarget = 0;


    public boolean armDone = false;
    public boolean setupDone = false;

    AprilThread at = new AprilThread();
    public ArmThread armt = new ArmThread();
    public PIDThread pt = new PIDThread();

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        // Reverse the left side motors and set behaviors to stop instead of coast

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        armHeight = hardwareMap.get(DistanceSensor.class, "armSensor");

        // define arm and servo objects and also spinner
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set prefs for arm and servo
        claw.setDirection(Servo.Direction.FORWARD);


        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;

        setupDone = true;
        runtime.reset();
    }
    public void autoinit() {

            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            pt.encoders = new Encoders(0, 0, 0);
            pt.pose = new Pose(0, 0, 0);
            pt.init();

            at.start();
    }
    public void teleinit() {
        armt.init();
        armDone = true;
    }


    public Pose getPose() {
        return pt.pose;

    }
    public void setClaw(boolean open) {
        claw.setPosition(open ? OPEN_POSITION : CLOSED_POSITION);
    }
    public boolean getClaw() {
        return claw.getPosition() == OPEN_POSITION;
    }
    public double getClawHeight() {
        return armHeight.getDistance(DistanceUnit.CM);
    }
    public int getPrincipalTag(){
        return at.getDetected();
    }
    public void armDrive(double power) {

        if (armHeight.getDistance(DistanceUnit.CM) < 5 && power < 0) { // might be < instead of >
            arm.setPower(0);
        } else {
            arm.setPower(power);
        }

    }
    public void setPIDTActive(boolean active) {
        pt.end(active);
    }
    public void setArmActive(boolean active) {
        armt.opModeIsActive = active;
    }
    public void setPIDActive(boolean active) {
        pt.pidActive = active;
    }

    public void pidDrive(double x, double y, double r) {
        xTarget = x;
        yTarget = y;
        rTarget = r;
    }
    public boolean done() {
        return armDone & setupDone;
    }

    public void playSound(String filename){
        // play a sound
        // doesnt work but would be really fun :(

        int startupID = hw.appContext.getResources().getIdentifier(filename, "raw", hw.appContext.getPackageName());
        Context appContext = hw.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
    public Orientation getAngularOrientation(){
        return imu.getAngularOrientation();
    }
    public int[] isDone() {
        return pt.isDone();
    }
    // Late night thoughts so I can continue them tmrw:
    /*
    - ideally, run the positioning stuff on a seperate thread
        - have a method to talk to that thread and get position while we do other stuff on main.
        - this is convinient because it means we dont have to worry about delay on the position resulting in more accurate measurements.
    - Look at the gm0 thing i have open to figure out how to use the encoders.
    - Would be sick if we actually use tensorflow models. Could be useful for object detection of the junctions.

     */
    private class AprilThread {// extends Thread {

        public ArrayList<AprilTagDetection> detections = new ArrayList<>();
        public BotVision bv = new BotVision();
        public AprilTagDetectionPipeline atdp =  new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);

        int numFramesWithoutDetection = 0;
        final int DECIMATION_LOW = 2;
        final int DECIMATION_HIGH = 3;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 2.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 7;

        public void start() {
            bv.init(hw, atdp);


        }
        public int getDetected(){
            return checkDetections();
        }
        public int checkDetections() {
            detections = atdp.getDetectionsUpdate();
            if (detections != null) {

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        atdp.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        atdp.setDecimation(DECIMATION_HIGH);
                    }
                }

            }

            return ((detections != null && detections.size() > 0) ? detections.get(0).id : 0);
        }
    }

    public void setTargetArmHeight(double height) {
        aTarget = height;
    }

    public class ArmThread {
        private PID arm_pid = null;
        public boolean opModeIsActive = true;
        double height = 0;

        public void init() {
            arm_pid = new PID(ap, 0, ad, true);
            arm_pid.init(height);

        }
        public void tick() {

            try {


                height = getClawHeight();
                if (height > 800) return;
                arm_pid.setConsts(ap, 0, ad);
                arm_pid.setTarget(aTarget);

                double e = arm_pid.tick(height);
                if (!driverOperating) {
                    if (arm_pid.isDone() == 1) {
                        armDrive(0);
                    }
                    armDrive(e);
                }



            }
            catch (Exception e){
                return;
            }
        }
    }
    public class PIDThread
    {


        public Encoders encoders = null;
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        public Pose pose = null;
        private PID py, px, pr = null;
        public boolean opModeIsActive = true;
        public boolean pidActive = false;
        ElapsedTime et = new ElapsedTime();
        protected double[] left = {
                1,  -1,
                -1,  1
        };
        protected double[] back = {
                -1, -1,
                -1, -1
        };
        protected double[] clock = {
                -1,  1,
                -1,  1
        };
        public PIDThread() {
            pose = new Pose(0, 0, 0);

        }
        double centere;
        double lefte;
        double righte;
        double lastCenter;
        double lastLeft;
        double lastRight;
        double dx = 0;
        double dy = 0;
        double dr = 0;
        double deltaCenter = 0;
        double deltaRight = 0;
        double deltaLeft = 0;
        boolean firstTick = true;

        public void init() {
            encoders = new Encoders(0, 0, 0);
            pose = new Pose(0, 0, 0);

            dx = 0;
            dy = 0;
            dr = 0;
            centere = -motorFrontLeft.getCurrentPosition();
            lefte = motorBackLeft.getCurrentPosition();
            righte = motorBackRight.getCurrentPosition();
            lastCenter = centere;
            lastLeft = lefte;
            lastRight = righte;

            py = new PID(yp, 0, yd, false); // don't need to correct for sensor jitter because we are using encoders
            py.init(pose.y);
            px = new PID(xp, 0, xd, false); // -0.025, -0.00008, -0.2
            px.init(pose.x);
            pr = new PID(rp, 0, rd, false); // -0.025, -0.00008, -0.2
            pr.init(pose.r);
            deltaCenter = 0;
            deltaRight = 0;
            deltaLeft = 0;

        }


        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        public void end(boolean val) {
            opModeIsActive = val;
        }
        public void tick() {
            if (!pidActive) return;
            if (!opModeIsActive) return;
                // TODO: add dimension for rotation, will involve calculating x/y with rotation
                //  in mind thus a combination of current x/y encoder readings. We want to
                //  maintain an absolute positioning system (field centric)

                double xScaler = (220787f / 240f);
                double yScaler = (220787f / 240f);
                double rScaler = (130538f / (6 * PI)) * (180/PI);
                double x = xTarget * xScaler; // in (experimentally obtained)
                double y = yTarget * yScaler; // in
                double r = rTarget * rScaler; // radians

                px.setConsts(xp, 0, xd);
                py.setConsts(yp, 0, yd);
                pr.setConsts(rp, 0, rd);
                px.setTarget(x);
                py.setTarget(y);
                pr.setTarget(r);

                tele.addData("sx", px.getDerivative());
                tele.addData("sy", py.getDerivative());
                tele.addData("sr", pr.getDerivative());

                double[] out = {0, 0, 0, 0};
                double ex = px.tick(pose.x);
                double ey = py.tick(pose.y);
                double er = pr.tick(pose.r);


                tele.addData("ex", ex);
                tele.addData("ey", ey);
                tele.addData("er", er);

                for (int i = 0; i<out.length; i++) { // add the individual vectors
                    out[i] = ex * this.left[i] + ey * this.back[i] + er * -this.clock[i];
                }

                // TODO: is there anything wrong with linear scaling (dividing by greatest value) here?
                double abc = absmac(out); // get max value for scaling
                if (abc > 1){
                    for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                        out[i] /= abs(abc);
                    }
                }


                for (int i = 0; i<out.length; i++) out[i] *= scaleFactor;
                for (int i = 0; i<out.length; i++) if (abs(out[i]) < stalPower) out[i] = 0;

                driveVector(out);

                // update vals
                updateEncoders();

                    /*
                    encoders
                               f
                     ______________________
                     |                    |
                     |         b >        |
                     |  ^      fl         |
                  r  |  a bl       br c   |  l
                     |                v   |
                     |____________________|
                                b
                          view from top
                     */

                centere = -motorFrontLeft.getCurrentPosition();
                lefte = motorBackLeft.getCurrentPosition();
                righte = -motorBackRight.getCurrentPosition();


                deltaCenter = (centere) - lastCenter;
                deltaRight = righte - lastRight;
                deltaLeft = lefte - lastLeft;
                dx = deltaCenter - (deltaRight+deltaLeft) / 2;
                dy = (deltaRight-deltaLeft) / 2;
                dr = (deltaRight+deltaLeft) / 2;
                tele.addData("dr", dr);
                tele.addData("dx", dx);
                tele.addData("dy", dy);
                lastCenter = centere;
                lastLeft = lefte;
                lastRight = righte;
                if (firstTick) {
                    firstTick = false;
                    dx = 0;
                    dy = 0;
                    dr = 0;
                    pose.setPose(0,0,0);
                }
                pose.setPose(dx + pose.x, dy + pose.y, dr + pose.r);

                tele.addData("pr", pose.r);
                tele.addData("px", pose.x);
                tele.addData("py", pose.y);

                tele.addData("rd", pr.isDone());
                tele.addData("xd", px.isDone());
                tele.addData("yd", py.isDone());


                tele.addData("encr", motorBackLeft.getCurrentPosition());
                tele.addData("encl", motorBackRight.getCurrentPosition());
                tele.addData("encc", -motorFrontLeft.getCurrentPosition());
                tele.update();


        }
        public Encoders getEncoders(){
            updateEncoders();
            return encoders;
        }
        public Pose getPose() {
            return pose;
        }
        public int[] isDone() {
            return new int[]{px.isDone(), py.isDone(), pr.isDone()};
        }
        public void updateEncoders(){
            try {
                encoders.right = motorBackLeft.getCurrentPosition();
                encoders.left = - motorBackRight.getCurrentPosition();
                encoders.center = motorFrontLeft.getCurrentPosition();
            }
            catch (Exception e) {
                e.printStackTrace();
            }

        }
    }
}
