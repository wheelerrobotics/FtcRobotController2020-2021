package org.firstinspires.ftc.teamcode.comp.robot.Odo;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class Odo extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    public static double trackwidth = 7.505;
    public static double forward_offset = -4;
    public static double scalar = 0.7;

    public static double xp = 0.06; //0.00005;
    public static double xd = 0.3; //0.0004;
    public static double xi = 0;  //0.0004;
    public static double yp = 0.06; //0.00005; // 0.00005
    public static double yd = 0.3; //0.0004; // 0.0004
    public static double yi = 0; //0.0004;
    public static double rp = -2; //0.00005;
    public static double rd = -0.4; //0.0004;
    public static double ri = 0; //0.0004;
    public static double dthresh = 0.001;
    public double scaleFactor = 0.6;
    public boolean TESTING = false;
    public void test() {
        TESTING = true;
        pt.start();
        xp = 1;
        xd = 0;
        yd = 0;
        rd = 0;
        yp = 1;
        rp = 1;
    }
    public void setTestConsts(Pose pos, Pose target) {
        pt.setTestPoses(pos, target);
    }

    public double stalPower = 0.08;

    double xTarget = 0;
    double yTarget = 0;
    double rTarget = 0;

    public boolean opModeIsActive = true;
    public boolean pidActive = false;

    PIDThread pt = new PIDThread();

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

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;

        pt.start();


        runtime.reset();
    }
    public void autoinit() {
        pt.encoders = new Encoders(0, 0, 0);

        //at.start();
    }

    /*
    public sampleSensorsForPose() {
        this.dx =
    }
    public updatePose(){

    }
*/

    public Pose getPose() {
        return pt.pose;
    }
    public Pose tick() {
        return pt.tick();
    }
    public int getPrincipalTag(){
        return 0;
    }

    public void pidDrive(double x, double y, double r) {
        xTarget = x;
        yTarget = y;
        rTarget = r;
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

    public void tickPID() {
        pt.tick();
    }
    public void setMovement(boolean movement) {
        pt.setMovement(movement);
    }
    private class PIDThread
    {

        private Encoders encoders = new Encoders(0, 0, 0);
        private PID py, px, pr = null;
        boolean MOVING = true;
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
        Pose pose = new Pose(0, 0, 0);
        Pose roboTargetVectors = new Pose(0, 0, 0);
        Pose fieldTargetVectors = new Pose(0, 0, 0);
        Pose fieldTargetPose = new Pose(0, 0, 0);

        FtcDashboard dashboard = null;
        Telemetry tele = null;


        double dx = 0;
        double dy = 0;
        double dr = 0;
        double ecenter = 0;
        double eleft = 0;
        double eright = 0;
        double lastCenter = 0;
        double lastLeft = 0;
        double lastRight = 0;

        double deltaCenter = 0;
        double deltaRight = 0;
        double deltaLeft = 0;


        public void start() {

            py = new PID(yp, yi, yd, false); // don't need to correct for sensor jitter because we are using encoders
            py.init(pose.y);
            px = new PID(xp, xi, xd, false); // -0.025, -0.00008, -0.2
            px.init(pose.x);
            pr = new PID(rp, ri, rd, false); // -0.025, -0.00008, -0.2
            pr.init(pose.r);
            if (!TESTING) {
                dashboard = FtcDashboard.getInstance();
                tele = FtcDashboard.getInstance().getTelemetry();
                ecenter = motorFrontLeft.getCurrentPosition();
                eleft = -motorBackLeft.getCurrentPosition();
                eright = motorBackRight.getCurrentPosition();
                lastCenter = ecenter;
                lastLeft = eleft;
                lastRight = eright;
            }

        }
        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        public void setTestPoses(Pose pos, Pose target){
            fieldTargetPose = target;
            pose = pos;
        }
        public void setMovement(boolean movement) {
            MOVING = movement;
            if (!MOVING) {
                motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorStop();
            }
            else {
                motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
        }
        public Pose tick() {
            // we record the Y values in the main class to make showing them in telemetry
            // easier.

                    // TODO: add dimension for rotation, will involve calculating x/y with rotation
                    //  in mind thus a combination of current x/y encoder readings. We want to
                    //  maintain an absolute positioning system (field centric)

                    if (!TESTING) {
                        fieldTargetPose.x = xTarget; // in (experimentally obtained)
                        fieldTargetPose.y = yTarget; // in
                        fieldTargetPose.r = rTarget; // radians

                        px.setConsts(xp, xi, xd);
                        py.setConsts(yp, yi, yd);
                        pr.setConsts(rp, ri, rd);
                    }else {

                        px.setConsts(1, 0, 0);
                        py.setConsts(1, 0, 0);
                        pr.setConsts(1, 0, 0);
                    }

                    px.setTarget(fieldTargetPose.x);
                    py.setTarget(fieldTargetPose.y);
                    pr.setTarget(fieldTargetPose.r);



                    double[] out = {0, 0, 0, 0};
                    double ex = px.tick(pose.x);
                    double ey = py.tick(pose.y);
                    double er = pr.tick(pose.r);

                    fieldTargetVectors.x = ex;
                    fieldTargetVectors.y = ey;
                    fieldTargetVectors.r = er;

                    roboTargetVectors = fieldTargetVectors.getPoseRobotCentric(pose.r);

                    // roboTargetVectors.setPose(roboTargetVectors.x , roboTargetVectors.y, roboTargetVectors.r);

                    if (TESTING) return roboTargetVectors;


                    tele.addData("ex", ex);
                    tele.addData("ey", ey);
                    tele.addData("er", er);

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay()
                            .strokeCircle(pose.y, pose.x, 10).strokeLine(pose.y, pose.x, pose.y + 7*cos(pose.r), pose.x + 7*sin(pose.r));
                    dashboard.sendTelemetryPacket(packet);
                    packet.fieldOverlay()
                            .strokeCircle(fieldTargetPose.y, fieldTargetPose.x, 3);
                    dashboard.sendTelemetryPacket(packet);

                    tele.addData("rex", roboTargetVectors.x);
                    tele.addData("rey", roboTargetVectors.y);
                    tele.addData("rer", roboTargetVectors.r);

                    for (int i = 0; i<out.length; i++) { // add the individual vectors
                        out[i] = roboTargetVectors.x * this.left[i] + roboTargetVectors.y * this.back[i] + roboTargetVectors.r * this.clock[i];
                    }

                    // TODO: is there anything wrong with linear scaling (dividing by greatest value) here?
                    double abc = absmac(out); // get max value for scaling
                    if (abc > 1){
                        for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                            out[i] /= abs(abc);
                        }
                    }

                    for (int i = 0; i<out.length; i++) out[i] *= scalar;
                    for (int i = 0; i<out.length; i++) if (abs(out[i]) < stalPower) out[i] = 0;

                    if (!TESTING && MOVING) driveVector(out);

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



                    ecenter = motorFrontLeft.getCurrentPosition();
                    eleft = -motorBackLeft.getCurrentPosition();
                    eright = motorBackRight.getCurrentPosition();

                    tele.addData("d", eright - eleft);
                    // new copied math :)
                    deltaLeft = eleft - lastLeft;
                    deltaRight = eright - lastRight;
                    deltaCenter = ecenter - lastCenter;
                    // odometry solution: GET MIRRORED ODOMETRY PODS (im so stupid :/)
                    double phi = (deltaLeft - deltaRight) / trackwidth;

                    double delta_middle_pos = (deltaLeft + deltaRight) / 2;
                    double delta_perp_pos = deltaCenter - forward_offset * phi;

                    double delta_y = delta_middle_pos * cos(pose.r) - delta_perp_pos * sin(pose.r);
                    double delta_x = delta_middle_pos * sin(pose.r) + delta_perp_pos * cos(pose.r);

                    pose.setPose(pose.x + delta_x * 96 / 164386.368 * 144 / 149.9566615577327, pose.y + delta_y * 120.3 / 213441.556 * 144 / 144.68928366141162, pose.r + phi * (4*PI / 22658.894070619575) *(20*PI / 63.92351749263598) * (40*PI / 122.35421285146982));

                    lastCenter = ecenter;
                    lastLeft = eleft;
                    lastRight = eright;


                    tele.addData("px", pose.x);
                    tele.addData("py", pose.y);
                    tele.addData("pr", pose.r);

                    tele.update();
                    return roboTargetVectors;


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
                encoders.left = -motorBackRight.getCurrentPosition();
                encoders.center = motorFrontLeft.getCurrentPosition();
            }
            catch (Exception e) {
                e.printStackTrace();
            }

        }
    }
}
