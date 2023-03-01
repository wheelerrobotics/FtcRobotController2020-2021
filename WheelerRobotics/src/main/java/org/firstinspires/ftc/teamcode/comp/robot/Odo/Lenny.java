package org.firstinspires.ftc.teamcode.comp.robot.Odo;

import static java.lang.Math.E;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.comp.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.comp.helpers.Heights;
import org.firstinspires.ftc.teamcode.comp.helpers.PID;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;

@Config
public class Lenny extends Meccanum implements Robot {
    protected HardwareMap hw = null;


    public double maxHeight = 4200; // change to fit slide
    public double minHeight = 0; // probably good (maybe just set to 20 so the 10ish off errors are unnoticed)

    public static double differenceScalar = 0.01; // scales slide tick difference correction intensity
    public static double scaler = 0.008; // scales width of simoid, a const goes along with it so dont change on its own
    public static double sp = 0.005; // slide kp const
    public static double sd = 0; // slide kp const
    public double currentCone = 5;

    public DcMotorEx slideLeft = null;
    public DcMotorEx slideRight = null;

    public Servo leftArm = null;
    public Servo rightArm = null;
    public Servo wrist = null;
    public Servo claw = null;

    public DistanceSensor dist = null;

    SlideThread st = new SlideThread();
    ClawArmWristThread cawt = new ClawArmWristThread();
    AprilDet ad = null;

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


        slideLeft = (DcMotorEx) hardwareMap.dcMotor.get("slideLeft");
        slideRight = (DcMotorEx) hardwareMap.dcMotor.get("slideRight");

        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw"); // port 2
        wrist = hardwareMap.get(Servo.class, "wrist"); // port 3
        leftArm = hardwareMap.get(Servo.class, "armLeft"); // port 4
        rightArm = hardwareMap.get(Servo.class, "armRight"); // port 5

        leftArm.setDirection(Servo.Direction.REVERSE);


        dist = hardwareMap.get(DistanceSensor.class, "dist"); // port 2
        // Meccanum Motors Definition and setting prefs

        // motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft");
        // motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft");
        // motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight");
        // motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight");



        // Reverse the left side motors and set behaviors to stop instead of coast

        //motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;



        runtime.reset();
    }
    public void slideinit() {
        st.start();
    }
    public void cawtinit() {
        cawt.start();
    }
    public void detinit(String webcamName) {
        ad = new AprilDet();
        ad.init(hw, webcamName);
    }
    public double getDistance(DistanceUnit units) {
        return dist.getDistance(units);
    }

    /*
    public sampleSensorsForPose() {
        this.dx =
    }
    public updatePose(){

    }
*/
    public int getPrincipalTag(){
        ad.checkDetections();
        return ad.getDetected();
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
    // Late night thoughts so I can continue them tmrw:
    /*
    - ideally, run the positioning stuff on a seperate thread
        - have a method to talk to that thread and get position while we do other stuff on main.
        - this is convinient because it means we dont have to worry about delay on the position resulting in more accurate measurements.
    - Look at the gm0 thing i have open to figure out how to use the encoders.
    - Would be sick if we actually use tensorflow models. Could be useful for object detection of the junctions.

     */
    public void tick() {
        st.tick();
        cawt.tick();
    }

    public boolean SLIDE_TARGETING = false;
    //SLIDES
    public void driveSlides(double power) {
        st.driveSlides(power);
        SLIDE_TARGETING = false;
    }
    public double getSlideHeight() {
        return st.getSlidesPos();
    }
    public void setSlideTarget(double target) {
        st.setTarget(target);
    }
    public void setSlideTargetDelay(double target, double ms) {
        st.setSlideTargetDelay(target, ms);
    }
    // CAWT (Claw Arm Wrist Thread)
    public void setArmTarget(double target) {
        cawt.setArmTarget(target);
    }
    public void setSlideMinMax(double min, double max) {
        st.setMinMax(min, max);
    }
    public void setWristTarget(double target) {
        cawt.setWristTarget(target);
    }
    public void setClawTarget(double target) {
        cawt.setClawTarget(target);
    }
    public void setSlideNextCone() {
        if (currentCone == 5) st.setTarget(Heights.cone5);
        else if (currentCone == 4) st.setTarget(Heights.cone4);
        else if (currentCone == 3) st.setTarget(Heights.cone3);
        else if (currentCone == 2) st.setTarget(Heights.cone2);
        else if (currentCone == 1) st.setTarget(Heights.cone1);
        else st.setTarget(Heights.cone);
        currentCone--;
    }

    public boolean isBusy() {
        // true if there are any unresolved targets in caw or slide
        return cawt.isBusy() && st.isBusy();
    }
    private class ClawArmWristThread {
        public double clawClosed = 0.7; // 0.76 for non-straining I think
        public double clawOpen = 1;
        public double levelWristPlace = 0.05;
        // will we ever want a non level claw rot for cycling back from a place? or does that just complicate too much :/
        public double levelWristPickup = 0.725;
        public double lowArmPickup = 0.92;
        public double levelArmPickup = 0.915;
        public double upSlantArmPlace = 0.31;
        public double levelArmPlace = 0.24;
        public double beforeSlidesArmPlace = 0.53;
        public double beforeSlidesArmPickup = 0.63;
        public double armCenter = (beforeSlidesArmPickup + beforeSlidesArmPlace)/2;

        // TODO: should make function out of needed values so for some arbitrary clawPos I can extrapolate maxClawBeforeSlidesDistance for Pickup/Placea
        // TODO: NEEDED VALUES: how close the claw hits the slides (each side) if closed and rotating
        public double maxClawClosedBeforeSlidesDistancePickup = 0.69; // GUESS
        public double maxClawClosedBeforeSlidesDistancePlace = 0.47; // GUESS
        // TODO: NV: how close claw hits slides (each side) when open and rotating
        public double maxClawOpenBeforeSlidesDistancePickup = 0.8; // GUESS
        public double maxClawOpenBeforeSlidesDistancePlace = 0.36; // GUESS

        public double armTarget = levelArmPlace;
        public double wristTarget = levelWristPickup;
        public double clawTarget = clawOpen;

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        public void setTele(Telemetry t) {
            tele = t;
        }

        public void start() {

        }

        public void tick() {
            setArmPos(armTarget);
            setWristPos(wristTarget);
            setClawPos(clawTarget);
        }
        public void setArmTarget(double target) {
            armTarget = target;
        }
        public void setWristTarget(double target) {
            wristTarget = target;
        }
        public void setClawTarget(double target) {
            clawTarget = target;
        }
        // second priority (shoudnt ever conflict tho)
        boolean CLAW_SAFE = false; // assume safety, dont leave room for nullpointers
        public void setClawPos(double pos) {
            double buffer = 0.03;
            claw.setPosition(pos);
            if (CLAW_SAFE) claw.setPosition(pos);
            else claw.setPosition(clawClosed + 0.07);

        }
        // third priority
        boolean WRIST_SAFE = false; // never assume safety until checked, dont leave room for nullpointers
        public void setWristPos(double pos) {
            if (WRIST_SAFE) wrist.setPosition(pos);
            else wrist.setPosition(abs(pos - levelWristPickup) > abs(pos - levelWristPlace) ? levelWristPlace : levelWristPickup);
        }

        boolean ARM_THROUGH_SAFE = false; // never assume safety until checked, dont leave room for nullpointers
        boolean ARM_THROUGH_ON_CURRENT = false; // never assume safety until checked, dont leave room for nullpointers
        double lastPos = 0;
        public void setArmPos(double pos) {
            double buffer = 0.05; // safety buffer
            ARM_THROUGH_SAFE = getWristPos() == levelWristPickup || getWristPos() == levelWristPlace;
            ARM_THROUGH_ON_CURRENT = (getArmPos() > beforeSlidesArmPlace - buffer)
                    && (getArmPos() < beforeSlidesArmPickup + buffer);
            if ((lastPos < armCenter && pos > armCenter) || (pos < armCenter && lastPos > armCenter))

            if (ARM_THROUGH_SAFE) {
                // if wrist is in non-obstruc pos (with cone), ARM_THROUGH_SAFE, dont care, set arm to whatever
                //setArmPosBasic(pos);
            }
            if (ARM_THROUGH_ON_CURRENT) {
                // if current movement will pass through slides (ARM_THROUGH_ON_CURRENT),
                //      and wrist in possibly obstructive place (!ARM_THROUGH_SAFE),
                //      freeze and rotate wrist first
                CLAW_SAFE = false;
                WRIST_SAFE = false;
                //setArmPosBasic(getArmPos()); // freeze arm until safe
            }
            // otherwise, wrist is safe to do whatever
            else {
                WRIST_SAFE = true;
                CLAW_SAFE = true;
            }
            setArmPosBasic(pos);
            tele.addData("arm", getArmPos());
            lastPos = pos;
        }
        private void setArmPosBasic(double pos) {
            // servos are slightly off, this corrects.
            // NOTE: NEVER DIRECTLY SET ARM SERVO POSITIONS WITHOUT THIS FUNCTION!!!!
            rightArm.setPosition(pos > 0.03 ? pos : 0);
            leftArm.setPosition(pos-0.03 > 0 ? pos-0.03 : 0);
        }
        public double getArmPos() {
            return rightArm.getPosition();
        }
        public double getClawPos() {
            return claw.getPosition();
        }
        public double getWristPos() {
            return wrist.getPosition();
        }
        public boolean isBusy() {
            // claw has leway because sometimes set to impossible position in order to squeeze
            double clawBuffer = 0.08;
            boolean armBusy = armTarget == getArmPos();
            boolean clawBusy = (getClawPos() - clawBuffer) < clawTarget && (getClawPos() + clawBuffer) > clawTarget;
            // !WRIST_SAFE to avoid stalling in a set position for arm in slides while setpos for wrist is weird
            boolean wristBusy = getWristPos() == wristTarget || !WRIST_SAFE;
            return armBusy && clawBusy && wristBusy;
        }
    }
    private class SlideThread {
        public double leftBasePos = 0;
        public double rightBasePos = 0;
        public double leftPos = 0;
        public double rightPos = 0;
        public double errorThreshold = 20;
        public double derivativeThreshold = 1;
        public double slideTar = 0; // target of slide (duh)

        public double slideTargetDelayMs = 0;
        public double slideTargetDelay = 0;
        public ElapsedTime slideTargetTimer = new ElapsedTime();

        public double power = 0;

        //public double slideTar = 0;
        public PID slidePID;

        //public double maxHeight = 1000;
        //public double minHeight = 0;

        //public double differenceScalar = 0.0001;
        //public double scaler = 50;
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        public void setTele(Telemetry t) {
            tele = t;
        }

        public void start() {
            leftBasePos = slideLeft.getCurrentPosition();
            rightBasePos = slideRight.getCurrentPosition();
            slideTargetTimer.reset();

            slidePID = new PID(0.001, 0, 0, false);
            tele = FtcDashboard.getInstance().getTelemetry();
        }
        public void setSlideTargetDelay(double target, double ms) {
            slideTargetDelayMs = ms;
            slideTargetDelay = target;
            slideTargetTimer.reset();
        }
        public void setMinMax(double min, double max) {
            minHeight = min;
            maxHeight = max;
        }

        public void tick() {

            slidePID.setConsts(sp, 0, sd);
            slidePID.setTarget(slideTar);
            leftPos = slideLeft.getCurrentPosition() - leftBasePos;
            rightPos = slideRight.getCurrentPosition() - rightBasePos;

            tele.addData("left", leftPos);
            tele.addData("right", rightPos);

            if ((leftPos + rightPos) /2 < minHeight  && power < 0) {
                SLIDE_TARGETING = true;
                slideTar = minHeight;
            }
            if ((rightPos+leftPos)/2 > maxHeight && power > 0) {
                SLIDE_TARGETING = true;
                slideTar = maxHeight;
            }
            if (SLIDE_TARGETING) {
                power = slidePID.tick((leftPos + rightPos) / 2);
                tele.addData("pidpower", power);
            }
            if (slideTargetDelayMs != 0) {
                if (slideTargetTimer.milliseconds() > slideTargetDelayMs) {
                    setTarget(slideTargetDelay);
                    SLIDE_TARGETING = true;
                    slideTargetDelayMs = 0;
                }
            }

            tele.addData("drivingl", minMaxScaler(leftPos, (power + differenceScaler(rightPos - leftPos))));
            tele.addData("drivingr", minMaxScaler(rightPos, (power + differenceScaler(leftPos - rightPos))));
            tele.addData("dl", differenceScaler(rightPos - leftPos));
            tele.addData("dr", differenceScaler(leftPos - rightPos));
            tele.addData("slides", SLIDE_TARGETING);
            tele.addData("pos", getSlidesPos());
            tele.update();


            slideLeft.setPower(minMaxScaler(leftPos, (power + differenceScaler(rightPos - leftPos))));
            slideRight.setPower(minMaxScaler(rightPos, (power + differenceScaler(leftPos - rightPos))));

        }

        public double minMaxScaler(double x, double power) {
            return power * (!SLIDE_TARGETING ? (power < 0 ? ((1.3 * 1/(1+pow(E, -scaler*(x-300+minHeight)))) - 0.1) : ((1.3 * 1/(1+pow(E, scaler*(x+300-maxHeight)))) - 0.1)) : 1);
        }

        public double differenceScaler(double difference) {
            return differenceScalar * difference;
        }

        public void driveSlides(double p) {
            tele.addData("cpower", power);
            SLIDE_TARGETING = false;
            power = p;
        }
        public double getSlidesPos() {
            return (leftPos + rightPos) / 2;
        }

        public void setTarget(double tar) {
            slideTar = tar;
            SLIDE_TARGETING = true;
        }
        public boolean isBusy() {

            return slidePID.getDerivative() < derivativeThreshold && abs(getSlidesPos() - slideTar) < errorThreshold;
            //                                                       could get proportion (^) from pid but dont want to
        }

    }


    public void setTele(Telemetry t) {
        st.setTele(t);
    }

}
