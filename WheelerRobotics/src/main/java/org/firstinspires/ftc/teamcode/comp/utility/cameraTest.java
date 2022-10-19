package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.ColorIsolationPipeline;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.DummyCVPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
public class cameraTest extends LinearOpMode {

    /*
    new capper color in dib light
    htar	50
    hthresh	10
    ltar	0.44
    lthresh	0.4
    star	0.92
    sthresh	0.1
     */
    @Config
    public static class fig {
        public static double fx = 578.272;
        public static double fy = 578.272;
        public static double cx = 402.145;
        public static double cy = 221.506;

        // UNITS ARE METERS
        public static double tagsize = 0.166;
    }
    private AprilTagDetectionPipeline aprilTagDetectionPipeline = null;
    private BotVision bv = new BotVision();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    int numFramesWithoutDetection = 0;
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(fig.tagsize, fig.fx, fig.fy, fig.cx, fig.cy);
        bv.init(hardwareMap, aprilTagDetectionPipeline);


        waitForStart();

        while (opModeIsActive()){
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if(detections!=null)
            {
                tele.addData("FPS", bv.webcam.getFps());
                tele.addData("Overhead ms", bv.webcam.getOverheadTimeMs());
                tele.addData("Pipeline ms", bv.webcam.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size()==0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection>=THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection=0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z<THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection:detections)
                    {
                        tele.addLine(String.format("\nDetected tag ID=%d",detection.id));
                        tele.addLine(String.format("Translation X: %.2f feet",detection.pose.x*FEET_PER_METER));
                        tele.addLine(String.format("Translation Y: %.2f feet",detection.pose.y*FEET_PER_METER));
                        tele.addLine(String.format("Translation Z: %.2f feet",detection.pose.z*FEET_PER_METER));
                        tele.addLine(String.format("Rotation Yaw: %.2f degrees",Math.toDegrees(detection.pose.yaw)));
                        tele.addLine(String.format("Rotation Pitch: %.2f degrees",Math.toDegrees(detection.pose.pitch)));
                        tele.addLine(String.format("Rotation Roll: %.2f degrees",Math.toDegrees(detection.pose.roll)));
                    }
                }

                tele.update();
                }
        }
    }
}
