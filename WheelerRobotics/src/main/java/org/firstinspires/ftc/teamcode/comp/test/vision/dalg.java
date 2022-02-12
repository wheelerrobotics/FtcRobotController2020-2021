package org.firstinspires.ftc.teamcode.comp.test.vision;

import static java.lang.Math.abs;

import androidx.core.graphics.ColorUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp
@Disabled
public class dalg extends LinearOpMode
{
    OpenCvWebcam webcam;

    // will kopans pls enjoy this partly overcommented code.

    // note: idk it this actually works, its just untested concepts

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.setPipeline(new SamplePipeline()); // open connection to cam device

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()  // this actually opens the device but async (idk what im talking abt)
        {
            @Override
            public void onOpened()
            {
                // i think this is supposed to stream to the hub, but it doesnt work :/
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 20);

                // 145-155
                //
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                FtcDashboard.getInstance().getTelemetry().addData("Crashed", "camera");
                FtcDashboard.getInstance().getTelemetry().update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */

            // debugging data (FPS is very useful for finding a sweet spot in img processing)
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }

    // note from repo, this runs on a different thread from opmode, so just fyi, it wont sync really
    class SamplePipeline extends OpenCvPipeline
    {

        int quality = 5;

        ArrayList<ArrayList<Double>> detect = new ArrayList<ArrayList<Double>>(); // defined here as to reduce some memory pressure

        double[] lab = {0.0, 0.0, 0.0};
        float[] hsl = {0, 0, 0};
        float target = 150;
        float threshold = 5;
        @Override
        public Mat processFrame(Mat input)
        {

            HashMap<Integer, Float> cols = new HashMap<>();
            HashMap<Integer, Float> rows = new HashMap<>();



            for(int i = 0; i<input.rows(); i+=quality){
                if(!rows.containsKey(i)) rows.put(i, (float) 0);
                for(int e = 0; e<input.cols(); e+=quality){
                    ColorUtils.RGBToHSL((int) input.get(i, e)[0],(int) input.get(10, 10)[1], (int) input.get(10, 10)[2], hsl);
                    if(!cols.containsKey(e)) cols.put(e, (float) 0);
                    if (hsl[0] > target-threshold && hsl[0] < target+threshold) {
                        cols.put(e, cols.get(e)+abs(target-hsl[0]));
                        rows.put(i, rows.get(i)+abs(target-hsl[0]));

                        //Imgproc.circle(input, new Point(e, i), 0, new Scalar(input.get(i, e)[0], 0, input.get(i, e)[2]), quality);
                    }
                }
            }

            int col = 0;
            int row = 0;

            float rowmax = 0;
            float colmax = 0;

            for(int i = 0; i<cols.size()*quality; i+=quality) {
                if(cols.get(i) > colmax){
                    colmax = cols.get(i);
                    col = i;
                }
            }

            for(int i = 0; i<rows.size()*quality; i+=quality) {
                if(rows.get(i) > rowmax){
                    rowmax = rows.get(i);
                    row = i;
                }
            }


            Imgproc.circle(input, new Point(col, row), 5, new Scalar(200, 200, 0), 5);

            FtcDashboard.getInstance().getTelemetry().addData("row", row);
            FtcDashboard.getInstance().getTelemetry().addData("col", col);

            FtcDashboard.getInstance().getTelemetry().addData("rowm", rows.get(row));
            FtcDashboard.getInstance().getTelemetry().addData("colm", cols.get(col));


            FtcDashboard.getInstance().getTelemetry().addData("rosw", rows.size());
            FtcDashboard.getInstance().getTelemetry().addData("cosl", cols.size());


            /*COLORS

            *note: some wont really work and that is because the low red might be more than the high red
            *this needs to be fixed

            IMPORTANT: these must be take with the webcam and then sampled for rgb vals
            IMPORTANT +1: a better webcam would be nice, the purple and blue is nearly identical on the current one

            got this.
            CONE PURPLE:
            max 165 154 235
            min 85 72 154

            gotta get these:
            CUBE ORANGE:
            DUCK YELLOW:
            BALL WHITE: (will be difficult :/)
            TAPE BLUE: (for detecting markers allowing for more dynamic barcode checking)
            TAPE RED: (for same as tape blue but for red team)

            should probably make a colors enum or smth
            */

            FtcDashboard.getInstance().getTelemetry().addData("h: ", hsl[0]);
            FtcDashboard.getInstance().getTelemetry().addData("s: ", hsl[1]);
            FtcDashboard.getInstance().getTelemetry().addData("l: ", hsl[2]);

            ColorUtils.RGBToLAB((int) input.get(100, 100)[0],(int) input.get(10, 10)[1], (int) input.get(10, 10)[2], lab);
            ColorUtils.RGBToHSL((int) input.get(100, 100)[0],(int) input.get(10, 10)[1], (int) input.get(10, 10)[2], hsl);

            Imgproc.circle(input, new Point(100, 100), 1, new Scalar(200, 200, 0), 1);

            FtcDashboard.getInstance().getTelemetry().update();


            return input;
        }

        @Override
        public void onViewportTapped() { /* do stuff here if u want */ }

    }

}


