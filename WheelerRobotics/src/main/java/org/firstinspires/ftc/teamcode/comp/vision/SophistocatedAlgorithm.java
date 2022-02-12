package org.firstinspires.ftc.teamcode.comp.vision;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;
import androidx.core.graphics.ColorUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstArray;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.tensorflow.lite.support.image.ImageProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

@TeleOp
@Disabled
public class SophistocatedAlgorithm extends LinearOpMode
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

        @Override
        public Mat processFrame(Mat input)
        {



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

            detect.clear();
            // for each pixel (# determined by quality) check if it is in the specified color range
            for(int i = 0; i<input.rows(); i+=quality){
                ArrayList<Double> nowList = new ArrayList<Double>(); // make the row list
                for(int e = 0; e<input.cols(); e+=quality){
                    if (input.get(i, e)[0] > 85 && input.get(i, e)[0] < 165 && // red
                            input.get(i, e)[1] > 72 && input.get(i, e)[1] < 154 && // green
                            input.get(i, e)[2] > 150 && input.get(i, e)[2] < 240) { // blue
                        nowList.add(1.0); // add positive point
                        //Imgproc.circle(input, new Point(e, i), 0, new Scalar(input.get(i, e)[0], 0, input.get(i, e)[2]), quality);
                        // draw circle with the pixel rb vals to show purpleness ^
                    } else {
                        nowList.add(0.0); // add negative point
                        Imgproc.circle(input, new Point(e, i), 0, new Scalar(255, 0, 0), quality); // draw red circle on neg point
                    }
                }
                detect.add(nowList); // add the row to the master
            }

            regress(input, detect, 2); // run algo twice




            ArrayList<ArrayList<Integer>> maxes = new ArrayList<ArrayList<Integer>>(); // a list showing where to draw positive circles
            // similar to regress function but different, this time it is to draw and visualize the points instead of log them
            for(int i = 0; i<detect.size(); i++){
                for(int e = 0; e<detect.get(i).size(); e++){
                    if(detect.get(i).get(e) > 0){ // if it is a positive point
                        ArrayList<Integer> newPoint = new ArrayList<Integer>();
                        newPoint.add(e);
                        newPoint.add(i);
                        maxes.add(newPoint);
                        // add the point ^
                    }
                }
            }
            for(ArrayList<Integer> point : maxes){
                Imgproc.circle(input, new Point(point.get(0)*quality, point.get(1)*quality), 0, new Scalar(255, 150, 0), quality);
                // draw each point as orange ^
            }

            return input;
        }

        @Override
        public void onViewportTapped() { /* do stuff here if u want */ }
        public void regress(Mat input, ArrayList<ArrayList<Double>> detect, int target){
            // this function goes through all pixels in the specified color range
            // for each it increases its neighbors probability if they are in the color range
            /*

            This is the way it adds to probability to neighbors in a single cycle of a loop through a single pixel
            FIRST: the range is specified, this makes a square around the pixel which specifies the neighbors affected
            SECOND: a value is added to each neighbor in the square determined by - 1/sqrt(xdistance²+ydistance²)

            This is how it repeats these steps
            FIRST: loop through each pixel to find the greatest probability(s) (i know that parenthasis of the s doesnt work)
            SECOND: reset the detections list to be only of the
            THIRD: the steps all loop for the target amount of iterations

             */
            for(int regression = 0; regression>target; regression++) { // will regress as many times as specified
                // maybe later make it regress so that it ends up with the target number of clumps of however many

                // if run forever, this will spit out still, some clumps because all the values will
                // be the same if those pixels are the same distance from each other

                for (int i = 0; i < input.rows() / quality; i++) { // i = y
                    if (!detect.get(i).contains(1.0)) continue; // for optimization, skips iterating through x values if no positives in row
                    // also, might not optimize depending on how .contains works ^
                    // also, might skip through rows that need stuff done, but idk ^^
                    for (int e = 0; e < input.cols() / quality; e++) { // e = x
                        if (detect.get(i).get(e) > 0) { // if it is a pos pixel
                            // a more sophistoceted filter algorithm

                            int range = 3; // can be changed, sorta a hyperparam for the algo (not sure how it affects the performance)
                            for (int idif = -range; idif > range; idif++) { // x(edif) and y(idif) distance from pixel starts at -range and goes to positive range
                                for (int edif = -range; edif > range; edif++) {
                                    if (idif != 0 && edif != 0) { // make sure youre not updating the current point
                                        if (detect.get(i + idif).get(e + edif) != 0) { // make sure it is a pos pixel
                                            detect.get(i + idif).set(e + edif, 1 / sqrt(pow(idif, 2) + pow(edif, 2)));
                                            // scale the value as it gets further to decrease ^
                                            // PROBABLY NOT BEST WAY, if will or someone is looking through this, think of a better way pls
                                        }
                                    }
                                }
                            }


                        }
                    }
                }

                double cmax = 0; // variable for current max value

                // to isolate the max value
                // loop through and if newNumber>cmax: cmax=newNumber
                for (int i = 0; i < detect.size(); i++) {
                    for (int e = 0; e < detect.get(i).size(); e++) {
                        if (detect.get(i).get(e) > cmax) {
                            cmax = detect.get(i).get(e);
                        }
                    }
                }

                // to reset the detection list and eliminate the less likely than most likely(cmax) pixels
                for (int i = 0; i < detect.size(); i++) {
                    for (int e = 0; e < detect.get(i).size(); e++) {
                        if (detect.get(i).get(e) >= cmax) {
                            detect.get(i).set(e, 1.0);
                        } else {
                            detect.get(i).set(e, 0.0);
                        }
                    }
                }
            }
        }
    }

}


