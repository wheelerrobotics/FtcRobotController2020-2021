package org.firstinspires.ftc.teamcode.comp.vision;
/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.pow;

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
import java.util.HashMap;

@TeleOp
@Disabled
public class WebcamExamples extends LinearOpMode
{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Opened!");
                telemetry.update();

            }

            @Override
            public void onError(int errorCode)
            {

                FtcDashboard.getInstance().getTelemetry().addData("Crashed", "camera");
                FtcDashboard.getInstance().getTelemetry().update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            FtcDashboard.getInstance().getTelemetry().addData("Frame Count", webcam.getFrameCount());
            FtcDashboard.getInstance().getTelemetry().addData("FPS", String.format("%.2f", webcam.getFps()));
            FtcDashboard.getInstance().getTelemetry().addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            FtcDashboard.getInstance().getTelemetry().addData("Pipeline time ms", webcam.getPipelineTimeMs());
            FtcDashboard.getInstance().getTelemetry().addData("Overhead time ms", webcam.getOverheadTimeMs());
            FtcDashboard.getInstance().getTelemetry().addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            FtcDashboard.getInstance().getTelemetry().update();

        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        int quality = 5;

        @Override
        public Mat processFrame(Mat input)
        {
// this function goes through all pixels in the specified color range
            // for each it increases its neighbors probability if they are in the color range
            /*

            This is the way it adds to probability
            x: the current target pixel

            |0|0|1|0|0|
            |0|2|3|2|0|
            |1|3|x|3|1|
            |0|2|3|2|0|
            |0|0|1|0|0|

            It then goes through each logging the highest scored pixel
            These pixels are then isolated and the same process is performed until there are however many masses detected
*/
            // 165 154 235
            // 85 72 154
            // 165 154 235
            // 85 72 154

            // cone hsl: min(250, 36.3%, 44.3%) max(248, 66.9%, 76.3%)
            // cone hsl: target (249, 51.6%, 60.3%)
            // cone hsl: thresh (1, 15.3, 16)

            float htar = 249F;
            float star = 0.516F;
            float ltar = 0.603F;

            float hthresh = 3F;
            float sthresh = 0.153F;
            float lthresh = 0.16F;



            float[] hsl = {0, 0, 0};
            ArrayList<ArrayList<Integer>> detect = new ArrayList<ArrayList<Integer>>();
            telemetry.addData("h", hsl[0]);
            telemetry.addData("s", hsl[1]*255);
            telemetry.addData("l", hsl[2]*255);


            for(int i = 0; i<input.rows(); i+=quality){

                ArrayList<Integer> nowList = new ArrayList<Integer>();
                for(int e = 0; e<input.cols(); e+=quality){
                    ColorUtils.RGBToHSL((int) input.get(i, e)[0], (int) input.get(i, e)[1], (int) input.get(i, e)[2], hsl);
                    if (hsl[0] > htar-hthresh && hsl[0] < htar+hthresh &&
                            hsl[1] > star-sthresh && hsl[1] < star+sthresh &&
                            hsl[2] > ltar-lthresh && hsl[2] < ltar+lthresh) {
                        nowList.add(1);
                        Imgproc.circle(input, new Point(e, i), 0, new Scalar(input.get(i, e)[0], 0, input.get(i, e)[2]), quality);
                    } else {
                        Imgproc.circle(input, new Point(e, i), 0, new Scalar(255, 0, 0), quality);
                        nowList.add(0);
                    }
                }
                detect.add(nowList);
            }


            for(int i = 0; i<input.rows()/quality; i++){
                for(int e = 0; e<input.cols()/quality; e++){
                    if(detect.get(i).get(e) > 0){
                        // close neighbors + 2
                        if(e-1 >= 0) if(detect.get(i).get(e-1) !=0) detect.get(i).set(e-1, detect.get(i).get(e-1) + 3);
                        if(e+1 < detect.get(i).size()-1) if(detect.get(i).get(e+1) !=0) detect.get(i).set(e+1, detect.get(i).get(e+1) + 3);
                        if(i-1 >= 0) if(detect.get(i-1).get(e) !=0) detect.get(i-1).set(e, detect.get(i-1).get(e) + 3);
                        if(i+1 < detect.size()-1) if(detect.get(i+1).get(e) !=0) detect.get(i+1).set(e, detect.get(i+1).get(e) + 3);

                        // corners
                        if(e-1 >= 0 && i-1 >= 0) if(detect.get(i-1).get(e-1) !=0) detect.get(i-1).set(e-1, detect.get(i-1).get(e-1) + 2);
                        if(e+1 < detect.get(i).size()-1 && i-1 >= 0) if(detect.get(i-1).get(e+1) !=0) detect.get(i-1).set(e+1, detect.get(i-1).get(e+1) + 2);
                        if(e-1 >= 0 && i+1 < detect.size()-1) if(detect.get(i+1).get(e-1) !=0) detect.get(i+1).set(e-1, detect.get(i+1).get(e-1) + 2);
                        if(e+1 < detect.get(i).size()-1 && i+1 < detect.size()-1) if(detect.get(i+1).get(e+1) !=0) detect.get(i+1).set(e+1, detect.get(i+1).get(e+1) + 2);

                        // 2 away
                        if(i-2 >= 0) if(detect.get(i-2).get(e) !=0) detect.get(i-2).set(e, detect.get(i-2).get(e) + 1);
                        if(i+2 < detect.size()-2) if(detect.get(i+2).get(e) !=0) detect.get(i+2).set(e, detect.get(i+2).get(e) + 1);
                        if(e-2 >= 0) if(detect.get(i).get(e-2) !=0) detect.get(i).set(e-2, detect.get(i).get(e-2) + 1);
                        if(e+2 < detect.get(i).size()-2) if(detect.get(i).get(e+2) !=0) detect.get(i).set(e+2, detect.get(i).get(e+2) + 1);

                    }
                }
            }
            int cmax = 0;
            HashMap<Integer, HashMap<String, Integer>> maxes = new HashMap<Integer, HashMap<String, Integer>>();
            for(int i = 0; i<detect.size(); i++) {
                for (int e = 0; e < detect.get(i).size(); e++) {
                    if(detect.get(i).get(e) > cmax){
                        cmax = detect.get(i).get(e);
                    }
                }
            }

            for(int i = 0; i<detect.size(); i++){
                for(int e = 0; e<detect.get(i).size(); e++){
                    if(detect.get(i).get(e) >= cmax){
                        HashMap<String, Integer> newPoint = new HashMap<String, Integer>();
                        newPoint.put("x", e);
                        newPoint.put("y", i);
                        maxes.put(maxes.size(), newPoint);
                        FtcDashboard.getInstance().getTelemetry().addData("mac", cmax);
                        FtcDashboard.getInstance().getTelemetry().update();
                    }
                }
            }
            //maxes = maxer(maxes, 2);
            for(HashMap<String, Integer> point : maxes.values()){
                Imgproc.circle(input, new Point(point.get("x")*quality, point.get("y")*quality), 0, new Scalar(255, 150, 0), quality);
            }

            return input;
        }
        public HashMap<Integer, HashMap<String, Integer>> maxer(HashMap<Integer, HashMap<String, Integer>> selectedMaxes, int distance){
            ArrayList<Integer> remover = new ArrayList<Integer>();
            for (HashMap<String, Integer> targetMax : selectedMaxes.values()){
                for (int i = 0; i<selectedMaxes.size(); i++) {
                    HashMap<String, Integer> otherMaxes = selectedMaxes.get(i);
                    if (targetMax == otherMaxes) continue ;

                    if (Math.sqrt(Math.pow(abs(targetMax.get("x") - otherMaxes.get("x")), 2) + pow(abs(targetMax.get("y") - otherMaxes.get("y")),2)) <= distance) {
                        remover.add(i);
                    }
                }
            }
            for (Integer targetMax : remover){
                selectedMaxes.remove(targetMax);
            }

            return selectedMaxes;

        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            /* viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }*/
        }
    }

}


