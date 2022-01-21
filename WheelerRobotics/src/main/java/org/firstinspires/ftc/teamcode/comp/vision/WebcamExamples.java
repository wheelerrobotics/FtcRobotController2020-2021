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

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp
public class WebcamExamples extends LinearOpMode
{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new SamplePipeline());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().getTelemetry().addData("WEDABEST", "Frfr");
                FtcDashboard.getInstance().getTelemetry().update();
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
        webcam.startStreaming(320, 240);

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                //webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            //sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        int quality = 5;
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */

            // 165 154 235
            // 85 72 154
            ArrayList<ArrayList<Integer>> detect = new ArrayList<ArrayList<Integer>>();

                for(int i = 0; i<input.rows(); i+=quality){

                    ArrayList<Integer> nowList = new ArrayList<Integer>();
                    for(int e = 0; e<input.cols(); e+=quality){
                            if (input.get(i, e)[0] > 85 && input.get(i, e)[0] < 165 &&
                                    input.get(i, e)[1] > 72 && input.get(i, e)[1] < 154 &&
                                    input.get(i, e)[2] > 150 && input.get(i, e)[2] < 240) {
                                nowList.add(1);
                                //Imgproc.circle(input, new Point(e, i), 0, new Scalar(input.get(i, e)[0], 0, input.get(i, e)[2]), quality);
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
            ArrayList<ArrayList<Integer>> maxes = new ArrayList<ArrayList<Integer>>();
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
                        ArrayList<Integer> newPoint = new ArrayList<Integer>();
                        newPoint.add(e);
                        newPoint.add(i);
                        maxes.add(newPoint);
                        FtcDashboard.getInstance().getTelemetry().addData("mac", cmax);
                        FtcDashboard.getInstance().getTelemetry().addData("detect", detect);
                        FtcDashboard.getInstance().getTelemetry().update();
                    }
                }
            }
            for(ArrayList<Integer> point : maxes){
                Imgproc.circle(input, new Point(point.get(0)*quality, point.get(1)*quality), 0, new Scalar(255, 150, 0), quality);
            }




            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
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
        public void regress(Mat input, ArrayList<ArrayList<Integer>> detect){
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

            for(int i = 0; i<input.rows()/quality; i++){
                for(int e = 0; e<input.cols()/quality; e++){
                    if(detect.get(i).get(e) > 0){


                        // close neighbors +3
                        if(e-1 >= 0) if(detect.get(i).get(e-1) !=0) detect.get(i).set(e-1, detect.get(i).get(e-1) + 3);
                        if(e+1 < detect.get(i).size()-1) if(detect.get(i).get(e+1) !=0) detect.get(i).set(e+1, detect.get(i).get(e+1) + 3);
                        if(i-1 >= 0) if(detect.get(i-1).get(e) !=0) detect.get(i-1).set(e, detect.get(i-1).get(e) + 3);
                        if(i+1 < detect.size()-1) if(detect.get(i+1).get(e) !=0) detect.get(i+1).set(e, detect.get(i+1).get(e) + 3);

                        // corners +2
                        if(e-1 >= 0 && i-1 >= 0) if(detect.get(i-1).get(e-1) !=0) detect.get(i-1).set(e-1, detect.get(i-1).get(e-1) + 2);
                        if(e+1 < detect.get(i).size()-1 && i-1 >= 0) if(detect.get(i-1).get(e+1) !=0) detect.get(i-1).set(e+1, detect.get(i-1).get(e+1) + 2);
                        if(e-1 >= 0 && i+1 < detect.size()-1) if(detect.get(i+1).get(e-1) !=0) detect.get(i+1).set(e-1, detect.get(i+1).get(e-1) + 2);
                        if(e+1 < detect.get(i).size()-1 && i+1 < detect.size()-1) if(detect.get(i+1).get(e+1) !=0) detect.get(i+1).set(e+1, detect.get(i+1).get(e+1) + 2);

                        // 2 away +1
                        if(i-2 >= 0) if(detect.get(i-2).get(e) !=0) detect.get(i-2).set(e, detect.get(i-2).get(e) + 1);
                        if(i+2 < detect.size()-2) if(detect.get(i+2).get(e) !=0) detect.get(i+2).set(e, detect.get(i+2).get(e) + 1);
                        if(e-2 >= 0) if(detect.get(i).get(e-2) !=0) detect.get(i).set(e-2, detect.get(i).get(e-2) + 1);
                        if(e+2 < detect.get(i).size()-2) if(detect.get(i).get(e+2) !=0) detect.get(i).set(e+2, detect.get(i).get(e+2) + 1);

                    }
                }
            }

            int cmax = 0; // variable for current max value

            // to isolate the max value
            for(int i = 0; i<detect.size(); i++) {
                for (int e = 0; e < detect.get(i).size(); e++) {
                    if(detect.get(i).get(e) > cmax){
                        cmax = detect.get(i).get(e);
                    }
                }
            }

            // to reset the detection list and eliminate the less likely pixels
            for(int i = 0; i<detect.size(); i++){
                for(int e = 0; e<detect.get(i).size(); e++){
                    if(detect.get(i).get(e) >= cmax){
                        detect.get(i).set(e, 1);
                    }else{
                        detect.get(i).set(e, 0);
                    }
                }
            }

        }
    }

}


