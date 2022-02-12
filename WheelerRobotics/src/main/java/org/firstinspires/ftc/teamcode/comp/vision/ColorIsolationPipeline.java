package org.firstinspires.ftc.teamcode.comp.vision;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import androidx.core.graphics.ColorUtils;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;

class ColorIsolationPipeline extends OpenCvPipeline
{
    int quality = 5;
    private HashMap<Integer, HashMap<String, Integer>> maxes;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry tele = dash.getTelemetry();


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

        // cone hsl: max(250, 36.3%, 44.3%) min(266, 47.4%, 26.1%)
        // cone hsl: target (249, 51.6%, 60.3%)
        // cone hsl: thresh (1, 15.3, 16)

        // felt cone hsl: min(267, 32.5%, 47.1%) max(248, 66.9%, 76.3%)
        // felt cone hsl: target (266, 36.6%, 39.95%)
        // felt cone hsl: thresh (2, 10.5, 5.95)

        float htar = 266F;
        float star = 0.37F;
        float ltar = 0.4F;

        float hthresh = 4F;
        float sthresh = 0.11F;
        float lthresh = 0.6F;


        float[] hsl = {0, 0, 0};
        ArrayList<ArrayList<Integer>> detect = new ArrayList<ArrayList<Integer>>();

        int left = 0;
        int right = 0;

        // isolate color range
        for(int i = 0; i<input.rows(); i+=quality){
            ArrayList<Integer> nowList = new ArrayList<Integer>();
            for(int e = 0; e<input.cols(); e+=quality){
                ColorUtils.RGBToHSL((int) input.get(i, e)[0], (int) input.get(i, e)[1], (int) input.get(i, e)[2], hsl);
                if (hsl[0] > htar-hthresh && hsl[0] < htar+hthresh &&
                        hsl[1] > star-sthresh && hsl[1] < star+sthresh &&
                        hsl[2] > ltar-lthresh && hsl[2] < ltar+lthresh) {
                    nowList.add(1);
                    if(e < input.cols()/2) left += 1;
                    else right += 1;
                    Imgproc.circle(input, new Point(e, i), 0, new Scalar(input.get(i, e)[0], 0, input.get(i, e)[2]), quality);
                } else {
                    Imgproc.circle(input, new Point(e, i), 0, new Scalar(input.get(i, e)[0]+10, input.get(i, e)[1]+10, input.get(i, e)[2]+10), quality);
                    nowList.add(0);
                }
            }
            detect.add(nowList);
        }

        // "blur", assign probability to each pixel
        /*for(int i = 0; i<input.rows()/quality; i++){
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
        }*/
        /*
        int cmax = 0;
        maxes = new HashMap<Integer, HashMap<String, Integer>>();
        */

        /*
            1:
                "x": 20
                "y": 10
            2:
                "x": 20
                "y": 10
         */
        // get greatest max
        /*for(int i = 0; i<detect.size(); i++) {
            for (int e = 0; e < detect.get(i).size(); e++) {
                if(detect.get(i).get(e) > cmax){
                    cmax = detect.get(i).get(e);
                }
            }
        }

        // create list of points from values of greatest max
        for(int i = 0; i<detect.size(); i++){
            for(int e = 0; e<detect.get(i).size(); e++){
                if(detect.get(i).get(e) >= cmax){
                    HashMap<String, Integer> newPoint = new HashMap<String, Integer>();
                    newPoint.put("x", e);
                    newPoint.put("y", i);
                    maxes.put(maxes.size(), newPoint);
                    FtcDashboard.getInstance().getTelemetry().addData("mac", cmax);
                }
            }
        }*/

        FtcDashboard.getInstance().getTelemetry().addData("right", right);
        FtcDashboard.getInstance().getTelemetry().addData("left", left);

        if (right > 5 && left < right) FtcDashboard.getInstance().getTelemetry().addLine("Prediction! 2");
        else if (left > 5 && left > right) FtcDashboard.getInstance().getTelemetry().addLine("Prediction! 1");
        else FtcDashboard.getInstance().getTelemetry().addLine("Prediction! 3");

        FtcDashboard.getInstance().getTelemetry().update();

        //maxes = maxer(maxes, 2);
        // draw where the things are
        //for(HashMap<String, Integer> point : maxes.values()){ }

        return input;
    }


    public HashMap<Integer, HashMap<String, Integer>> getDetections(){
        tele.addData("maxes", maxes.toString());
        return maxes;
    }


    // maxer algo translated from python
    // get pixels around positives and eliminate them if another is found withing distance to it
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



        for (Integer targetMax : remover) {
            selectedMaxes.remove(targetMax);
        }

        return selectedMaxes;


    }

    @Override
    public void onViewportTapped() {
        tele.addData("maxes", maxes.toString());
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