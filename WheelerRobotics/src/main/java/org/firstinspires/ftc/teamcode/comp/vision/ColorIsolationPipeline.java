package org.firstinspires.ftc.teamcode.comp.vision;

import static org.checkerframework.checker.units.UnitsTools.min;
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
    private ArrayList<ArrayList<Integer>> detect;

    int left = 0;
    int right = 0;
    int cmax = 0;

    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final Telemetry tele = dash.getTelemetry();

    // 1, 2, 3 from left to right relative to robot
    private int conePosition = 0;

    public enum processors { OFF, SIMPLE, COMPLEX, SIMPLER }
    private processors processorSetting = processors.SIMPLE; // default to no processing as not to slow down imps

    private float Lhtar = 0F;
    private float Lstar = 0F;
    private float Lltar = 0F;
    private float Lhthresh = 0F;
    private float Lsthresh = 0F;
    private float Llthresh = 0F;

    public void setParams(float htar, float star, float ltar, float hthresh, float sthresh, float lthresh){
        Lhtar = htar;
        Lstar = star;
        Lltar = ltar;
        Lhthresh = hthresh;
        Lsthresh = sthresh;
        Llthresh = lthresh;
    }

    @Override
    public Mat processFrame(Mat input) {
        if(processorSetting == processors.COMPLEX){
            // will be to locate blocks
            detect = new ArrayList<ArrayList<Integer>>();
            cmax = 0;
            maxes = new HashMap<Integer, HashMap<String, Integer>>();
        } else if(processorSetting == processors.SIMPLER){
            // to get data of cone
            detect = new ArrayList<ArrayList<Integer>>();
            left = 0;
            right = 0;
            colorIsolator(input); // return detects and add to r/l
            updateConePos();
        } else if(processorSetting == processors.SIMPLE) {
            // to get data of cone
            detect = new ArrayList<ArrayList<Integer>>();
            left = 0;
            right = 0;
            coolColorIsolator(input); // return detects and add to r/l
            updateConePos();
        } else if (processorSetting == processors.OFF){
            // do something when not processing? idk
        }

        FtcDashboard.getInstance().getTelemetry().update();
        return input;
    }
    public void updateConePos(){
        // update cone position by getting if left/right regions have more detected pixels
        FtcDashboard.getInstance().getTelemetry().addData("right", right);
        FtcDashboard.getInstance().getTelemetry().addData("left", left);

        if (right > 5 && left < right) {
            FtcDashboard.getInstance().getTelemetry().addLine("Prediction! 2");
            conePosition = 2;
        } else if (left > 5 && left > right) {
            FtcDashboard.getInstance().getTelemetry().addLine("Prediction! 1");
            conePosition = 1;
        } else {
            FtcDashboard.getInstance().getTelemetry().addLine("Prediction! 3");
            conePosition = 3;
        }
    }
    public HashMap<Integer, HashMap<String, Integer>> getDetections(){
        // get specific detection positions, maybe maxer it later?

        tele.addData("maxes", maxes.toString());
        return maxes;
    }
    public void fromGreatestMax(int cmax){
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
        }
    }
    public void greatestMax(){
        // get greatest max
        for(int i = 0; i<detect.size(); i++) {
            for (int e = 0; e < detect.get(i).size(); e++) {
                if(detect.get(i).get(e) > cmax){
                    cmax = detect.get(i).get(e);
                }
            }
        }
    }
    public void colorIsolator(Mat input){
        // isolate color range
        float htar = 266F;
        float star = 0.5F;
        float ltar = 0.5F;

        float hthresh = 3F;
        float sthresh = 0.5F;
        float lthresh = 0.5F;


        float[] hsl = {0, 0, 0};

        for(int i = 0; i<input.rows(); i+=quality){
            ArrayList<Integer> nowList = new ArrayList<Integer>();
            for(int e = 0; e<input.cols(); e+=quality){
                ColorUtils.RGBToHSL((int) input.get(i, e)[0], (int) input.get(i, e)[1], (int) input.get(i, e)[2], hsl);

                if ((hsl[0] > htar-hthresh && hsl[0] < htar+hthresh &&
                        hsl[1] > star-sthresh && hsl[1] < star+sthresh &&
                        hsl[2] > ltar-lthresh && hsl[2] < ltar+lthresh)) {
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

    }
    public void coolColorIsolator(Mat input){

        float[] hsl = {0, 0, 0};

        for(int i = 0; i<input.rows(); i+=quality){
            ArrayList<Integer> nowList = new ArrayList<Integer>();
            for(int e = 0; e<input.cols(); e+=quality){
                ColorUtils.RGBToHSL((int) input.get(i, e)[0], (int) input.get(i, e)[1], (int) input.get(i, e)[2], hsl);
                if (hsl[0] > Lhtar-Lhthresh && hsl[0] < Lhtar+Lhthresh &&
                        hsl[1] > Lstar-Lsthresh && hsl[1] < Lstar+Lsthresh &&
                        hsl[2] > Lltar-Llthresh && hsl[2] < Lltar+Llthresh) {
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

    }
    public void blurer(Mat input) {

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

        // "blur", assign probability to each pixel
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
    }
    public HashMap<Integer, HashMap<String, Integer>> maxer(HashMap<Integer, HashMap<String, Integer>> selectedMaxes, int distance){
        // maxer algo translated from python
        // get pixels around positives and eliminate them if another is found withing distance to it
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
    public int getConePosition() {
        return conePosition;
    }
    public void setProcessorSetting(processors setting){
        processorSetting = setting;
    }

    @Override
    public void onViewportTapped() {
        tele.addData("maxes", maxes.toString());
    }
}