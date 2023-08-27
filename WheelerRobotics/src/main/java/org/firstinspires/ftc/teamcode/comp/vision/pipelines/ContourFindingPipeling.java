package org.firstinspires.ftc.teamcode.comp.vision.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ContourFindingPipeling extends OpenCvPipeline
{

    public enum processors { OFF, SIMPLE, COMPLEX, SIMPLER }
    private processors processorSetting = processors.COMPLEX; // default to no processing as not to slow down imps

    @Override
    public Mat processFrame(Mat input) {
        Mat hierarchey = new Mat();
        Mat gray = new Mat(input.rows(), input.cols(), input.type());
        Mat placeholder = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
            // will be to locate blocks

            Imgproc.adaptiveThreshold(input, binary, 125,
                    Imgproc.ADAPTIVE_THRESH_MEAN_C,
                    Imgproc.THRESH_BINARY, 11, 12);
            //Finding Contours
            /*
            List<MatOfPoint> contours = new ArrayList<>();

            hierarchey = new Mat();
            Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //Drawing the Contours
            Scalar color = new Scalar(0, 0, 255);

            Imgproc.drawContours(binary, contours, -1, color, 2, Imgproc.LINE_8, hierarchey, 2, new Point() ) ;

            Mat temp= new Mat();
            Imgproc.cvtColor(src,temp,COLOR_BGR2HSV);
            Scalar low= new Scalar(10,0,0);
            Scalar high= new Scalar(20,255,255);
            Mat mask = new Mat();
            inRange(temp,low,high,mask);

            image.setTo(new Scalar(0, 0, 255), mask);
 */

        return binary;
    }
}