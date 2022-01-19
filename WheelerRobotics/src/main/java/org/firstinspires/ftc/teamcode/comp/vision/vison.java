package org.firstinspires.ftc.teamcode.comp.vision;

import android.annotation.SuppressLint;
import android.os.Handler;
import android.os.Looper;
import android.telecom.Call;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "tfue", group = "Daniel's Wonderful Coding Adventures")
public class vison extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite"; // the actual first provided trained model
    private static final String[] LABELS = { // labels to be used for labeling the objects and referring to them maybe
            "Ball", // white whiffle ball freight
            "Cube", // yellow cube frewight
            "Duck", // yellow duck
            "Marker" // tape the duck sits upon
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = // holy shit thats a long key o-o
            "Ab7oYbj/////AAABmUWCVRgoZkuuu7bkLaZWztV7Ce6KfyDyGfPgSRuFHm3URYp9CqsXMGsvKUxaji17TFgwS/zkC66Lt1zEtXINDE58AeSA8f2gNcdr+pQO8hZvRmoIUjmniZ5Fr7IuPuHoz2cWgcPN8H8EJOviayojof5lroZ4A6HI9PXHowRW8GPkjFRJDOhY6GpapILDQRu4UxPNKqM6+diTjb2KkJn8XtGE5vxPdqRAS4dSPy9yRrCiAnwzTMRy+DaELRDsOl1sgaEOMzjlv1919iSxBUwQUTRRWeg13l14BemOfTgpCmpC2DbzoAujIBKolyeys0yXTLhI4ETJSOICAKp3wvhFwxpKFH1LCs+vIuTrxx+pACtK";

    // var to be used for viforia object
    private VuforiaLocalizer vuforia;

    // the actual od object (object detector, not overdose)
    private TFObjectDetector tfod;

    @SuppressLint("DefaultLocale") // just to supress warnings
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // THIS IS IMPORTANT SO LOOK AT IT AND DONT DELETE

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) { // while running
                if (tfod != null) { // if there is a object detector and all the inits worked
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); // take all the recognized objects into this list
                    if (updatedRecognitions != null) { // if there are actual objects in that list
                        telemetry.addData("# Object Detected", updatedRecognitions.size()); // say how many objects recognized in that list
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) { // iterate through that list but also through the number i
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel()); // print out "label (#of-element)  label-from-label-string[]-of-object"
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", // print out "left,top (#of-element)  frame coords of left and top recognized point"
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", // print out "right,bottom (#of-element)  frame coords of right and bottom recognized point"
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                            // IDEA ON USAGE :
                            // we can find where the objects are by combining size and location in frame
                            // we need to know where the webcam is and have a normalized position we can easily set it to if it is knocked.
                            // a webcam on a servo would look super cool and could maybe be useful if given lines of sight all around
                        }
                        telemetry.update(); // actually printing all the queued data
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //vuforia.getCamera().createCaptureRequest();
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}