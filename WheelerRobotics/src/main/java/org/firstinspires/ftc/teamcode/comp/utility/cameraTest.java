package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;

@Config
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

    private BotVision bv = new BotVision();
    enum hsl {
            HUE, SATURATION, LUMINANCE, OFF
    }
    private Telemetry tele = FtcDashboard.getInstance().getTelemetry();

    public static double htar = 0F;
    public static double star = 0.5F;
    public static double ltar = 0.5F;

    public static double hthresh = 0F;
    public static double sthresh = 0.5F;
    public static double lthresh = 0.5F;

    @Override
    public void runOpMode() throws InterruptedException {
        bv.init(hardwareMap);

        waitForStart();

        hsl setting = hsl.OFF;

        while (opModeIsActive()){
            bv.setParams(Float.parseFloat(Double.toString(htar)), Float.parseFloat(Double.toString(star)), Float.parseFloat(Double.toString(ltar)), Float.parseFloat(Double.toString(hthresh)), Float.parseFloat(Double.toString(sthresh)), Float.parseFloat(Double.toString(lthresh)));

            tele.addData("star", star);
            tele.addData("htar", htar);
            tele.addData("ltar", ltar);

            tele.addData("sthresh", sthresh);
            tele.addData("hthresh", hthresh);
            tele.addData("lthresh", lthresh);

            /*if(gamepad1.dpad_down){
                setting = hsl.HUE;
            }else if (gamepad1.dpad_up){
                setting = hsl.SATURATION;
            }else if(gamepad1.dpad_left){
                setting = hsl.LUMINANCE;
            }else if(gamepad1.dpad_right){
                setting = hsl.OFF;
            }

            if(setting == hsl.LUMINANCE){
                ltar -= gamepad1.left_stick_y/2000;
                lthresh -= gamepad1.right_stick_y/2000;
            }else if(setting == hsl.HUE){
                htar -= gamepad1.left_stick_y/100;
                hthresh -= gamepad1.right_stick_y/100;
            }else if(setting == hsl.SATURATION){
                star -= gamepad1.left_stick_y/2000;
                sthresh -= gamepad1.right_stick_y/2000;
            }*/



        }
    }
}
