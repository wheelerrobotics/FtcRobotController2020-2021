package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;

@Config
@TeleOp
public class servoTester extends LinearOpMode {
    public volatile static double claw = 1;
    public volatile static double arm = 0.66;
    public volatile static double wrist = 0.93;
    public volatile static double slides = 0;

    public volatile static double smax = 0;
    public volatile static double smin = 0; // make sure down isnt up

    // claw closed = 0.7 open = 1
    // positions = relative to pickup side
    // left: 0.97 right: 1.00
    // claw rot leve: 0.05
    // claw rot level: 0.725 right side up pickup
    // lowest arm rot: 0.92 <- pickup level is ~0.915
    // level other side: 0.31 not exactly, but good for line up
    //                   0.24 exactly level

    // just about to hit belts: 0.53 on drop side, 0.63 on pickup side






    @Override
    public void runOpMode() throws InterruptedException {
        Lenny l = new Lenny();

        l.init(hardwareMap);
        l.slideinit();
        l.cawtinit();

        waitForStart();

        while (opModeIsActive()) {
            l.tick();
            l.setClawTarget(claw);
            l.setArmTarget(arm);
            l.setWristTarget(wrist);

            //l.setSlideTarget(slides);
            l.setSlideTarget(slides);

            l.setSlideMinMax(smin, smax);

        }

    }
}
