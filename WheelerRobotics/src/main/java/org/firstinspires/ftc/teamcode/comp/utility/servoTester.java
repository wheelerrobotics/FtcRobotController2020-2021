package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class servoTester extends LinearOpMode {
    public volatile static double s1pos = 1;
    public volatile static double s3pos = 0;
    public volatile static double s4pos = 0;

    public volatile static boolean rev1 = false;
    public volatile static boolean rev2 = true;
    public volatile static boolean rev3 = false;
    public volatile static boolean rev4 = false;

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

        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
        Servo servo4 = hardwareMap.get(Servo.class, "servo4");




        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(s1pos > 0.03 ? s1pos : 0);
            servo2.setPosition(s1pos-0.03 > 0 ? s1pos-0.03 : 0);

            servo1.setDirection(rev1 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servo2.setDirection(rev2 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servo3.setPosition(s3pos);
            servo4.setPosition(s4pos);

            servo3.setDirection(rev3 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servo4.setDirection(rev4 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }

    }
}
