package org.firstinspires.ftc.teamcode.comp.teleop.Brokey;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.controller.Brokey.ControllerMapBrokey;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Brokey;

@TeleOp(name = "Conrad's ", group = "TeleOp")
@Disabled
public class BrokeyOkey extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Brokey b = new Brokey();
        b.init(hardwareMap);
        ControllerMapBrokey cmb = new ControllerMapBrokey();
        cmb.init(b, gamepad1, gamepad2);

        waitForStart();
        while (opModeIsActive()) {
            cmb.checkControls();


        }
    }
}
