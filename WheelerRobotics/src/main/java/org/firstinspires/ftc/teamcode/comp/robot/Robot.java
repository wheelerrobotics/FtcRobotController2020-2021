package org.firstinspires.ftc.teamcode.comp.robot;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public interface Robot {
    ElapsedTime runtime = new ElapsedTime();
    void init(HardwareMap hardwareMap); // this is where all the hardware declarations go :)

}
