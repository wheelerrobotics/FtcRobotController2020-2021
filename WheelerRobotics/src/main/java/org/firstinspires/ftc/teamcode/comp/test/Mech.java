package org.firstinspires.ftc.teamcode.comp.test;

import org.firstinspires.ftc.teamcode.comp.chassis.BaseMeccanum;

import java.util.Arrays;

public class Mech extends BaseMeccanum {
    int[] forward = { 1, 1,
                      1, 1 };
    int[] left = {  1,-1,
                   -1, 1 };

    void driveVectors(int front, int lefts, double r) {
        motorFrontLeft.setTargetPosition(front * forward[0] + lefts * left[0]);
        motorFrontRight.setTargetPosition(front * forward[1] + lefts * left[1]);
        motorBackLeft.setTargetPosition(front * forward[2] + lefts * left[2]);
        motorBackRight.setTargetPosition(front * forward[3] + lefts * left[3]);
    }
}
