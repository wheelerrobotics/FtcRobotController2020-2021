package org.firstinspires.ftc.teamcode.comp.tests;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.comp.helpers.PID;
import org.junit.Test;

public class PIDTest {
    @Test
    public void testPID_Zeroed() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing

        double kd = 0;
        double kp = 0;
        double ki = 0;

        double target = 30;

        PID testPID = new PID(kp, ki, kd, false);
        testPID.setTarget(target);
        assertEquals(0, testPID.tick(100), 0.0);
        assertEquals(0, testPID.tick(0), 0.0);
        assertEquals(0, testPID.tick(-100), 0.0);
    }
    @Test
    public void testPID_P() {
        double kd = 0;
        double kp = 1;
        double ki = 0;

        double target = 30;

        PID testPID = new PID(kp, ki, kd, false);
        testPID.setTarget(target);
        assertEquals(-70, testPID.tick(100), 0.0);
        assertEquals(0, testPID.tick(30), 0.0);
        assertEquals(30, testPID.tick(0), 0.0);
    }
    @Test
    public void testPID_D() {
        double kd = 0;
        double kp = 1;
        double ki = 0;

        double target = 30;
        // tick twice and measure second tick
    }
    @Test
    public void testPID_I() {
        double kd = 0;
        double kp = 1;
        double ki = 0;

        double target = 30;
        // tick twice (or more) and measure second tick
    }
    @Test
    public void testPID_Pause() {
        double kd = 0;
        double kp = 1;
        double ki = 0;

        double target = 30;
        // tick twice (or more) and measure second tick
    }
}
