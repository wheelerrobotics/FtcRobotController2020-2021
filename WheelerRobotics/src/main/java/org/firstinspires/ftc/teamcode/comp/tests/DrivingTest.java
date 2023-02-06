package org.firstinspires.ftc.teamcode.comp.tests;

import static org.junit.Assert.assertTrue;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.comp.utility.Pose;
import org.junit.Test;

public class DrivingTest {
    @Test
    public void testMathPP() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 0), new Pose(100, 100, 0));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x > 0);
        assertTrue(p.y > 0);


    }@Test
    public void testMathNP() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 0), new Pose(-100, 100, 0));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x < 0);
        assertTrue(p.y > 0);


    }@Test
    public void testMathPN() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 0), new Pose(100, -100, 0));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x > 0);
        assertTrue(p.y < 0);


    }
    @Test
    public void testMathNN() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 0), new Pose(-100, -100, 0));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x < 0);
        assertTrue(p.y < 0);


    }
    @Test
    public void testMathNNR() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 3.14), new Pose(-100, -100, 3.14));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x > 0);
        assertTrue(p.y > 0);


    }
    @Test
    public void testMathNPR() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 3.14), new Pose(-100, 100, 3.14));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x > 0);
        assertTrue(p.y < 0);


    }
    @Test
    public void testMathPNR() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 3.14), new Pose(100, -100, 3.14));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x < 0);
        assertTrue(p.y > 0);


    }
    @Test
    public void testMathPPR() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, 3.14), new Pose(100, 100, 3.14));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x < 0);
        assertTrue(p.y < 0);


    }
    @Test
    public void testMathZPR2() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, PI/2), new Pose(0, 100, PI/2));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x > 0);
        assertTrue(abs(p.y) < 2);
    }
    @Test
    public void testMathZNR2() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, PI/2), new Pose(0, -100, PI/2));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(p.x < 0);
        assertTrue(abs(p.y) < 2);
    }
    @Test
    public void testMathNZR2() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, PI/2), new Pose(-100, 0, PI/2));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(abs(p.x) < 1);
        assertTrue(p.y > 0);
    }
    @Test
    public void testMathPZR2() {
        // thought about making list then looping over values, but that would make it harder to
        // isolate the specific instance which is failing
        Odo odo = new Odo();
        odo.test();
        odo.setTestConsts(new Pose(0, 0, PI/2), new Pose(100, 0, PI/2));
        odo.tick();
        odo.tick();
        Pose p = odo.tick();

        System.out.println(p.x);
        System.out.println(p.y);
        System.out.println(p.r);
        assertTrue(abs(p.x) < 1);
        assertTrue(p.y < 0);
    }
}
