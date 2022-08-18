package org.firstinspires.ftc.teamcode.comp.chassis;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public interface Chassis {
    default void delay(double time){
        // used to stall program
        // used in many different applications and cloned to the auto classes
        ElapsedTime e = new ElapsedTime();
        e.reset();
        while(e.milliseconds() < time){
            // stal program
        }
    }
    default DistanceSensor try_declare_distance_sensor(String distance_sensor_name, HardwareMap hw){
        try {
            return hw.get(DistanceSensor.class, distance_sensor_name);
        } catch (Exception e) {
            return null;
        }
    }
}