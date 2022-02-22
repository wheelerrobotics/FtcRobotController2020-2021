package org.firstinspires.ftc.teamcode.comp.test.auto.mixedsensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.chassis.BaseMeccanum;


public class DistancePreference {
    private SensorDirection direction;
    private int ticks;
    private DistanceSensor selectedSensor;
    private boolean noSensor = false;
    private boolean weightOverride = false;

    private double sensorWeight = 0.5;
    private double encoderWeight = 0.5;

    public DistancePreference(SensorDirection sensorDirection, int ticks, HardwareMap hw){
        this.direction = sensorDirection;
        this.ticks = ticks;

        if(sensorDirection == SensorDirection.BACK) selectedSensor = hw.get(DistanceSensor.class, "distanceBack");
        else if(sensorDirection == SensorDirection.LEFT) selectedSensor = hw.get(DistanceSensor.class, "distanceLeft");
        else if(sensorDirection == SensorDirection.RIGHT) selectedSensor = hw.get(DistanceSensor.class, "distanceRight");
        else if(sensorDirection == SensorDirection.NONE) {
            noSensor = true;
            this.weightOverride = true;
            this.sensorWeight = 0;
            this.encoderWeight = 1;
        }
    }
    public DistancePreference(SensorDirection sensorDirection, int ticks, HardwareMap hw, double sensorWeight, double encoderWeight){
        this.direction = sensorDirection;
        this.ticks = ticks;
        this.weightOverride = true;

        this.sensorWeight = sensorWeight;
        this.encoderWeight = encoderWeight;

        if(sensorDirection == SensorDirection.BACK) selectedSensor = hw.get(DistanceSensor.class, "distanceBack");
        else if(sensorDirection == SensorDirection.LEFT) selectedSensor = hw.get(DistanceSensor.class, "distanceLeft");
        else if(sensorDirection == SensorDirection.RIGHT) selectedSensor = hw.get(DistanceSensor.class, "distanceRight");
        else if(sensorDirection == SensorDirection.NONE) noSensor = true;
    }
    public int getTicks(){
        return this.ticks;
    }
    public double getDistanceWeight() {
        return sensorWeight;
    }
    public double getEncoderWeight() {
        return encoderWeight;
    }

    public double getDistance(){
        // double negative, ha
        if(direction == SensorDirection.BACK) return (selectedSensor.getDistance(DistanceUnit.INCH) / 12) * BaseMeccanum.TICKS_IN_FOOT;
        else if(direction == SensorDirection.LEFT || direction == SensorDirection.RIGHT) return (selectedSensor.getDistance(DistanceUnit.INCH) / 12) * BaseMeccanum.SIDEWAYS_TICKS_IN_FOOT;
        else return 0;
    }
    public boolean isWeightOverriden(){
        return weightOverride;
    }
    public boolean isNoSensor(){
        return noSensor;
    }

}
