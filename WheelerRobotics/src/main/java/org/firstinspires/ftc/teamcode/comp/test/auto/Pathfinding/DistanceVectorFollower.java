package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;

import static java.lang.Math.abs;

public class DistanceVectorFollower {
    private Mechanum robo;
    private DistanceVector2 dv;
    private double completedness = 0;
    private boolean paused = false;
    private Point startPos;
    private Point endPos;
    private Point currentPos;

    public DistanceVectorFollower(DistanceVector2 dv, Mechanum robo) {
        this.robo = robo;
        this.startPos = robo.getPosition();
        this.endPos = new Point(robo.getPosition().x() + startPos.x() - endPos.x() , robo.getPosition().y() + abs(startPos.y() - endPos.y()));
        this.dv = dv;
    }
    public void updateCurrentPosition(){
        this.currentPos = robo.getPosition();
    }
    public double follow(){
        // drive robot angle
        if(!paused) {
            SlopeVector2d cv = dv.sVector2d();
            //robo.driveRobotSVector(cv);
            completedness = MathUtil.distanceForumula(startPos, currentPos)/ dv.distance();
        }else{
            //robo.driveStop();
        }

        return completedness;
    }
    public double pause(){
        this.paused = true;
        return completedness;
    }
    public double unpause(){
        this.paused = false;
        return completedness;
    }
    public double togglepause(){
        this.paused = !this.paused;
        return completedness;
    }
    public double completedness(){
        return this.completedness;
    }


}
