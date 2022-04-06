package org.firstinspires.ftc.teamcode.comp.chassis;

public class Wheelz {
    double[] vector;
    void Wheelz(double fl, double fr, double bl, double br){
        this.vector = new double[]{fl, fr, bl, br};
    }
    void Wheelz(double[] vector){
        this.vector = vector;
    }
    void add(Wheelz addee) {
    }
}
