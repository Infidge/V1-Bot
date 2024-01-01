package org.firstinspires.ftc.teamcode.PID_Classes;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PID_Coefficients {
    double p;
    double i;
    double d;

    public PID_Coefficients(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getP(){
        return p;
    }

    public double getI(){
        return i;
    }

    public double getD(){
        return d;
    }
}
