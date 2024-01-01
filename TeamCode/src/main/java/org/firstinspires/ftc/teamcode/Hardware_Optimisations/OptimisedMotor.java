package org.firstinspires.ftc.teamcode.Hardware_Optimisations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class OptimisedMotor {

    DcMotorEx motor;
    double lastPower;
    double powerTolerance = 0.01;

    public OptimisedMotor(DcMotorEx motor){
        this.motor = motor;
    }

    public void setPower(double power){
        if (Math.abs(power - lastPower) > powerTolerance) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public double getLastPower(){
        return lastPower;
    }

    public void setName(String motorName, HardwareMap hwMap){
        motor = hwMap.get(DcMotorEx.class, motorName);
    }

    public void setMode(DcMotorEx.RunMode mode){
        motor.setMode(mode);
    }

    public void setDirection(DcMotorEx.Direction direction){
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public double getCurrent(){
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }
}
