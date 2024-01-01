package org.firstinspires.ftc.teamcode.Hardware_Optimisations;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Utils.toAbsolutePosition;

public class OptimisedCRServo {

    CRServo crservo;
    double lastPower;
    double powerTolerance = 0.01;
    AnalogInput encoder = null;

    public OptimisedCRServo(CRServo crservo, double initPower){
        this.crservo = crservo;
        this.lastPower = initPower;
    }

    public OptimisedCRServo(CRServo crservo, double initPower, AnalogInput encoder){
        this.crservo = crservo;
        this.lastPower = initPower;
        this.encoder = encoder;
    }

    public void setPower(double power){
        if (Math.abs(power - lastPower) > powerTolerance) {
            crservo.setPower(power);
            lastPower = power;
        }
    }

    public double getPower(){
        return crservo.getPower();
    }

    public void setDirection(CRServo.Direction direction){
        crservo.setDirection(direction);
    }

    public double getPosition(){
        if (encoder != null)
            return toAbsolutePosition(encoder.getVoltage());
        else return 0;
    }
}
