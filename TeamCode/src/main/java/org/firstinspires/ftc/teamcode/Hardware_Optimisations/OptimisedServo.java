package org.firstinspires.ftc.teamcode.Hardware_Optimisations;

import static org.firstinspires.ftc.teamcode.Utils.toAbsolutePosition;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OptimisedServo {

    Servo servo;
    double lastPosition = 2.0;
    double posTolerance = 0.0000001;
    AnalogInput encoder;

    public OptimisedServo(Servo servo){
        this.servo = servo;
    }

    public OptimisedServo(AnalogInput encoder){
        this.encoder = encoder;
    }

    public void setPosition(double position){
        if (Math.abs(position - lastPosition) > posTolerance) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    public void setName(String name, HardwareMap hwMap){
        servo = hwMap.get(Servo.class, name);
    }

    public double getPosition(){
        return servo.getPosition();
    }

    public double getLastPosition(){
        return lastPosition;
    }
}
