package org.firstinspires.ftc.teamcode.Hardware_Optimisations;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {

    DigitalChannel limitSwitch;

    public LimitSwitch(DigitalChannel limitSwitch){
        this.limitSwitch = limitSwitch;
    }

    public boolean isPressed(){
        return limitSwitch.getState();
    }

    public void setName(String name, HardwareMap hwMap){
        limitSwitch = hwMap.get(DigitalChannel.class, name);
    }
}
