package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedServo;

public class Drone_Launcher {

    OptimisedServo drone_trigger;

    launcherStates launcherState = launcherStates.HOLD;

    public Drone_Launcher(){
    }

    public void init(HardwareMap hwMap){
        drone_trigger.setName("launcher_trigger", hwMap);
        drone_trigger.setPosition(drone_trigger.getLastPosition());
    }

    public enum launcherStates{
        HOLD(Constants.trigger_loaded),
        RELEASE(Constants.trigger_released);

        double pos;

        launcherStates (double pos){
            this.pos = pos;
        }

        double get(){
            return pos;
        }
    }

    public void update(){
        drone_trigger.setPosition(launcherState.get());
    }
}
