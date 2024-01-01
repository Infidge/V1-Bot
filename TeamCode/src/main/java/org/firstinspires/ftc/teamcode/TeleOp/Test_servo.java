package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test_servo", group="TeleOp")
public class Test_servo extends LinearOpMode {

    Servo servo1, servo2;

    @Override
    public void runOpMode(){

        servo1 = hardwareMap.get(Servo.class, "right_bucket");
        servo2 = hardwareMap.get(Servo.class, "right_latch");

        waitForStart();

        while (opModeIsActive()){
            servo1.setPosition(1.0);
        }
    }
}
