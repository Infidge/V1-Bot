package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Controls;
import org.firstinspires.ftc.teamcode.Hardware.Depositor;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Intake;

@TeleOp(name="TeleOp", group="TeleOp")
public class Tele_Op extends LinearOpMode {

    Drivetrain dt = new Drivetrain();
    Intake intake = new Intake();
    Depositor depo = new Depositor();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    ElapsedTime intakeToggleTime = new ElapsedTime();
    //Controls control = new Controls();

    @Override
    public void runOpMode(){

        dt.init(hardwareMap);
        intake.init(hardwareMap);
        depo.init(hardwareMap);
        //launcher.init(hardwareMap);

        currentGamepad1.copy(gamepad1);
        previousGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        previousGamepad2.copy(gamepad2);

        intakeToggleTime.reset();

        waitForStart();
        
        while (opModeIsActive()){
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a && !previousGamepad1.a)
                intake.toggleBucket();

            if (currentGamepad1.right_bumper)
                intake.turnOn();
            else if (currentGamepad1.left_bumper)
                intake.reverse();
            else
                intake.turnOff();

            intake.autoRaiseBucket();

            if (currentGamepad2.b && !previousGamepad2.b)
                depo.toggleBuckets();

            if (currentGamepad2.a && !previousGamepad2.a)
                depo.toggleLatches();

            if (currentGamepad2.left_bumper) {
                depo.retractLift();
            }

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                depo.raiseLiftOrPixelLevel();
            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                depo.lowerPixelLevel();
            }

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            
            dt.mecanumDrive(gamepad1);
            intake.update();
            depo.update();

            //launcher.update();
        }
    }
}
