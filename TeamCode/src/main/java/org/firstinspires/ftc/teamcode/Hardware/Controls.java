package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controls
{

    Gamepad currentGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;

    public Controls(Gamepad gamepad11, Gamepad gamepad12, Gamepad gamepad21, Gamepad gamepad22) {
        this.currentGamepad1 = gamepad11;
        this.previousGamepad1 = gamepad12;
        this.currentGamepad2 = gamepad21;
        this.previousGamepad2 = gamepad22;
    }

    public void init(Gamepad gamepad1, Gamepad gamepad2)
    {
        currentGamepad1.copy(gamepad1);
        previousGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        previousGamepad2.copy(gamepad2);
    }

    public void updateGamepad1(Gamepad gamepad1, Intake intake)
    {
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.a && !previousGamepad1.a)
            intake.toggleBucket();

        if (currentGamepad1.right_bumper)
            intake.turnOn();
        else if (currentGamepad1.left_bumper)
            intake.reverse();
        else
            intake.turnOff();

        previousGamepad1.copy(currentGamepad1);
    }

    public void updateGamepad2 (Gamepad gamepad2, Depositor depositor)
    {
        currentGamepad2.copy(gamepad2);

        if (currentGamepad2.a && !previousGamepad2.a)
            depositor.toggleLatches();

        if (currentGamepad2.b && !previousGamepad2.b)
            depositor.toggleBuckets();
    }
}
