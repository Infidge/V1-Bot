package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Depositor;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.BluePropDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name ="Blue_Backdrop_Park", group = "BLUE")
public class Blue_Backdrop_Park extends LinearOpMode {

    enum AutoStages {
        detect,
        goToPark,
        stop
    }

    enum Randomisation {
        LEFT,
        MIDDLE,
        RIGHT
    }


    AutoStages currentState = AutoStages.detect;

    Randomisation randomisation = Randomisation.LEFT;

    ElapsedTime detectionTime = new ElapsedTime();

    double startX = 30.0;
    double startY = 161.7;
    double startHeading = Math.toRadians(270.0);

    Intake intake = new Intake();
    Depositor depo = new Depositor();

    TrajectorySequence goToParkLeft;
    TrajectorySequence goToParkMiddle;
    TrajectorySequence goToParkRight;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));

        intake.init(hardwareMap);
        depo.init(hardwareMap);

        BluePropDetection detectProp = new BluePropDetection(hardwareMap, telemetry);
        BluePropDetection.ElementPosition position;

        generatePaths(drive);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            intake.update();
            depo.updateAuton();
            switch (currentState) {
                case detect:
                    detectionTime.reset();
                    while (detectionTime.seconds() < 0.2) {
                        position = detectProp.getElementPosition();
                        if (position == BluePropDetection.ElementPosition.LEFT)
                            randomisation = Randomisation.LEFT;
                        else if (position == BluePropDetection.ElementPosition.MIDDLE)
                            randomisation = Randomisation.MIDDLE;
                        else if (position == BluePropDetection.ElementPosition.RIGHT)
                            randomisation = Randomisation.RIGHT;
                        else randomisation = Randomisation.RIGHT;
                    }
                    detectProp.stopCamera();
                    chooseParkPath(drive);
                    currentState = AutoStages.goToPark;
                    break;
                case goToPark:
                    if (!drive.isBusy())
                        currentState = AutoStages.stop;
                    break;
                case stop:
                    terminateOpModeNow();
                    break;
                default:
                    break;
            }
        }
    }

    void generatePaths(SampleMecanumDrive drive)
    {
        goToParkLeft =  drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(47, 105), Math.toRadians(315))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(30, 150, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(120, 150, Math.toRadians(270)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(153, 153), Math.toRadians(270))
                .build();

        goToParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(30, 85), Math.toRadians(270))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(120, 150, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(153, 153), Math.toRadians(270))
                .build();

        goToParkRight = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(10, 107), Math.toRadians(225))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(120, 150, Math.toRadians(270)), Math.toRadians(270))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(153, 153), Math.toRadians(270))
                .build();
    }

    void chooseParkPath(SampleMecanumDrive drive) {
        if (randomisation == Randomisation.LEFT)
            drive.followTrajectorySequenceAsync(goToParkLeft);
        else if (randomisation == Randomisation.MIDDLE)
            drive.followTrajectorySequenceAsync(goToParkMiddle);
        else drive.followTrajectorySequenceAsync(goToParkRight);
    }
}
