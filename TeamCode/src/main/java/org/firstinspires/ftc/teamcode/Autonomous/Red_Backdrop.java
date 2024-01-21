package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Depositor;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name ="Red_Backdrop", group = "RED")
public class Red_Backdrop extends LinearOpMode {

    enum AutoStages {
        detect,
        goToBackdrop,
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

    double startX = 30.0;
    double startY = -161.7;
    double startHeading = Math.toRadians(90.0);

    Intake intake = new Intake();
    Depositor depo = new Depositor();

    TrajectorySequence goToBackdropLeft;
    TrajectorySequence goToBackdropMid;
    TrajectorySequence goToBackdropRight;

    TrajectorySequence goToParkLeft;
    TrajectorySequence goToParkMiddle;
    TrajectorySequence goToParkRight;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));

        intake.init(hardwareMap);
        depo.init(hardwareMap);

        generatePaths(drive);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            intake.update();
            depo.updateAuton();
            switch (currentState) {
                case detect:
                    chooseBackdropPath(drive);
                    currentState = AutoStages.goToBackdrop;
                    break;
                case goToBackdrop:
                    if (!drive.isBusy()) {
                        chooseParkPath(drive);
                        currentState = AutoStages.goToPark;
                    }
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
        goToBackdropLeft =  drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(10, -105), Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    depo.toggleBuckets();
                    depo.raiseLiftOrPixelLevel();
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(110, -85, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(145, -85), Math.toRadians(0))
                .build();

        goToBackdropMid = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(30, -85), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    depo.toggleBuckets();
                    depo.raiseLiftOrPixelLevel();
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(110, -100, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(145, -100), Math.toRadians(0))
                .build();

        goToBackdropRight = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(47, -107), Math.toRadians(45))
                .setTangent(Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    depo.toggleBuckets();
                    depo.raiseLiftOrPixelLevel();
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(30, -140, Math.toRadians(0)), Math.toRadians(0))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(120, -117), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(145, -117), Math.toRadians(0))
                .build();

        goToParkLeft = drive.trajectorySequenceBuilder(goToBackdropLeft.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(120, -85))
                .lineToConstantHeading(new Vector2d(90, -150))
                .addDisplacementMarker(() -> {
                    depo.retractLift();
                    depo.toggleBuckets();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(170, -170), Math.toRadians(0))
                .build();

        goToParkMiddle = drive.trajectorySequenceBuilder(goToBackdropMid.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(120, -85))
                .lineToConstantHeading(new Vector2d(90, -150))
                .addDisplacementMarker(() -> {
                    depo.retractLift();
                    depo.toggleBuckets();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(170, -170), Math.toRadians(0))
                .build();

        goToParkRight = drive.trajectorySequenceBuilder(goToBackdropRight.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(90, -150))
                .addDisplacementMarker(() -> {
                    depo.retractLift();
                    depo.toggleBuckets();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(170, -170), Math.toRadians(0))
                .build();

    }

    void chooseBackdropPath(SampleMecanumDrive drive) {
        if (randomisation == Randomisation.LEFT)
            drive.followTrajectorySequenceAsync(goToBackdropLeft);
        else if (randomisation == Randomisation.MIDDLE)
            drive.followTrajectorySequenceAsync(goToBackdropMid);
        else drive.followTrajectorySequenceAsync(goToBackdropRight);
    }

    void chooseParkPath(SampleMecanumDrive drive) {
        if (randomisation == Randomisation.LEFT)
            drive.followTrajectorySequenceAsync(goToParkLeft);
        else if (randomisation == Randomisation.MIDDLE)
            drive.followTrajectorySequenceAsync(goToParkMiddle);
        else drive.followTrajectorySequenceAsync(goToParkRight);
    }
}
