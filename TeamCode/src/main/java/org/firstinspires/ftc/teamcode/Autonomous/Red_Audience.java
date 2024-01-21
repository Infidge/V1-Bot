package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Depositor;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name ="Red_Audience", group = "RED")
public class Red_Audience extends LinearOpMode {

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

    double startX = -90;
    double startY = -161.7;
    double startHeading = Math.toRadians(90);

    Randomisation randomisation = Randomisation.LEFT;

    TrajectorySequence goToBackdropLeft;
    TrajectorySequence goToBackdropMid;
    TrajectorySequence goToBackdropRight;

    TrajectorySequence goToParkLeft;
    TrajectorySequence goToParkMiddle;
    TrajectorySequence goToParkRight;

    Intake intake = new Intake();
    Depositor depo = new Depositor();

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
                .splineTo(new Vector2d(-110, -100), Math.toRadians(135))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-100, -140, Math.toRadians(0)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-145, -90), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-115, -35), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(15, -35))
                .addDisplacementMarker(() -> {
                    depo.toggleBuckets();
                    depo.raiseLiftOrPixelLevel();
                    depo.raiseLiftOrPixelLevel();
                })
                .lineToConstantHeading(new Vector2d(80, -35))
                .splineToConstantHeading(new Vector2d(120, -85), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(140, -85), Math.toRadians(0))
                .build();

        goToBackdropMid = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-90, -105), Math.toRadians(90))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-90,-150, Math.toRadians(180)))
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-150, -90), Math.toRadians(90))
                .splineTo(new Vector2d(-120, -30), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(90, -30))
                .addDisplacementMarker(() -> {
                    depo.toggleBuckets();
                    depo.raiseLiftOrPixelLevel();
                })
                .splineTo(new Vector2d(120, -75), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(140, -75))
                .build();

        goToBackdropRight = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-75, -105), Math.toRadians(45))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-90,-150, Math.toRadians(180)))
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-150, -90), Math.toRadians(90))
                .splineTo(new Vector2d(-120, -30), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(90, -30))
                .addDisplacementMarker(() -> {
                    depo.toggleBuckets();
                    depo.raiseLiftOrPixelLevel();
                })
                .splineTo(new Vector2d(120, -75), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(140, -75))
                .build();

        goToParkLeft = drive.trajectorySequenceBuilder(goToBackdropLeft.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(100, -85))
                .addDisplacementMarker(() -> {
                    depo.retractLift();
                    depo.toggleBuckets();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(150, -40), Math.toRadians(0))
                .build();

        goToParkMiddle = drive.trajectorySequenceBuilder(goToBackdropMid.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(120, -50))
                .addDisplacementMarker(() -> {
                    depo.retractLift();
                    depo.toggleBuckets();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(150, -40), Math.toRadians(0))
                .build();

        goToParkRight = drive.trajectorySequenceBuilder(goToBackdropRight.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(120, -50))
                .addDisplacementMarker(() -> {
                    depo.retractLift();
                    depo.toggleBuckets();
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(150, -40), Math.toRadians(0))
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
