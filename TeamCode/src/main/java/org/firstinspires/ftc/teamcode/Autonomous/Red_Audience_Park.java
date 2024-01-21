package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Depositor;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.RedPropDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name ="Red_Audience_Park", group = "RED")
public class Red_Audience_Park extends LinearOpMode {

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

    double startX = -90;
    double startY = -161.7;
    double startHeading = Math.toRadians(90);

    Randomisation randomisation = Randomisation.RIGHT;

    ElapsedTime detectionTime = new ElapsedTime();

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

        RedPropDetection detectProp = new RedPropDetection(hardwareMap, telemetry);
        RedPropDetection.ElementPosition position;

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
                        if (position == RedPropDetection.ElementPosition.LEFT)
                            randomisation = Randomisation.LEFT;
                        else if (position == RedPropDetection.ElementPosition.MIDDLE)
                            randomisation = Randomisation.MIDDLE;
                        else if (position == RedPropDetection.ElementPosition.RIGHT)
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
                .splineTo(new Vector2d(-113, -100), Math.toRadians(135))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-100, -140, Math.toRadians(90)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-150, -90), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-150, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-120, -35), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(155, -30))
                .build();

        goToParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .splineTo(new Vector2d(-85, -80), Math.toRadians(90))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-100, -140, Math.toRadians(90)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-150, -90), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-150, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-120, -35), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(155, -30))
                .build();

        goToParkRight = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .splineTo(new Vector2d(-73, -107), Math.toRadians(45))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-100, -140, Math.toRadians(90)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-150, -90), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-150, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-120, -35), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(155, -30))
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
