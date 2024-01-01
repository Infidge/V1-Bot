package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedMotor;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedServo;

public class Intake {

    DcMotorEx intakeMotor;
    OptimisedMotor intakeSpinners = new OptimisedMotor(intakeMotor);

    Servo leftIntake;
    OptimisedServo leftIntakeAngle = new OptimisedServo(leftIntake);
    Servo rightIntake;
    OptimisedServo rightIntakeAngle = new OptimisedServo(rightIntake);

    RevColorSensorV3 leftSensor;
    RevColorSensorV3 rightSensor;

    IntakeStates intakeState = IntakeStates.TRANSFER;
    IntakeSpinnerStates intakeSpinnerState = IntakeSpinnerStates.STOP;
    IntakePixelCount intakePixels = IntakePixelCount.EMPTY;

    boolean autoRaise = false;

    ElapsedTime readTime = new ElapsedTime();

    public Intake(){
    }

    /**INIT*/
    public void init(HardwareMap hwMap){
        intakeSpinners.setName("intakeMotor", hwMap);

        intakeSpinners.setPower(intakeSpinnerState.get());

        intakeSpinners.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeSpinners.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeSpinners.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeAngle.setName("left_intake_angle", hwMap);
        rightIntakeAngle.setName("right_intake_angle", hwMap);

        leftIntakeAngle.setPosition(intakeState.getLeftPos());
        rightIntakeAngle.setPosition(intakeState.getRightPos());

        leftSensor = hwMap.get(RevColorSensorV3.class, "left_sensor");
        rightSensor = hwMap.get(RevColorSensorV3.class, "right_sensor");

        readTime.reset();
    }

    /**STATE ENUMS*/
    public enum IntakeStates{
        TRANSFER(Constants.left_intake_transfer, Constants.right_intake_transfer),
        COLLECT(Constants.left_intake_collect, Constants.right_intake_collect),
        STACK_OF_3(Constants.left_intake_3_stack, Constants.right_intake_3_stack),
        STACK_OF_5(Constants.left_intake_5_stack, Constants.right_intake_5_stack);

        double leftPos;
        double rightPos;

        IntakeStates(double leftPos, double rightPos){
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
        double getLeftPos(){
            return leftPos;
        }

        double getRightPos(){
            return rightPos;
        }
    }

    public enum IntakeSpinnerStates{
        STOP(0.0),
        COLLECT(Constants.intake_collect_power),
        REVERSE(Constants.intake_reverse_power);

        double power;

        IntakeSpinnerStates(double power){
            this.power = power;
        }

        double get(){
            return power;
        }
    }

    public enum IntakePixelCount{
        EMPTY,
        LEFT,
        RIGHT,
        FULL;
    }

    public void toggleBucket() {
        intakeState = (intakeState == IntakeStates.COLLECT) ? IntakeStates.TRANSFER : IntakeStates.COLLECT;
        autoRaise = false;
    }

    public void autoRaiseBucket() {
        if (intakePixels == IntakePixelCount.FULL && autoRaise)
             intakeState = IntakeStates.TRANSFER;
    }

    public void turnOn() {
        intakeSpinnerState = IntakeSpinnerStates.COLLECT;
        autoRaise = true;
    }

    public void turnOff() {
        intakeSpinnerState = IntakeSpinnerStates.STOP;
    }

    public void reverse() {
        intakeSpinnerState = IntakeSpinnerStates.REVERSE;
    }


    public void update() {
        intakeSpinners.setPower(intakeSpinnerState.get());
        leftIntakeAngle.setPosition(intakeState.getLeftPos());
        rightIntakeAngle.setPosition(intakeState.getRightPos());

        if (readTime.seconds() > 0.150) {
            double interL, interR;
            interL = leftSensor.getDistance(DistanceUnit.CM);
            interR = rightSensor.getDistance(DistanceUnit.CM);

            if (interL < 1.5 && interR < 1.5)
                intakePixels = IntakePixelCount.FULL;
            else if (interL < 1.5)
                intakePixels = IntakePixelCount.LEFT;
            else if (interR < 1.5)
                intakePixels = IntakePixelCount.RIGHT;
            else
                intakePixels = IntakePixelCount.EMPTY;

            readTime.reset();
        }
    }
}
