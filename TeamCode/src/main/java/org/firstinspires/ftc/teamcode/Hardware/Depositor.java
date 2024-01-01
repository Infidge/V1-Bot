package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.LimitSwitch;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedMotor;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedServo;
import org.firstinspires.ftc.teamcode.PID_Classes.PID_Coefficients;
import org.firstinspires.ftc.teamcode.PID_Classes.PID_Controller;
import org.opencv.core.Mat;

public class Depositor{
    DcMotorEx lSlide, rSlide;
    OptimisedMotor leftSlide = new OptimisedMotor(lSlide);
    OptimisedMotor rightSlide = new OptimisedMotor(rSlide);

    Servo lExt, rExt, lBuck, rBuck, lLatch, rLatch;
    OptimisedServo leftExtension = new OptimisedServo(lExt);
    OptimisedServo rightExtension = new OptimisedServo(rExt);
    OptimisedServo leftBucket = new OptimisedServo(lBuck);
    OptimisedServo rightBucket = new OptimisedServo(rBuck);
    OptimisedServo leftLatch = new OptimisedServo(lLatch);
    OptimisedServo rightLatch = new OptimisedServo(rLatch);

    DigitalChannel lSwitch;
    LimitSwitch limitSwitch = new LimitSwitch(lSwitch);

    PID_Coefficients pid = new PID_Coefficients(0.5,0.0,0.2);
    PID_Controller liftController = new PID_Controller(pid);

    LiftStates liftState = LiftStates.IDLE;
    LeftArmStates leftArmState = LeftArmStates.IN;
    RightArmStates rightArmState = RightArmStates.IN;
    LeftExtensionStates leftExtensionState = LeftExtensionStates.IN;
    RightExtensionStates rightExtensionState = RightExtensionStates.IN;
    LeftLatchStates leftLatchState = LeftLatchStates.CLOSED;
    RightLatchStates rightLatchState = RightLatchStates.CLOSED;

    boolean toggling = false;
    boolean goOut = false;

    ElapsedTime extensionDelay = new ElapsedTime();

    public Depositor(){
    }

    public void init (HardwareMap hwMap){
        leftSlide.setName("left_slide", hwMap);
        rightSlide.setName("right_slide", hwMap);

        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftExtension.setName("left_extension", hwMap);
        rightExtension.setName("right_extension", hwMap);
        leftBucket.setName("left_bucket", hwMap);
        rightBucket.setName("right_bucket", hwMap);
        leftLatch.setName("left_latch", hwMap);
        rightLatch.setName("right_latch", hwMap);

        //leftExtension.setPosition(leftExtensionState.get());
        //rightExtension.setPosition(rightExtensionState.get());
        leftBucket.setPosition(leftArmState.getArmPos());
        rightBucket.setPosition(rightArmState.getArmPos());
        leftLatch.setPosition(leftLatchState.getLatchPos());
        rightLatch.setPosition(rightLatchState.getLatchPos());

        limitSwitch.setName("limit_switch", hwMap);

    }

    public enum LeftArmStates{
        IN(Constants.left_depo_arm_in),
        OUT(Constants.left_depo_arm_out);

        double armPos;

        LeftArmStates(double armPos){
            this.armPos = armPos;
        }

        double getArmPos(){
            return armPos;
        }

    }

    public enum RightArmStates{
        IN(Constants.right_depo_arm_in),
        OUT(Constants.right_depo_arm_out);

        double armPos;

        RightArmStates(double armPos){
            this.armPos = armPos;
        }

        double getArmPos(){
            return armPos;
        }
    }
    
    public enum LeftLatchStates {
        CLOSED (Constants.left_latch_closed),
        OPEN (Constants.left_latch_open);
        
        double latchPos;
        
        LeftLatchStates(double latchPos) {
            this.latchPos = latchPos;
        }
        
        double getLatchPos() {
            return latchPos;
        }
    }
    
    public enum RightLatchStates {
        CLOSED(Constants.right_latch_closed),
        OPEN(Constants.right_latch_open);
        
        double latchPos;
        
        RightLatchStates(double latchPos) {
            this.latchPos = latchPos;
        }
        
        double getLatchPos() {
            return latchPos;
        }
    }

    public enum LeftExtensionStates {
        IN(Constants.left_horizontal_extension_in),
        OUT(Constants.left_horizontal_extension_out);

        double pos;

        LeftExtensionStates(double pos){
            this.pos = pos;
        }

        double get(){
            return pos;
        }
    }

    public enum RightExtensionStates {
        IN(Constants.right_horizontal_extension_in),
        OUT(Constants.right_horizontal_extension_out);

        double pos;

        RightExtensionStates(double pos){
            this.pos = pos;
        }

        double get(){
            return pos;
        }
    }

    public enum LiftStates{
        IDLE(-1),
        RETRACT(0),
        PIXEL1(Constants.pixel_1_position),
        PIXEL2(Constants.pixel_1_position + Constants.pixel_level_increment),
        PIXEL3(Constants.pixel_1_position + Constants.pixel_level_increment * 2),
        PIXEL4(Constants.pixel_1_position + Constants.pixel_level_increment * 3),
        PIXEL5(Constants.pixel_1_position + Constants.pixel_level_increment * 4),
        PIXEL6(Constants.pixel_1_position + Constants.pixel_level_increment * 5),
        PIXEL7(Constants.pixel_1_position + Constants.pixel_level_increment * 6),
        PIXEL8(Constants.pixel_1_position + Constants.pixel_level_increment * 7),
        PIXEL9(Constants.pixel_1_position + Constants.pixel_level_increment * 8),
        PIXEL10(Constants.pixel_1_position + Constants.pixel_level_increment * 9),
        PIXEL11(Constants.pixel_1_position + Constants.pixel_level_increment * 10);

        int pos;

        LiftStates(int pos){
            this.pos = pos;
        }

        int get(){
            return pos;
        }
    }
    
    public void toggleLatches() {
        leftLatchState = (leftLatchState == LeftLatchStates.CLOSED) ? LeftLatchStates.OPEN : LeftLatchStates.CLOSED;
        rightLatchState = (rightLatchState == RightLatchStates.CLOSED) ? RightLatchStates.OPEN : RightLatchStates.CLOSED;
    }

    public void toggleBuckets() {
        if (leftExtensionState == LeftExtensionStates.OUT) {
            leftExtensionState = LeftExtensionStates.IN;
            rightExtensionState = RightExtensionStates.IN;
            goOut = false;
        }
        else {
            leftArmState = LeftArmStates.OUT;
            rightArmState = RightArmStates.OUT;
            goOut = true;
        }
        extensionDelay.reset();
        toggling = true;
    }

    public void updateToggle() {
        if (extensionDelay.seconds() > 0.3 && toggling){
            if (!goOut) {
                leftArmState = LeftArmStates.IN;
                rightArmState = RightArmStates.IN;
            }
            else {
                leftExtensionState = LeftExtensionStates.OUT;
                rightExtensionState = RightExtensionStates.OUT;
            }
            toggling = false;
        }
    }

    public double servoPCOntroller(double currentPos, double target) {
        double error = target - currentPos;
        if (Math.abs(error) > 0.13)
            return currentPos + 0.0025 * Math.signum(error);
        else return currentPos + 0.003 * Math.signum(error);
    }

    public void update(){
        if (liftState != LiftStates.IDLE && liftState != LiftStates.RETRACT) {
            leftSlide.setPower(liftController.update(leftSlide.getCurrentPosition(), liftState.get()));
            rightSlide.setPower(liftController.update(leftSlide.getCurrentPosition(), liftState.get()));
        }
        else if (liftState == LiftStates.IDLE) {
            leftSlide.setPower(0.0);
            rightSlide.setPower(0.0);
        }
        leftBucket.setPosition(servoPCOntroller(leftBucket.getPosition(), leftArmState.getArmPos()));
        leftLatch.setPosition(leftLatchState.getLatchPos());
        rightBucket.setPosition(servoPCOntroller(rightBucket.getPosition(), rightArmState.getArmPos()));
        rightLatch.setPosition(rightLatchState.getLatchPos());
        updateToggle();
        leftExtension.setPosition(leftExtensionState.get());
        rightExtension.setPosition(rightExtensionState.get());
    }
}
