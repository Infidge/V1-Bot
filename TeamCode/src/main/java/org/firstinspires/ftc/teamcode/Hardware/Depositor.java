package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.LimitSwitch;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedMotor;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedServo;
import org.firstinspires.ftc.teamcode.PID_Classes.PID_Coefficients;
import org.firstinspires.ftc.teamcode.PID_Classes.PID_Controller;

@Config
public class Depositor{
    DcMotorEx lSlide, rSlide;
    public OptimisedMotor leftSlide = new OptimisedMotor(lSlide);
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

    public static int LIFT_TARGET_POSITION = 0;
    public static double P = 0.015;
    public static double I = 0.0;
    public static double D = 0.005;
    public static PID_Coefficients pid = new PID_Coefficients(P, I, D);
    PID_Controller liftController = new PID_Controller(pid);
    LiftStates liftState = LiftStates.IDLE;
    int liftPixelLevel = 1;

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

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

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
        IDLE,
        RETRACT,
        SCORING
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
            return currentPos + 0.0035 * Math.signum(error);
        else  if (Math.abs(error) > 0.07)
            return currentPos + 0.004 * Math.signum(error);
        else return target;
    }

    public void retractLift(){
        liftState = LiftStates.RETRACT;
    }

    public void raiseLiftOrPixelLevel() {
        if (liftState == LiftStates.SCORING) {
            liftPixelLevel += 2;
            liftPixelLevel = Range.clip(liftPixelLevel, 1, 11);
        } else {
            liftState = LiftStates.SCORING;
        }
    }

    public void lowerPixelLevel() {
        liftPixelLevel -= 2;
        liftPixelLevel = Range.clip(liftPixelLevel, 1, 11);
    }

    public int liftTargetPosition = 0;

    public void update(){
        liftController.setCoefficients(new PID_Coefficients(P, I, D));
        if (liftState == LiftStates.SCORING) {
            liftTargetPosition = Constants.pixel_1_position + (liftPixelLevel - 1) * Constants.pixel_level_increment;
            leftSlide.setPower(liftController.update(leftSlide.getCurrentPosition(), liftTargetPosition));
            rightSlide.setPower(liftController.update(leftSlide.getCurrentPosition(), liftTargetPosition));
        } else if (liftState == LiftStates.RETRACT) {
            if (limitSwitch.isPressed()) {
                liftState = LiftStates.IDLE;
                leftSlide.setPower(0.0);
                rightSlide.setPower(0.0);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                leftSlide.setPower(-1.0);
                rightSlide.setPower(-1.0);
            }
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
