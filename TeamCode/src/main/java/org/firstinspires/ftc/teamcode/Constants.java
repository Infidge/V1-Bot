package org.firstinspires.ftc.teamcode;

public class Constants {

    /**Depositor constants*/
    public static double left_horizontal_extension_in = 0.00;
    public static double right_horizontal_extension_in = 1.0 - left_horizontal_extension_in;
    public static double left_horizontal_extension_out = 0.23;
    public static double right_horizontal_extension_out = 1.0 - left_horizontal_extension_out;

    public static double left_depo_arm_in = 0.885;
    public static double right_depo_arm_in = 0.085;
    public static double left_depo_arm_out = 0.55;
    public static double right_depo_arm_out = 0.42;

    public static double left_latch_closed = 0.7;
    public static double right_latch_closed = 0.13;
    public static double left_latch_open = 0.4;
    public static double right_latch_open = 0.35;

    public static int pixel_1_position = 60;
    public static int pixel_level_increment = 100;
    
    /**Intake constants*/
    public static double left_intake_collect = 0.995;
    public static double right_intake_collect = 1.0 - left_intake_collect;
    public static double left_intake_3_stack = 0.0;
    public static double right_intake_3_stack = 1.0 - left_intake_3_stack;
    public static double left_intake_5_stack = 0.0;
    public static double right_intake_5_stack = 1.0 - left_intake_5_stack;
    public static double left_intake_transfer = 0.45;
    public static double right_intake_transfer = 1.0 - left_intake_transfer;

    public static double intake_collect_power = 1.0;
    public static double intake_reverse_power = -0.7;

    /**Launcher constants*/
    public static double trigger_loaded = 0.0;
    public static double trigger_released = 0.0;
}
