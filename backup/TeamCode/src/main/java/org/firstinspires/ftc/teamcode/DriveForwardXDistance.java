package org.firstinspires.ftc.teamcode;

/**
 * Created by Sean on 10/26/2016.
 */

public class DriveForwardXDistance {
    private final static int ENCODER_CPR = 1120;
    private final static double GEAR_RATIO = 40;
    private final static int WHEEL_DIAMETER = 4;
    private static double Dist = 0;
    private final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; //Inches
    private final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    private final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;


    public void driveWithEncoder (double distance) {
            final double Calculation = 3566.87898 * distance;

    }
}
