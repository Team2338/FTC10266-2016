package org.firstinspires.ftc.teamcode;

/**
 * Created by CarsonM on 11/2/2016.
 */

public class DriveEncoder {
    private final static int ENCODER_CPR = 1120;
    private final static double GEAR_RATIO = 40;
    private final static int WHEEL_DIAMETER = 4;
    private static double DISTANCE = 0;
    private final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final static double COUNTS = ENCODER_CPR * GEAR_RATIO;

    public double driveWithEncoder(double distance) {
        double calculation = (COUNTS / CIRCUMFERENCE) * distance; //ticks per inch
        return calculation;
    }
}