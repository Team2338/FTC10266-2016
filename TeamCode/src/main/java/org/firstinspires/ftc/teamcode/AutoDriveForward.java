/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
*/
package org.firstinspires.ftc.teamcode;
//
// AUTO, THIS CODE WORKS
//
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import static android.os.SystemClock.sleep;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoDriveForward", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled

public class AutoDriveForward extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    DcMotor intakeLowMotor;
    DcMotor intakeHighMotor;

    // NOT USED FOR STATE
    ColorSensor colorSensor;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   100;     // period of each cycle
    static final double MAX_POS     =  .50;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    Servo servo;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftDrive");
        backRightMotor = hardwareMap.dcMotor.get("backRightDrive");

        leftShooterMotor = hardwareMap.dcMotor.get("LeftShooter");
        rightShooterMotor = hardwareMap.dcMotor.get("RightShooter");

        intakeLowMotor = hardwareMap.dcMotor.get("collectorLow");
        intakeHighMotor = hardwareMap.dcMotor.get("collectorHigh");

        //odsSensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // NOT USED FOR STATE
        servo = hardwareMap.servo.get("servo1");

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    //TODO: Fix electrical
    public void start() // This code doesn't make sense, current code is used to compensate electrical issues
    {
            // NEED A LARGER POSITION SO TIME DOES NOT RUN OUT
            frontLeftMotor.setTargetPosition(17200);
            frontRightMotor.setTargetPosition(17200);

/*          frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //frontLeftMotor.setPower(0.25);
            //frontRightMotor.setPower(-0.25);

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);


        /*
        telemetry.addData(" rampUp1  ", rampUp);
        if (rampUp) {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            telemetry.addData(" MOVE IN  ", position);
            if (position <= MIN_POS ) {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }
        servo.setPosition(position);
        telemetry.addData(" INIT LOOP  ", position);
        */
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop()
    {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right Ticks", frontRightMotor.getCurrentPosition());

       // telemetry.addData("Status Before If", chuck_int);

        /*
        telemetry.addData("Servo Position ", position);
        telemetry.addData(" rampUp2  ", rampUp);


            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                telemetry.addData(" MOVE OUT  ", position);

                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                telemetry.addData(" MOVE IN  ", position);
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
*/
            // Display the current value
            //telemetry.addData("Servo Position", "%5.2f", position);
           // telemetry.addData(">", "Press Stop to end test." );
           // telemetry.update();

            // Set the servo to the new position and pause;
           // servo.setPosition(position);
//            sleep(CYCLE_MS);



            // USED TO JUST RUN THE INTAKE AND SHOOTER FOR PRACTICE
    /*    if ((runtime.seconds() >= 0) && (runtime.seconds() <= 52.50))
        {
            telemetry.addData("1 SPOOL SHOOTER ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

           // intakeLowMotor.setPower(1.00);
            //intakeHighMotor.setPower(1.00);

            //leftShooterMotor.setPower(1.00);
           // rightShooterMotor.setPower(-1.00);


        }
*/

        // Stationary, Spool up Shooters, No Intakes, 2.5 Seconds
        if ((runtime.seconds() >= 0) && (runtime.seconds() <= 2.50))
        {
            telemetry.addData("1 SPOOL SHOOTER ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(1.00);
            rightShooterMotor.setPower(-1.00);


        }
        // Stationary, Keep Shooter Running, Start Center, No Front Intake, 3 Seconds
        if ((runtime.seconds() >= 2.51) && (runtime.seconds() <= 5.50))
        {
            telemetry.addData("2 CENTER AND SHOOT ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(-1.00);

            leftShooterMotor.setPower(1.00);   // SHOOTER SPEED
            rightShooterMotor.setPower(-1.00); // SHOOTER SPEED



        }
        // Stop Intake and Shooter, Move to Vortex for 2.5 Seconds
        if ((runtime.seconds() >= 5.51) && (runtime.seconds() <= 8.00))
        {
            telemetry.addData("3 Move To Vortex ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(1.00);
            backLeftMotor.setPower(1.00);

            frontRightMotor.setPower(-1.00);
            backRightMotor.setPower(-1.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);


        }
        // Stop at Vortex for 2.5 Seconds
        if ((runtime.seconds() >= 8.01) && (runtime.seconds() <= 10.00))
        {
            telemetry.addData("4 Stop at Vortex ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);

        }
        // Move from Vortex for 2.0 Seconds to Beacon Turn
        if ((runtime.seconds() >= 10.01) && (runtime.seconds() <= 12.00))
        {
            telemetry.addData("5 Go To Beacon Turn ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(1.00);
            backLeftMotor.setPower(1.00);

            frontRightMotor.setPower(-1.00);
            backRightMotor.setPower(-1.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);

        }
        // Stop for 2.0 Seconds before Beacon Turn
        if ((runtime.seconds() >= 12.01) && (runtime.seconds() <= 14.00))
        {
            telemetry.addData("6 Stop for Beacon Turn ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);

        }
        // Turn toward Beacon for .3 Seconds at Half Speed
        if ((runtime.seconds() >= 14.00) && (runtime.seconds() <= 14.30))
        {
            telemetry.addData("5 Beacon Turn Half Speed ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.50);
            backRightMotor.setPower(0.50);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);

        }
        // Move to Beacon for 1.5 Seconds at Half Speed
        if ((runtime.seconds() >= 14.31) && (runtime.seconds() <= 15.80))
        {
            telemetry.addData("5 Go To Beacon Half Speed    ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.50);
            backLeftMotor.setPower(0.50);

            frontRightMotor.setPower(-0.50);
            backRightMotor.setPower(-0.50);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);

        }
        // STOP ROBOT
        if (runtime.seconds() >= 15.81)
        {
            telemetry.addData("STOP ALL ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            backLeftMotor.setPower(0.00);

            frontRightMotor.setPower(0.00);
            backRightMotor.setPower(0.00);

            intakeLowMotor.setPower(0.00);
            intakeHighMotor.setPower(0.00);

            leftShooterMotor.setPower(0.00);
            rightShooterMotor.setPower(0.00);

        }

        /*
        if ((runtime.seconds() >= 15) && (runtime.seconds() <= 29.99))
        {
            telemetry.addData("In IF Statement .5 ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(-0.5);

        }
        if ((runtime.seconds() >= 30) && (runtime.seconds() <= 44.99))
        {
            telemetry.addData("In IF Statement .75 ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.75);
            frontRightMotor.setPower(-0.75);

        }
        if (runtime.seconds() >= 45)
        {
            telemetry.addData("In IF Statement .00 ", runtime.seconds());
            telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Ticks",frontRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(0.00);
            frontRightMotor.setPower(0.00);

        }
        //      // if ((frontLeftMotor.getCurrentPosition() >= 5199))
        */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop()
    {
        telemetry.addData("Status", "STOPPED");

    }
}
