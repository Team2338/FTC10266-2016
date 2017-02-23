/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
//
//
//Carson Auto
//
//
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="AutoCenterAndBeaconBlue", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutoCenterPark extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor colorSensor;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    DcMotor intakeLowMotor;
    DcMotor intakeHighMotor;

    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    Servo servo2;
    Servo servo3;
    double position;
    final double INCREMENT = 0.01;
    final double MAX_POS = 0.5;
    final double MIN_POS = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftDrive");
        backRightMotor = hardwareMap.dcMotor.get("backRightDrive");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");

        leftShooterMotor = hardwareMap.dcMotor.get("LeftShooter");
        rightShooterMotor = hardwareMap.dcMotor.get("RightShooter");

        intakeLowMotor = hardwareMap.dcMotor.get("collectorLow");
        intakeHighMotor = hardwareMap.dcMotor.get("collectorHigh");

        servo2 = hardwareMap.servo.get("servo2");  //actuator 1
        servo3 = hardwareMap.servo.get("servo3"); //actuator 2


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        position = 0;


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        colorSensor.enableLed(true);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
            */

    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right Ticks", frontRightMotor.getCurrentPosition());

        if (runtime.seconds() <= 3) {
            telemetry.addData("If 1",1);
            leftShooterMotor.setPower(1);
            rightShooterMotor.setPower(-1);
        }

        else if (runtime.seconds() <= 7) {
            telemetry.addData("In If 2",1);
            intakeHighMotor.setPower(-1);
        }

        else if (runtime.seconds() <= 10) {
            telemetry.addData("In If 3",1);
            intakeHighMotor.setPower(0);
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);

            frontRightMotor.setPower(-1);
            backRightMotor.setPower(-1);
            frontLeftMotor.setPower(1);
            backLeftMotor.setPower(1);
        }

        else if (runtime.seconds() <= 12.5) {
            telemetry.addData("In 3rd If", 1);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        }

        else if (runtime.seconds() <= 14.0) {
            frontRightMotor.setPower(1);
            backRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            backLeftMotor.setPower(1);
        }

        else if (runtime.seconds() <= 16.5) {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);   // WASWRONG
        }

        else if (runtime.seconds() <= 20) {
            frontRightMotor.setPower(1);
            backRightMotor.setPower(1);
            frontLeftMotor.setPower(-1);
            backLeftMotor.setPower(-1);
        }

        else {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            backRightMotor.setPower(0);  // WRONG
            frontLeftMotor.setPower(0);

            if (colorSensor.blue() >= 3) actuator(servo2, false);
            else if (colorSensor.red() >= 3) actuator(servo3, false);
        }

    }

    private void actuator(Servo servo, boolean in) { //method to make actuators work in auto
        if (in) {
            position -= INCREMENT;
            if (position < MIN_POS) position = MIN_POS;
        } else {
            position += INCREMENT;
            if (position > MAX_POS) position = MAX_POS;
        }

        servo.setPosition(position);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "STOPPED");
    }
}


