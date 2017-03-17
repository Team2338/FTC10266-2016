/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
//
// TeleOp - USES ONLY 1 CONTROLLER GAMEPAD1
// 
// TELEOP, THIS CODE WORKS                           
//                            
//
import com.ftdi.j2xx.D2xxManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.PeerApp;
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Template: Iterative OpMode", group="Iterative OpMode")  // @Autonomous(...) is the other common choice
@Disabled
public class TeleOpOneController extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor intakeLowMotor;
    DcMotor intakeHighMotor;

    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  .5;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    double  positionx = (MAX_POS - MIN_POS) / 2;
    double  positionb = (MAX_POS - MIN_POS) / 2;

    // Define class members
    Servo servo2;  // LINEAR ACTUATOR
    Servo servo3;  // LINEAR ACTUATOR
    Servo servo1;  // CAP BALL SERVO

    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
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

        intakeLowMotor = hardwareMap.dcMotor.get("collectorLow");
        intakeHighMotor = hardwareMap.dcMotor.get("collectorHigh");

        leftShooterMotor = hardwareMap.dcMotor.get("LeftShooter");
        rightShooterMotor = hardwareMap.dcMotor.get("RightShooter");

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo1 = hardwareMap.servo.get("servo1"); //capball
        servo2 = hardwareMap.servo.get("servo2");  //actuator 1
        servo3 = hardwareMap.servo.get("servo3"); //actuator 2

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
//         frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//         frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //
        // RESET ROBOT, CLOSE BOTH ACTUATORS
        //
        // ***** Linear Actuator 1
        telemetry.addData(" LA1 RESET ", positionx);
        positionx -= INCREMENT;
        if (positionx <= MIN_POS) {
            positionx = MIN_POS;
        }
        servo2.setPosition(positionx);

        // ***** Linear Actuator 2
        telemetry.addData(" LA2 RESET ", positionb);
        positionb -= INCREMENT;
        if (positionb <= MIN_POS) {
            positionb = MIN_POS;
        }
        servo3.setPosition(positionb);

        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

/*        if (rampUp) {
            telemetry.addData("In Rampup", 1);
            // Keep stepping up until we hit the max value.
            position += INCREMENT;
            if (position >= MAX_POS) {
                telemetry.addData("In Position", 1);
                position = MAX_POS;
            }
        } else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT;
            if (position <= MIN_POS) {
                telemetry.addData("In Else", 1);
                position = MIN_POS;
            }
        }
*/
        telemetry.addData("Status", "Running: " + runtime.toString());

        telemetry.addData("Front Left Ticks:", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right Ticks", frontRightMotor.getCurrentPosition());

        telemetry.addData("Motor Output", "Front Left" + frontLeftMotor.getPower());
        telemetry.addData("Motor Output", "Front Right" + frontRightMotor.getPower());
        telemetry.addData("Motor Output", "Rear Left" + backLeftMotor.getPower());
        telemetry.addData("Motor Output", "Rear Right" + backRightMotor.getPower());
        telemetry.addData("Cap Ball Servo Pos", "" + servo1.getPosition());
        telemetry.addData("Actuator Right Servo Pos", "" + servo2.getPosition());
        telemetry.addData("Actuator Left Servo Pos", "" + servo3.getPosition());
        telemetry.addData("Actuator Left Servo Pos", "" + servo3.getPosition());
        telemetry.addData("ShooterLeft Power", "" + leftShooterMotor.getPower());
        telemetry.addData("ShooterRight Power", "" + leftShooterMotor.getPower());
        telemetry.addData("ShooterLeft Pos", "" + leftShooterMotor.getCurrentPosition());
        telemetry.addData("ShooterRight Pos", "" + leftShooterMotor.getCurrentPosition());


        // DRIVE TRAIN MOTORS
        frontRightMotor.setPower(-gamepad1.right_stick_y);
        backRightMotor.setPower(-gamepad1.right_stick_y);

        frontLeftMotor.setPower(gamepad1.left_stick_y);
        backLeftMotor.setPower(gamepad1.left_stick_y);

        // REVERSE CENTER INTAKE IN CASE BALL GETS STUCK
        if(gamepad1.dpad_down) {
            intakeHighMotor.setPower(1);
        }

        // CAP BALL
        if (gamepad1.a) {
            servo1.setPosition(1);
        } else {
            servo1.setPosition(0.1);
        }

        // LEFT ACTUATOR
        if (gamepad1.x) {
            // OPEN
            positionx += INCREMENT;
            if (positionx >= MAX_POS) {
                positionx = MAX_POS;

            }
            servo3.setPosition(positionx);
        }
        else {
            // CLOSE
            positionx -= INCREMENT;
            if (positionx <= MIN_POS) {
                positionx = MIN_POS;

            }
            servo3.setPosition(positionx);
        }

        // RIGHT ACTUATOR
        if (gamepad1.b) {
            // OPEN
            positionb += INCREMENT;
            if (positionb >= MAX_POS) {
                positionb = MAX_POS;
                servo2.setPosition(positionb);
            }

        } else {
            // CLOSE
            positionb -= INCREMENT;
            if (positionb <= MIN_POS) {
                positionb = MIN_POS;
            }
            servo2.setPosition(positionb);
        }

/*
        if (gamepad1.left_bumper) {
            intakeLowMotor.setPower(-1);
        } else if (gamepad2.right_bumper) {
            intakeHighMotor.setPower(-1);
        } else {
            intakeLowMotor.setPower(0);
            intakeHighMotor.setPower(0);
        }
*/
        // FRONT INTAKE
        if (gamepad1.left_bumper) {
            intakeLowMotor.setPower(-1);
        } else {
            intakeLowMotor.setPower(0);
        }

        // CENTER INTAKE
        if (gamepad1.right_bumper) {
            intakeHighMotor.setPower(-1);
        } else {
            intakeHighMotor.setPower(0);
        }

        // SHOOTER
//        if (gamepad1.y) {
//            leftShooterMotor.setPower(0.95);  // Positive
//            rightShooterMotor.setPower(-0.95);  // Negative
//        } else {
//            leftShooterMotor.setPower(0);
//            rightShooterMotor.setPower(0);
//        }

        if (gamepad1.y) {
            leftShooterMotor.setPower(0.80);
            rightShooterMotor.setPower(-0.80);

            leftShooterMotor.setMaxSpeed(2800);
            rightShooterMotor.setMaxSpeed(2800);
        } else {
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

//have faith nerds
//okay we have 4 motors total we did it guys
// This will explode the robot
