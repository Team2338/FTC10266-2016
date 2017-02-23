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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Sensor Test", group="Iterative OpMode")  // @Autonomous(...) is the other common choice
@Disabled
public class TeleOpSensorTest extends OpMode
{
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.50;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    double  positiona = (MAX_POS - MIN_POS) / 2; //Cap Ball
    double  positionx = (MAX_POS - MIN_POS) / 2; //Linear1
    double  positionb = (MAX_POS - MIN_POS) / 2; //Linear2

    boolean rampUp = true;

    Servo servo1;  //Cap Ball
    Servo servo2;  //Linear1
    Servo servo3;  //Linear2


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
//    DcMotor leftShooterMotor;
//    DcMotor rightShooterMotor;

    ColorSensor colorSensor1;
//    OpticalDistanceSensor odsSensor;
    //float hsvValues[] = {0F,0F,0F};
    boolean bLedOn = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        colorSensor1 = hardwareMap.colorSensor.get("colorSensor1");
        colorSensor1.enableLed(bLedOn);
//        odsSensor = hardwareMap.opticalDistanceSensor.get("distanceSensor");
//        leftShooterMotor = hardwareMap.dcMotor.get("LeftShooter");
//        rightShooterMotor = hardwareMap.dcMotor.get("RightShooter");

        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery

        //frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
       // frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftDrive");
        backRightMotor = hardwareMap.dcMotor.get("backRightDrive");

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

        // ***** Linear Actuator 1
        telemetry.addData(" LA1 Start ", positionx);
        positionx -= INCREMENT ;
        telemetry.addData(" MOVE IN  ", positionx);
        if (positionx <= MIN_POS ) {
            positionx = MIN_POS;
        }
        servo2.setPosition(positionx);


        // ***** Linear Actuator 1
        telemetry.addData(" LA1 Start ", positionx);
        positionb -= INCREMENT ;
        telemetry.addData(" MOVE IN  ", positionb);
        if (positionb <= MIN_POS ) {
            positionb = MIN_POS;
        }
        servo3.setPosition(positionb);

        telemetry.addData(" INIT LOOP X  ", positionx);
        telemetry.addData(" INIT LOOP Y  ", positionb);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    //    colorSensor1.enableLed(true);

        telemetry.addData(" IN LOOP X ", positionx);
        telemetry.addData(" IN LOOP Y ", positionb);

      if (gamepad2.x) {
            telemetry.addData(" IN X OUT ", positionx);

            // Keep stepping up until we hit the max value.
            positionx += INCREMENT ;
            if (positionx >= MAX_POS ) {
                positionx= MAX_POS;
            }
            servo2.setPosition(positionx);

      } else {
            telemetry.addData(" IN X IN ", positionx);
            // Keep stepping down until we hit the min value.
            positionx -= INCREMENT ;
            if (positionx <= MIN_POS ) {
                positionx = MIN_POS;
            }
            servo2.setPosition(positionx);

      }

      if (gamepad2.b) {
          telemetry.addData(" IN Y OUT ", positionb);
          // Keep stepping up until we hit the max value.
            positionb += INCREMENT ;
            if (positionb >= MAX_POS ) {
                positionb = MAX_POS;
            }
      } else {
          telemetry.addData(" IN Y IN ", positionb);
            // Keep stepping down until we hit the min value.
            positionb -= INCREMENT ;
            if (positionb <= MIN_POS ) {
                positionb = MIN_POS;
            }
      }

      servo2.setPosition(positionx);
      servo3.setPosition(positionb);

    /*
        if(colorSensor.blue() >2) {
            rightShooterMotor.setPower(1);
            leftShooterMotor.setPower(1);
        }

        if(colorSensor.red() > 2) {

            rightShooterMotor.setPower(1); //replace the shooter motors with linear actuators.
            leftShooterMotor.setPower(1);
        }
        if(colorSensor.red() <= 0 && colorSensor.blue() <=0) {

            rightShooterMotor.setPower(0.0);
            leftShooterMotor.setPower(0.0);

        }
        */

//        if (rampUp) {
//            // Keep stepping up until we hit the max value.
//            position += INCREMENT ;
//            if (position >= MAX_POS ) {
//                position = MAX_POS;
//                rampUp = !rampUp;   // Switch ramp direction
//            }
//        }
//        else {
//            // Keep stepping down until we hit the min value.
//            position -= INCREMENT ;
//            if (position <= MIN_POS ) {
//                position = MIN_POS;
//                rampUp = !rampUp;  // Switch ramp direction



        /*telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Raw",    odsSensor.getRawLightDetected());
        telemetry.addData("Normal", odsSensor.getLightDetected());
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData(">", "Press Stop to end test." );
        servo.setPosition(position);
        */

      /*  telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor1.alpha());
        telemetry.addData("Red  ", colorSensor1.red());
        telemetry.addData("Green", colorSensor1.green());
        telemetry.addData("Blue ", colorSensor1.blue());
        telemetry.addData("Hue", hsvValues[0]);
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