/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Robots;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This Opmode is an example of a Proportional Controller
 * The drivetrain will move to a target distance using encoders and slow down when it approaches using proportional
 *
 * ---------------------------- PROPORTIONAL ------------------------------------
 * Error = Target - Current Position
 * Target is the distance that the robot will move to
 * Current position is an average of all the encoders
 * Kp is a constant that needs to be tuned
 * Gain = Error*Kp
 * velocity = speed*gain
 * velocity will store the result
 * speed is a constant chosen by the user
 * gain will decrease as the drivetrain approaches the target
 *
 * Proportional controller is a good solution for controlling a drivetrain autonomously,
 * but the robot will never reach the target, there will always be a steady state error.
 * The Steady State Error will not be too big, but it can make the code get stuck in the PDrive loop.
 * The solution for that is to exit the loop once the error is very small.
 *
 * ---------------------------- FTC DASHBOARD ------------------------------------
 * https://acmerobotics.github.io/ftc-dashboard/
 * Kp and target distance can be changed using the dashboard
 * Prints a graph of the position
 * To open the dashboard connect your laptop to the robot's wifi and then access this address using a browser:
 * http://192.168.43.1:8080/dash
 */

@Autonomous(name="tieriv", group="Linear Opmode")
//@Disabled
public class Kelly extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive = null;
    private DcMotor rightDrive2 = null;

    private DcMotor armmotor = null;
    private DcMotor intakemotor = null;
    private DcMotor feedermotor = null;
    private DcMotorEx shootermotor = null;

    DistanceSensor distanceSensor;
    DistanceSensor distanceSensor2;

    public static  double     P_DRIVE_COEFF = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7; //0.04
    //larger is more responsive but less stable

    static final double     COUNTS_PER_MOTOR_REV    = 746.6 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)/1.65;
    static final double     COUNTS_PER_INCH_STRAFE = ((COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415))*0.7874015748;

    @Override
    public void runOpMode() {

        // get a reference to our ColorSensor object.

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "sensor_range2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) distanceSensor2;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive  = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");

        feedermotor  = hardwareMap.get(DcMotor.class, "feedermotor");
        armmotor = hardwareMap.get(DcMotor.class, "armmotor");
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        shootermotor = hardwareMap.get(DcMotorEx.class, "shootermotor");


        // Most robots need the motor on one side to be reversed to drive forward
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        shootermotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        resetEncoders();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("first sensor", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Second sensor", distanceSensor2.getDistance(DistanceUnit.INCH));
            telemetry.update();
/*
            if(distanceSensor.getDistance(DistanceUnit.INCH) < 10){
                telemetry.addData("first sensor", "true");
                telemetry.addData("first sensor", distanceSensor.getDistance(DistanceUnit.INCH));
            } else {
                telemetry.addData("first sensor", "false");
                telemetry.addData("first sensor", distanceSensor.getDistance(DistanceUnit.INCH));
            }
            if (distanceSensor2.getDistance(DistanceUnit.INCH) < 10){
                telemetry.addData("second sensor", "true");
                telemetry.addData("second sensor", distanceSensor2.getDistance(DistanceUnit.INCH));
            } else {
                telemetry.addData("second sensor", "false");
                telemetry.addData("second sensor", distanceSensor2.getDistance(DistanceUnit.INCH));
            }
            telemetry.update();
            */

        }

        gyroDrive(0.3,45,0);

        if (distanceSensor.getDistance(DistanceUnit.INCH) < 4){ //detects rings on the bottom
            if (distanceSensor2.getDistance(DistanceUnit.INCH) < 5){ //detects ring on top, so whether or not 4 rings
                autonomousC();
            } else if (distanceSensor2.getDistance(DistanceUnit.INCH) > 5){
                gyroStrafe(0.3,10,0);
                stopMotors();
            }
        } else {
            gyroDrive(0.3,-30,0);
            stopMotors();
        }


    }
    public void autonomousC(){

        resetEncoders();
        gyroDrive(1, 68, 0);
        resetEncoders();
        // the distance is -ve because the target = current position +/- distance
        gyroStrafe(1, -35, 0);

        //sets down the wobble goal
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            armmotor.setPower(-0.8);
        }
        armmotor.setPower(0);

        resetEncoders();
        gyroStrafe(1, 22, 0);
        resetEncoders();
        gyroDrive(1, -46, 0);

        while(runtime.seconds()<3){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(0);
        }
        while(runtime.seconds() < 1.5){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(0);
        }
        runtime.reset();
        while(runtime.seconds() < 1.5){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(0);
        }  runtime.reset();
        while(runtime.seconds() < 1.5){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            shootermotor.setVelocity(-2000);
            feedermotor.setPower(0);
        }
        //lining up to intake 3 more rings
        resetEncoders();
        gyroDrive(1, -10, 0);


    }
    public void gyroStrafe(double speed, double distance, double angle) {
        int newLeftTarget1;
        int newRightTarget1;
        int newLeftTarget2;
        int newRightTarget2;
        int     moveCounts;
        double max1;
        double max2;
        double  error;
        double  steer;
        double  leftSpeed1;
        double  rightSpeed1;
        double  leftSpeed2;
        double  rightSpeed2;

        if (opModeIsActive()) {

            moveCounts = (int)(distance * COUNTS_PER_INCH_STRAFE);
            newLeftTarget1 = leftDrive.getCurrentPosition() - moveCounts;
            newLeftTarget2 = leftDrive2.getCurrentPosition() + moveCounts;
            newRightTarget1 = rightDrive.getCurrentPosition() + moveCounts;
            newRightTarget2 = rightDrive.getCurrentPosition() - moveCounts;

            leftDrive.setTargetPosition(newLeftTarget1);
            leftDrive2.setTargetPosition(newLeftTarget2);
            rightDrive.setTargetPosition(newRightTarget1);
            rightDrive2.setTargetPosition(newRightTarget2);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            while (opModeIsActive() &&
                    leftDrive.isBusy() && rightDrive.isBusy() && leftDrive2.isBusy() && rightDrive2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getErrorAngle(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed1 = (speed + steer);
                rightSpeed1 = speed - steer;
                leftSpeed2 = speed - steer;
                rightSpeed2 = speed + steer;

                max1 = Math.max(Math.abs(leftSpeed1), Math.abs(rightSpeed1));
                max2 = Math.max(Math.abs(leftSpeed2), Math.abs(rightSpeed2));

                if (max1 > 1.0)
                {
                    leftSpeed1 /= max1;
                    rightSpeed1 /= max1;
                    //divides the speeds by themselves so that the max = 1 always
                }

                if (max2 > 1.0)
                {
                    leftSpeed1 /= max2;
                    rightSpeed1 /= max2;
                    //divides the speeds by themselves so that the max = 1 always
                }


                leftDrive.setPower(leftSpeed1);
                leftDrive2.setPower(leftSpeed2);

                rightDrive.setPower(rightSpeed1);
                rightDrive2.setPower(rightSpeed2);
            }

        }
    }
    public void gyroDrive(double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double gain;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive.setTargetPosition(newLeftTarget);
            leftDrive2.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            rightDrive2.setTargetPosition(newRightTarget);

            //runs to target position
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            speed = Range.clip(Math.abs(speed), 0.0, 1.0);


            // making sure all motors are running
            while (opModeIsActive() &&
                    leftDrive.isBusy() && rightDrive.isBusy() && leftDrive2.isBusy() && rightDrive2.isBusy()) {

                //gain calculates the proportional
                gain = calcGain(moveCounts);

                // adjust relative speed based on heading error.
                error = getErrorAngle(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (distance < 0)
                    steer *= -1.0;

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    gain *= -1.0;
                //gain = -1 * gain

                //multiplies the speed (adjusted to drive straight) with the gain, which allows for proportional in driving
                leftSpeed = (speed - steer)*gain;
                rightSpeed = (speed + steer)*gain;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    //divides the speeds by themselves so that the max = 1 always
                }

                telemetry.update();

                leftDrive.setPower(leftSpeed);
                leftDrive2.setPower(leftSpeed);

                rightDrive.setPower(rightSpeed);
                rightDrive2.setPower(rightSpeed);
            }

            // Stop all motion;
            //stopMotors();

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public double getCurrentPosition(){
        return Math.abs(leftDrive.getCurrentPosition() + leftDrive2.getCurrentPosition()
                + rightDrive.getCurrentPosition() + rightDrive2.getCurrentPosition())/4;
        // Math.abs returns the absolute value of an argument
    }
    public double getErrorAngle(double targetAngle) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public double getError(double Target){
        return Target - getCurrentPosition();
    }
    public double calcGain(double Target) {
        //getError(target)*kp = the proportional, constantly changes due to changing error
        return Range.clip(getError(Target) * P_DRIVE_COEFF, -1, 1);
    }
    public void resetEncoders(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void stopMotors(){
        telemetry.addData("stop", "motors" );
        leftDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive.setPower(0);
        rightDrive2.setPower(0);
    }
}