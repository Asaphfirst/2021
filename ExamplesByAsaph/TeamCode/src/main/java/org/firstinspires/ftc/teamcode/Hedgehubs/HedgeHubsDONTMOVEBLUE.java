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

package org.firstinspires.ftc.teamcode.Hedgehubs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
@Autonomous(name="HedgeHubs Does not move BLUE", group="Linear Opmode")
//Disabled
public class HedgeHubsDONTMOVEBLUE extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    // Declare OpMode members.
    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;
    private DcMotorEx shootermotor = null;
    private DcMotor armmotor = null;
    private DcMotor feedermotor = null;
    private Servo claw = null;

    private ElapsedTime runtime = new ElapsedTime();
    public static PIDFCoefficients DrivetrainPID = new PIDFCoefficients(25, 0.05, 1.25, 0);
    public static double P_DRIVE_COEFF = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7; //0.04
    public static double P_DRIVE_COEFF2 = 0.0002;

    public static double Target = 50;

    static final double COUNTS_PER_MOTOR_REV = 746.6;    //
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415)) / 1.65;
    static final double COUNTS_PER_INCH_STRAFE = ((COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415)) * 0.7874015748;

    @Override
    public void runOpMode() {
        // dashboard = FtcDashboard.getInstance();
        // Telemetry dashboardTelemetry = dashboard.getTelemetry();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        left1 = hardwareMap.get(DcMotorEx.class, "left1");
        left2 = hardwareMap.get(DcMotorEx.class, "left2");
        right1 = hardwareMap.get(DcMotorEx.class, "right1");
        right2 = hardwareMap.get(DcMotorEx.class, "right2");

        shootermotor = hardwareMap.get(DcMotorEx.class, "shooter");
        feedermotor = hardwareMap.get(DcMotor.class, "feeder");
        claw = hardwareMap.get(Servo.class, "claw");

        shootermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        left2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, DrivetrainPID);

        resetEncoders();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        /*
        resetEncoders();
        gyroDrive(0.7, -70, 0);
        resetEncoders();
        gyroStrafe(0.4, -14, 0); //-18
        shootAuto();
        resetEncoders();
        gyroDrive(0.7, -10, 0);
*/
        shootAuto();
        while (opModeIsActive()){

        }


    }

    public void gyroStrafe(double speed, double distance, double angle) {

        int newLeft1Target;
        int newRight1Target;
        int newLeft2Target;
        int newRight2Target;
        int moveCounts;
        double max1;
        double max2;
        double error;
        double steer;
        double left1Speed;
        double left2Speed;
        double right1Speed;
        double right2Speed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH_STRAFE);

            newLeft1Target = left1.getCurrentPosition() - moveCounts;
            newLeft2Target = left1.getCurrentPosition() + moveCounts;

            newRight1Target = right1.getCurrentPosition() + moveCounts;
            newRight2Target = right1.getCurrentPosition() - moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            left1.setTargetPosition(newLeft1Target);
            left2.setTargetPosition(newLeft2Target);

            right1.setTargetPosition(newRight1Target);
            right2.setTargetPosition(newRight2Target);

            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            left1.setPower(speed);
            right1.setPower(speed);
            left2.setPower(speed);
            right2.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (left1.isBusy() && right1.isBusy()) && left2.isBusy() && right2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF2);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                //leftSpeed = speed - steer;
                //rightSpeed = speed + steer;

                left1Speed = speed + steer;
                left2Speed = speed - steer;

                right1Speed = speed + steer;
                right2Speed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max1 = Math.max(Math.abs(left1Speed), Math.abs(right1Speed));
                max2 = Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

                if (max1 > 1.0) {
                    left1Speed /= max1;

                    right1Speed /= max1;

                }
                if (max2 > 1.0) {
                    left2Speed /= max2;
                    right2Speed /= max2;
                }


                left1.setPower(left1Speed);
                left2.setPower(left2Speed);

                right1.setPower(right1Speed);
                right2.setPower(right2Speed);


                telemetry.update();

            }

            // Stop all motion;
            left1.setPower(0);
            right1.setPower(0);
            left2.setPower(0);
            right2.setPower(0);

            // Turn off RUN_TO_POSITION
            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getError(double targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public void gyroDrive(double speed, double distance, double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = left1.getCurrentPosition() + moveCounts;
            newRightTarget = right1.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            left1.setTargetPosition(newLeftTarget);
            left2.setTargetPosition(newLeftTarget);
            right1.setTargetPosition(newRightTarget);
            right2.setTargetPosition(newRightTarget);

            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            speed = Range.clip(Math.abs(speed), 0.0, 1.0);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    left1.isBusy() && right1.isBusy() && left2.isBusy() && right2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getErrorAngle(angle);
                steer = getSteer(error, P_DRIVE_COEFF);


                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left1.setPower(leftSpeed);
                left2.setPower(leftSpeed);

                right1.setPower(rightSpeed);
                right2.setPower(rightSpeed);


            }

            // Stop all motion;
            stopMotors();

            // Turn off RUN_TO_POSITION
            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getCurrentPosition() {
        return Math.abs(left1.getCurrentPosition() + left2.getCurrentPosition()
                + right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
        // Math.abs returns the absolute value of an argument
    }

    public double getErrorAngle(double targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void resetEncoders() {
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopMotors() {
        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
    }

    public void shootAuto() {
        runtime.reset();
        while (opModeIsActive() &&runtime.seconds() < 5) {
            shootermotor.setVelocity(-1800);
        }

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shootermotor.setVelocity(-1800);
            feedermotor.setPower(0.7);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            shootermotor.setVelocity(-1800);
            feedermotor.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shootermotor.setVelocity(-1800);
            feedermotor.setPower(0.7);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            shootermotor.setVelocity(-1800);
            feedermotor.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shootermotor.setVelocity(-1800);
            feedermotor.setPower(0.7);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            feedermotor.setPower(0);

        }


    }
}

/*
runtime.reset();
        while(runtime.seconds() < 3 ){
            shootermotor.setVelocity(2000);
        }

        runtime.reset();
        while(runtime.seconds() < 1.5){
            shootermotor.setVelocity(2000);
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            shootermotor.setVelocity(2000);
            feedermotor.setPower(0);
        }
        runtime.reset();
        while(runtime.seconds() < 1.5){
            shootermotor.setVelocity(2000);
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            shootermotor.setVelocity(2000);
            feedermotor.setPower(0);
        }  runtime.reset();
        while(runtime.seconds() < 1.5){
            shootermotor.setVelocity(2000);
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
*/
