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

package org.firstinspires.ftc.teamcode.Dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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
//@Config
@Autonomous(name="P_Drive_FTC_Dashboard", group="Linear Opmode")
//@Disabled
public class P_Drive_FTC_Dashboard extends LinearOpMode {

    FtcDashboard dashboard;


    // Declare OpMode members.
    private DcMotor left1 = null;
    private DcMotor left2 = null;
    private DcMotor right1 = null;
    private DcMotor right2 = null;

    public static  double     kp = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7;
    public static  double      Target = 50;


    static final double     COUNTS_PER_MOTOR_REV    = 704.86 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        //telemetry.addData("Status", "Initialized");



        left1  = hardwareMap.get(DcMotor.class, "left_1");
        left2 = hardwareMap.get(DcMotor.class, "left_2");
        right1  = hardwareMap.get(DcMotor.class, "right_1");
        right2 = hardwareMap.get(DcMotor.class, "right_2");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);


        resetEncoders();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        resetEncoders();
        while(1==1) {
            pDrive(Target, 1, dashboardTelemetry);
        }
    }

    public void pDrive(double distance, double speed, Telemetry dashboardTelemetry){

        double velocity;
        double gain;
        int target;

        //resetEncoders();

        target = (int)(distance * COUNTS_PER_INCH);

        // Set Target and Turn On RUN_TO_POSITION
        left1.setTargetPosition(target);
        left2.setTargetPosition(target);
        right1.setTargetPosition(target);
        right2.setTargetPosition(target);

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() && (left1.isBusy() && right1.isBusy()) && left2.isBusy() && right2.isBusy()) {

            gain = calcGain(target);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                gain *= -1.0;

            velocity = speed*gain;
            velocity = Range.clip(Math.abs(velocity), 0.0, 1.0);

            left1.setPower(velocity);
            right1.setPower(velocity);
            left2.setPower(velocity);
            right2.setPower(velocity);

           print(target,dashboardTelemetry);
        }

        // Stop all motion;
        stopMotors();

        // Turn off RUN_TO_POSITION
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()){
            print(target, dashboardTelemetry);
        }
    }
    public double getCurrentPosition(){
        return Math.abs(left1.getCurrentPosition() + left2.getCurrentPosition()
                + right1.getCurrentPosition() + right2.getCurrentPosition())/4;
                // Math.abs returns the absolute value of an argument
    }
    public double getError(double Target){
        return Target - getCurrentPosition();
    }
    public double calcGain(double Target){
        return  Range.clip( getError(Target)*kp, -1,1);
    }
    public void resetEncoders(){
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void stopMotors(){
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
    }
    public void print(double target, Telemetry dashboardTelemetry){
        /*telemetry.addData("Error", getError(target));
        telemetry.addData("Error in distance", getError(target)/COUNTS_PER_INCH);
        telemetry.addData("Distance", getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.update();*/


        dashboardTelemetry.addData("Distance", getCurrentPosition()/COUNTS_PER_INCH);
        dashboardTelemetry.update();

    }

}
