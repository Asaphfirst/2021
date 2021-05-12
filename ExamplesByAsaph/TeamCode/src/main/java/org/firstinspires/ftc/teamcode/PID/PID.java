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

package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * ---------------------------- FTC DASHBOARD ------------------------------------
 * https://acmerobotics.github.io/ftc-dashboard/
 * Kp and target distance can be changed using the dashboard
 * Prints a graph of the position
 * To open the dashboard connect your laptop to the robot's wifi and then access this address using a browser:
 * http://192.168.43.1:8080/dash
 *
 *
 *
 *
 * https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
 *
 */

@Config
@TeleOp(name="PID", group="Linear Opmode")
@Disabled

public class PID extends LinearOpMode {

    FtcDashboard dashboard;

    // Declare OpMode members.
    private DcMotor left1 = null;
    private DcMotor left2 = null;
    private DcMotor right1 = null;
    private DcMotor right2 = null;

    double integral=0;
     //Kp = 0.00202674492; // kp=1/(704.86*constant) = constant = 0.7;
    public static PIDCoefficients shooterPID = new PIDCoefficients(0.0047332348820,0,0);
    public static  double      Target = 50;
    public double greatest_dist = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 704.86 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    ElapsedTime PIDTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        left1  = hardwareMap.get(DcMotor.class, "left_1");
        left2 = hardwareMap.get(DcMotor.class, "left_2");
        right1  = hardwareMap.get(DcMotor.class, "right_1");
        right2 = hardwareMap.get(DcMotor.class, "right_2");

        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        /*left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        waitForStart();

        PID_Drive(Target,0, dashboardTelemetry);
        }

        void PID_Drive(double distance, double allowable_error_IN, Telemetry dashboardTelemetry){

        resetEncoders();

        allowable_error_IN *= COUNTS_PER_INCH;

        double error = 0;
        double lastError = 0;
        double velocity;
        int target = (int) (distance * COUNTS_PER_INCH);

            error = target - getCurrentPosition();
            double t1=0;
            PIDTimer.reset();

        while(opModeIsActive()){

            error = target - getCurrentPosition();

            double changInError = lastError - error;
            double t2 =PIDTimer.time();
            integral += changInError*PIDTimer.time();
            double derivative = changInError;///(t2 - t1);

            double P = shooterPID.p*error;
            double I = shooterPID.i*integral;
            double D = shooterPID.d*derivative;

            velocity = P+I+D;

            velocity = Range.clip((velocity), -1.0, 1.0);

            left1.setPower(velocity);
            left2.setPower(velocity);
            right1.setPower(velocity);
            right2.setPower(velocity);

            print(target,dashboardTelemetry);

            lastError =error;
            t1=t2;
        }
            stopMotors();

        }
        void resetEncoders(){
            left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    public double getCurrentPosition(){
        return (left1.getCurrentPosition() + left2.getCurrentPosition()
                + right1.getCurrentPosition() + right2.getCurrentPosition())/4;
        // Math.abs returns the absolute value of an argument
    }
    public void print(double target,Telemetry dashboardTelemetry){

        double dist = getCurrentPosition()/COUNTS_PER_INCH;

        if(dist>greatest_dist)
            greatest_dist = dist;

        
        dashboardTelemetry.addData("Distance", dist);
        dashboardTelemetry.addData("Peak", greatest_dist);
        dashboardTelemetry.addData("Error", target- getCurrentPosition());
        dashboardTelemetry.update();
    }
    public void stopMotors(){
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
    }
    }

