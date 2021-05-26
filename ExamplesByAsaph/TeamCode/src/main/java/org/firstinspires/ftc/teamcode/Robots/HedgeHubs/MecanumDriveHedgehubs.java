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

package org.firstinspires.ftc.teamcode.Robots.HedgeHubs;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name="HedgeHubs MecanumDrive", group="Linear Opmode")
@Disabled
public class MecanumDriveHedgehubs extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive1 = null;
    private DcMotorEx rightDrive1 = null;
    private DcMotorEx leftDrive2 = null;
    private DcMotorEx rightDrive2 = null;
    private DcMotorEx shootermotor = null;
    private DcMotor armmotor = null;
    private DcMotor feedermotor = null;

    private Servo claw = null;
    double MaxVel=2500;
    BNO055IMU imu;
    Orientation angles;
    boolean flag;
    boolean flagButton = false;

    /* Declare OpMode members. */
    static final double     COUNTS_PER_MOTOR_REV    = 746.6 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/1.65;

    static final double COUNTS_PER_INCH_STRAFE = ((COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415))*0.7874015748;
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;


    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.025;     // Larger is more responsive, but also less stable
    public static  double     P_DRIVE_COEFF           = 600;     // Larger is more responsive, but also less stable was 0.06 before
    Orientation currentangle ;
    boolean flagRachel = false;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1 = hardwareMap.get(DcMotorEx.class, "left1");
        rightDrive1 = hardwareMap.get(DcMotorEx.class, "right1");
        leftDrive2 = hardwareMap.get(DcMotorEx.class, "left2");
        rightDrive2 = hardwareMap.get(DcMotorEx.class, "right2");

        armmotor = hardwareMap.get(DcMotor.class, "arm");
        shootermotor = hardwareMap.get(DcMotorEx.class, "shooter");
        feedermotor = hardwareMap.get(DcMotor.class, "feeder");
        claw = hardwareMap.get(Servo.class, "claw");

        shootermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn = 0.8*gamepad1.right_stick_x; // Changed to limit turning speed
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            if(gamepad1.left_bumper) {
                gyroStrafeLeft(currentangle.firstAngle);
               flag = true;
            }
                else if(gamepad1.right_bumper) {
                gyroStrafeRight(currentangle.firstAngle);
                flag = true;
            }
                else {
                leftDrive1.setPower(leftPower);
                rightDrive1.setPower(rightPower);
                leftDrive2.setPower(leftPower);
                rightDrive2.setPower(rightPower);
                flag = false;

                currentangle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            }





            /** gamepad 2 functions
             **/

           GamePad2();
           Print();



        }
        }

    public void stopRobot(){
        leftDrive1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive1.setPower(0);
        rightDrive2.setPower(0);
    }
    public void Print(){
        telemetry.addData("Angle", currentangle.firstAngle);
        telemetry.addData("Drive flag", flagButton);
        //telemetry.addData("Servo position", claw.getPosition());
        //telemetry.addData("left1 Velocity", leftDrive1.getVelocity( ));
        //telemetry.addData("left2 Velocity", leftDrive2.getVelocity( ));
        //telemetry.addData("right1 Velocity", rightDrive1.getVelocity( ));
        //telemetry.addData("right2 Velocity", rightDrive2.getVelocity( ));
        telemetry.update();
    }
    public void GamePad2(){
        // Shooter
        if (gamepad2.y) {
            shootermotor.setVelocity(-1600); //-1700
            flagRachel = true;
        }
        else if(gamepad2.a){
            shootermotor.setVelocity(-1500);
            flagRachel = true;
        }
        else {
            flagRachel = false;
            shootermotor.setPower(0);
        }

        // Feeder
            if (gamepad2.dpad_left ) {
                if(flagRachel) {
                    feedermotor.setPower(0.7);
                }
                else{
                    feedermotor.setPower(1);
                }
            } else if (gamepad2.dpad_right) {
                feedermotor.setPower(-0.7);
                shootermotor.setVelocity(1000);
            } else {
                feedermotor.setPower(0);
            }
        // Arm
        if (gamepad2.right_bumper) {
            armmotor.setPower(1);
        }
        else if (gamepad2.left_bumper) {
            armmotor.setPower(-1);
        }
        else {
            armmotor.setPower(0);
        }

        // Claw
        if (gamepad2.x) {
            claw.setPosition(0);
        }
        else {
            claw.setPosition(0.8);
        }

    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void gyroStrafeLeft(double angle) {

        double  max1;
        double max2;
        double  error;
        double  steer;
        double  left1Speed;
        double  left2Speed;
        double  right1Speed;
        double  right2Speed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && flag) {

                if(!gamepad1.left_bumper)
                    flag=false;

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                left1Speed = -2400 - steer;
                left2Speed = 2400 + steer;
                right1Speed = 2400 - steer;
                right2Speed = -2400 + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max1 = Math.max(Math.abs(left1Speed), Math.abs(right1Speed));
                max2 = Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

                if (max1 > 2500) {
                    left1Speed /= max1;

                    right1Speed /= max1;

                }
                if (max2 > 2500) {
                    left2Speed /= max2;
                    right2Speed /= max2;
                }

                leftDrive1.setVelocity(left1Speed);
                leftDrive2.setVelocity(right1Speed);
                rightDrive1.setVelocity(left2Speed);
                rightDrive2.setVelocity(right2Speed);

            }
            stopRobot(); // Trying to fix the strafe problem
        }

        }
    public void gyroStrafeRight(double angle) {

        double  max1;
        double max2;
        double  error;
        double  steer;
        double  left1Speed;
        double  left2Speed;
        double  right1Speed;
        double  right2Speed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && flag) {

                if(!gamepad1.left_bumper)
                    flag=false;

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                left1Speed = 2400 + steer;
                left2Speed = -2400 - steer;
                right1Speed = -2400 + steer;
                right2Speed = 2400 - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max1 = Math.max(Math.abs(left1Speed), Math.abs(right1Speed));
                max2 = Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

                if (max1 > 2500) {
                    left1Speed /= max1;

                    right1Speed /= max1;

                }
                if (max2 > 2500) {
                    left2Speed /= max2;
                    right2Speed /= max2;
                }

                leftDrive1.setVelocity(left1Speed);
                leftDrive2.setVelocity(right1Speed);
                rightDrive1.setVelocity(left2Speed);
                rightDrive2.setVelocity(right2Speed);

            }

        }
        stopRobot();
    }
    public double getError(double targetAngle)
    {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

}


