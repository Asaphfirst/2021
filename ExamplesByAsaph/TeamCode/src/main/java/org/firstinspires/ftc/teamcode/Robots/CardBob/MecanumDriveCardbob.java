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

package org.firstinspires.ftc.teamcode.Robots.CardBob;
 /* This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
         * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
         * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
         * class is instantiated on the Robot Controller and executed.
        *
        * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
        * It includes all the skeletal structure that all linear OpModes contain.
        *
        * Use Android Studios to Copy this Class, and Paste it into your teams code folder with a new
        */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list


/**
 */

@TeleOp(name="CardbobMecanumDrive", group="Linear Opmode")
@Disabled
public class MecanumDriveCardbob extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotorEx shootermotor = null;
    private DcMotor armmotor = null;
    private DcMotor intakemotor = null;
    private DcMotor feedermotor = null;
    DistanceSensor distanceSensor;
    TouchSensor touchSensorarm;
    BNO055IMU imu;
    Orientation angles;

    double  max1;
    double  error;
    double  steer;
    double  left1Speed;
    double  right1Speed;
    boolean flag;

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
    static final double     P_DRIVE_COEFF           = 0.06;     // Larger is more responsive, but also less stable
    Orientation currentangle ;

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
        leftDrive1 = hardwareMap.get(DcMotor.class, "left_drive1"); //motor 0
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1"); //motor 1
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2"); //motor 2
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2"); //motor 3
        armmotor = hardwareMap.get(DcMotor.class, "armmotor"); //motor 4
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor"); //motor 5
        shootermotor = hardwareMap.get(DcMotorEx.class, "shootermotor"); //motor 6
        feedermotor = hardwareMap.get(DcMotor.class, "feedermotor"); //motor 6

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        touchSensorarm = hardwareMap.get(TouchSensor.class, "armtouch");


        //telemetry.addData("Speed",   "0.2f",  gyro.);
        telemetry.update();

        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        shootermotor.setDirection(DcMotor.Direction.REVERSE);

        shootermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        boolean intakePower = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            if(gamepad1.left_bumper) {
                resetEncoders();
                flag = true;
                gyroStrafe2(DRIVE_SPEED,32,currentangle.firstAngle);

            } else if(gamepad1.right_bumper) {
                resetEncoders();
                flag = true;
                gyroStrafe3(DRIVE_SPEED,-32,currentangle.firstAngle);
            } else {
                flag = false;
                leftDrive1.setPower(leftPower);
                rightDrive1.setPower(rightPower);
                leftDrive2.setPower(leftPower);
                rightDrive2.setPower(rightPower);


                currentangle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            }

            if (gamepad1.b) {
                if (distanceSensor.getDistance(DistanceUnit.INCH) > 22) ;
                leftDrive1.setPower(1);
                rightDrive1.setPower(-1);
                leftDrive2.setPower(-1);
                rightDrive2.setPower(1);
            } else {
                leftDrive1.setPower(0);
                rightDrive1.setPower(0);
                leftDrive2.setPower(0);
                rightDrive2.setPower(0);
            }


            /** gamepad 2 functions
             **/
            if (gamepad2.y) {
                shootermotor.setVelocity(2000); //1400
                telemetry.addData("Shooter speed", shootermotor.getVelocity());
                telemetry.update();

            } else if (gamepad2.x) {
                shootermotor.setVelocity(1800);
                telemetry.addData("Shooter speed", shootermotor.getVelocity());
                telemetry.update();
            } else {
                shootermotor.setPower(0);
            }


            if (gamepad2.b){
                intakePower = !intakePower;
            }

            if (gamepad2.left_bumper) {
                intakemotor.setPower(1);

            }
            else if (gamepad2.right_bumper) {
                intakemotor.setPower(-1);
            }
            else if (intakePower){
                intakemotor.setPower(0.35);
            }

            else{
                intakemotor.setPower(0);
            }
            if (gamepad2.dpad_left) {
                feedermotor.setPower(0.7);
            } else if (gamepad2.dpad_right) {
                feedermotor.setPower(-0.7);
            } else {
                feedermotor.setPower(0);
            }
            if (gamepad2.dpad_up) {

                armmotor.setPower(0.7);
            } else if (gamepad2.dpad_down && !touchSensorarm.isPressed()) {
                armmotor.setPower(-0.7);
            } else {
                armmotor.setPower(0);
            }

        }
        }


    public void gyroStrafe2(double speed, double distance, double angle) {

        int     newLeft1Target;
        int     newRight1Target;
        int     newLeft2Target;
        int     newRight2Target;
        int     moveCounts;
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

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH_STRAFE);

            newLeft1Target = leftDrive1.getCurrentPosition() - moveCounts;
            newLeft2Target = leftDrive1.getCurrentPosition() + moveCounts;

            newRight1Target = rightDrive1.getCurrentPosition() + moveCounts;
            newRight2Target = rightDrive1.getCurrentPosition() - moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            leftDrive1.setTargetPosition(newLeft1Target);
            leftDrive2.setTargetPosition(newLeft2Target);

            rightDrive1.setTargetPosition(newRight1Target);
            rightDrive2.setTargetPosition(newRight2Target);

            leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDrive1.setPower(speed);
            rightDrive1.setPower(speed);
            leftDrive2.setPower(speed);
            rightDrive2.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (flag && opModeIsActive() &&
                    leftDrive1.isBusy() && rightDrive1.isBusy() && leftDrive2.isBusy() && rightDrive2.isBusy())
            {

                if(!gamepad1.left_bumper)
                flag=false;
                // adjust relative speed based on heading error.
                // if driving in reverse, the motor correction also needs to be reverse

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

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
                max2 =  Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

                if (max1 > 1.0)
                {
                    left1Speed /= max1;

                    right1Speed /= max1;

                }
                if (max2>1.0){
                    left2Speed /= max2;
                    right2Speed /= max2;
                }



                leftDrive1.setPower(left1Speed);
                leftDrive2.setPower(left2Speed);

                rightDrive1.setPower(right1Speed);
                rightDrive2.setPower(right2Speed);


                telemetry.update();

            }

            // Stop all motion;
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void gyroStrafe3(double speed, double distance, double angle) {

        int     newLeft1Target;
        int     newRight1Target;
        int     newLeft2Target;
        int     newRight2Target;
        int     moveCounts;
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

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH_STRAFE);

            newLeft1Target = leftDrive1.getCurrentPosition() - moveCounts;
            newLeft2Target = leftDrive1.getCurrentPosition() + moveCounts;

            newRight1Target = rightDrive1.getCurrentPosition() + moveCounts;
            newRight2Target = rightDrive1.getCurrentPosition() - moveCounts;


            // Set Target and Turn On RUN_TO_POSITION
            leftDrive1.setTargetPosition(newLeft1Target);
            leftDrive2.setTargetPosition(newLeft2Target);

            rightDrive1.setTargetPosition(newRight1Target);
            rightDrive2.setTargetPosition(newRight2Target);

            leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDrive1.setPower(speed);
            rightDrive1.setPower(speed);
            leftDrive2.setPower(speed);
            rightDrive2.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (flag && opModeIsActive() &&
                    leftDrive1.isBusy() && rightDrive1.isBusy() && leftDrive2.isBusy() && rightDrive2.isBusy())
            {

                if(!gamepad1.right_bumper)
                    flag=false;
                // adjust relative speed based on heading error.
                // if driving in reverse, the motor correction also needs to be reverse

                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

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
                max2 =  Math.max(Math.abs(left2Speed), Math.abs(right2Speed));

                if (max1 > 1.0)
                {
                    left1Speed /= max1;

                    right1Speed /= max1;

                }
                if (max2>1.0){
                    left2Speed /= max2;
                    right2Speed /= max2;
                }



                leftDrive1.setPower(left1Speed);
                leftDrive2.setPower(left2Speed);

                rightDrive1.setPower(right1Speed);
                rightDrive2.setPower(right2Speed);


                telemetry.update();

            }

            // Stop all motion;
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }



    public double getError(double targetAngle)
    {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /*
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive1.setPower(leftSpeed);
        rightDrive1.setPower(rightSpeed);
        leftDrive2.setPower(leftSpeed);
        rightDrive2.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /*
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }



    public void resetEncoders(){
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}


