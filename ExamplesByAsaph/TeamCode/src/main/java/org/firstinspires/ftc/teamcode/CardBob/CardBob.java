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

package org.firstinspires.ftc.teamcode.CardBob;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RED OFFICIAL", group="Pushbot")
@Disabled

public class CardBob extends LinearOpMode
{

    ColorSensor Rightbox;
    DistanceSensor Rightbox2;

    ColorSensor sensorColor1;
    DistanceSensor sensorDistance1;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance2;

    DistanceSensor distanceSensor;

    TouchSensor touchSensorarm;
    // DistanceSensor distanceSensor1;


    BNO055IMU imu;

    Orientation angles;

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 746.6 ;    //
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415))/1.65;
    static final double     COUNTS_PER_INCH_STRAFE = ((COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415))*0.7874015748;

    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor shootermotor = null;
    private DcMotor armmotor = null;
    private DcMotor intakemotor = null;
    private DcMotor feedermotor = null;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.025;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.04;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {

        Rightbox = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        Rightbox2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        sensorColor1 = hardwareMap.get(ColorSensor.class, "sensor_color_distance1");
        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance1");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance2");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance2");
        touchSensorarm = hardwareMap.get(TouchSensor.class, "armtouch");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");


        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;


        float hsvValues[] = {0F, 0F, 0F}; // hsvValues is an array that will hold the hue, saturation, and value information.
        final float values[] = hsvValues; // values is a reference to the hsvValues array.
        final double SCALE_FACTOR = 255;
        float hsvValues1[] = {0F, 0F, 0F};
        final float values1[] = hsvValues1;  // values is a reference to the hsvValues array.

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive1"); //motor 0
        rightDrive1  = hardwareMap.get(DcMotor.class, "right_drive1"); //motor 0
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive2"); //motor 0
        rightDrive2  = hardwareMap.get(DcMotor.class, "right_drive2"); //motor 0
        feedermotor  = hardwareMap.get(DcMotor.class, "feedermotor"); //motor 0
        armmotor = hardwareMap.get(DcMotor.class, "armmotor"); //motor 4
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor"); //motor 5
        shootermotor = hardwareMap.get(DcMotor.class, "shootermotor"); //motor 6

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);


        // Ensure the robot it stationary, then reset the encoders
        resetEncoders();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        intakemotor.setPower(0.35);


        Color.RGBToHSV((int) (Rightbox.red() * SCALE_FACTOR),
                (int) (Rightbox.green() * SCALE_FACTOR),
                (int) (Rightbox.blue() * SCALE_FACTOR),
                hsvValues);

        Color.RGBToHSV((int) (sensorColor1.red() * SCALE_FACTOR),
                (int) (Rightbox.green() * SCALE_FACTOR),
                (int) (Rightbox.blue() * SCALE_FACTOR),
                hsvValues1);

        Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
                (int) (Rightbox.green() * SCALE_FACTOR),
                (int) (Rightbox.blue() * SCALE_FACTOR),
                hsvValues1);

        //left is positive, right is negative
        // gyroDrive(DRIVE_SPEED, 9.1,0.0); //Drive FWD 9.1 inches
        // gyroStrafe(DRIVE_SPEED,50,0);

        //while (!touchSensorarm.isPressed()){
        //  armmotor.setPower(-1); //move arm down until touch sensor is pressed
        //}


        gyroDrive(0.2, 38,0);
        gyroHold(0,0,2); //pause for 2 seconds


        // gyroDrive(0.35, 39, 0); //drive fwd 11 inches, (to the rings)
        //  gyroHold(0,0,2); //pause for 2 seconds
        // gyroDrive(0.6,10,0);
/*
            runtime.reset();
            while( runtime.seconds() < 3000 && opModeIsActive() ){
                leftDrive1.setPower(0);
                leftDrive2.setPower(0);
                rightDrive1.setPower(0);
                rightDrive2.setPower(0);

                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      leftDrive1.getCurrentPosition(), leftDrive2.getCurrentPosition(),
                        rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
                telemetry.update();
            }*/
        Color.RGBToHSV((int) (Rightbox.red() * SCALE_FACTOR),
                (int) (Rightbox.green() * SCALE_FACTOR),
                (int) (Rightbox.blue() * SCALE_FACTOR),
                hsvValues);

        if (hsvValues[0] > 45 && hsvValues[0] < 95) //DETECTING THE RINGS
        {

            if (hsvValues[1] > 0.5 && hsvValues[1] < 0.95) {

                if (hsvValues[2] > 550 && hsvValues[2] < 16550) {

                    if (distanceSensor.getDistance(DistanceUnit.INCH) > 5) {
                        //AUTONOMOUS A PLEASE JUST Put IT IN
                        telemetry.addLine("Autonomous B, 1 ring");
                        AutonomousB();
                    } else if (distanceSensor.getDistance(DistanceUnit.INCH) < 5) {
                        //AUTONOMOUS C PLEASE JUST PUT IT IN
                        telemetry.addLine("Autonomous C, 4 rings");
                        AutonomousC();
                    }
                }
            }
        } else {
            //AUTONOMOUS B PLEASE JUST PUT IT IN I SWEAR TO GOT LIKE JUST INSERT IT HERE BECAUSE ITS PROLly GONNA BE A
            telemetry.addLine("Autonomous A, no rings");
            AutonomousA();
        }

        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", Rightbox2.getDistance(DistanceUnit.CM)));
        // telemetry.addData("Alpha", Rightbox.alpha());
        //  telemetry.addData("Red  ", sensorColor.red());
        // telemetry.addData("Green", sensorColor.green());
        //telemetry.addData("Blue ", sensorColor.blue());
        // telemetry.addData("Hue", hsvValues[0]);
        //telemetry.addData("Saturation", hsvValues[1]);
        //telemetry.addData("Value", hsvValues[2]);


        telemetry.addData("Path", "Complete");
        telemetry.update();



    }

    public void resetEncoders(){
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftDrive1.getCurrentPosition() + moveCounts;
            newRightTarget = rightDrive1.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive1.setTargetPosition(newLeftTarget);
            rightDrive1.setTargetPosition(newRightTarget);
            leftDrive2.setTargetPosition(newLeftTarget);
            rightDrive2.setTargetPosition(newRightTarget);

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

            // Display drive status for the driver.
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      leftDrive1.getCurrentPosition(), leftDrive2.getCurrentPosition(),
                    rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
            telemetry.addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive1.isBusy() && rightDrive1.isBusy()) && leftDrive2.isBusy() && rightDrive2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);


                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftDrive1.setPower(leftSpeed);
                rightDrive1.setPower(rightSpeed);
                leftDrive2.setPower(leftSpeed);
                rightDrive2.setPower(rightSpeed);


                telemetry.update();

            }

            // Stop all motion;
            leftDrive1.setPower(0);
            rightDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroStrafe(double speed, double distance, double angle) {

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

            // Display drive status for the driver.
            telemetry.addData("Target",  "%7d:%7d",      newLeft1Target,  newRight1Target);
            telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      leftDrive1.getCurrentPosition(), leftDrive2.getCurrentPosition(),
                    rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
            telemetry.addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive1.isBusy() && rightDrive1.isBusy()) && leftDrive2.isBusy() && rightDrive2.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

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
            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive1.setPower(0);
        rightDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);
    }


    /*
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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
    boolean onHeading(double speed, double angle, double PCoeff)
    {
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
    public void printEncoders(){
        telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      leftDrive1.getCurrentPosition(), leftDrive2.getCurrentPosition(),
                rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());

        telemetry.update();
    }

    public void AutonomousA() {
        resetEncoders();
        gyroDrive(0.3, 20, 0);
        resetEncoders();
        gyroStrafe(0.3, -32, 0);

        runtime.reset();
        while(runtime.seconds()<0.5){
            armmotor.setPower(-0.8);
        }
        armmotor.setPower(0);
        runtime.reset();
        while(runtime.seconds() < 1.5) {
            intakemotor.setPower(-0.35);
        }
        intakemotor.setPower(0);
        resetEncoders();
        gyroStrafe(0.35,20,0);
        resetEncoders();

        gyroDrive(0.35,-2,0);
        resetEncoders();

        gyroDrive(0.4,6,0);

        resetEncoders();

        runtime.reset();
        while(runtime.seconds() < 3 ){
            shootermotor.setPower(-0.8);
        }

        runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
        runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }  runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
        resetEncoders();
        gyroDrive(10,10,0);



    }
    public void AutonomousB(){

        resetEncoders();
        gyroDrive(0.35, 45, 0); //15
        resetEncoders();
        gyroStrafe(0.35, -13, 0);

        runtime.reset();
        while(runtime.seconds()<0.5){
            armmotor.setPower(-0.8);
        }
        armmotor.setPower(0);
        runtime.reset();
        while(runtime.seconds() < 1.5) {
            intakemotor.setPower(-0.35);
        }
        intakemotor.setPower(0);

        resetEncoders();
        gyroDrive(0.4,-23.5,0);

        resetEncoders();
        gyroDrive(0.3,2,0);
        // resetEncoders();
        //gyroDrive(0.4,5,0);

        runtime.reset();
        while(runtime.seconds() < 3 ){
            shootermotor.setPower(-0.8);
        }

        runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
        runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }  runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
        resetEncoders();
        gyroDrive(0.4,7,0);


    }
    public void AutonomousC(){
        resetEncoders();
        gyroDrive(0.35, 68, 0); //15
        resetEncoders();
        gyroStrafe(0.4, -35, 0);

        runtime.reset();
        while(runtime.seconds()<0.5){
            armmotor.setPower(-0.8);
        }
        armmotor.setPower(0);
        runtime.reset();
        while(runtime.seconds() < 1.5) {
            intakemotor.setPower(-0.35);
        }
        intakemotor.setPower(0);
        resetEncoders();

        gyroStrafe(0.4,22,0);
        resetEncoders();
        gyroDrive(0.4,-46.5,0);
        resetEncoders();
        gyroDrive(0.4,3,0);

        resetEncoders();
        // resetEncoders();
        //gyroDrive(0.4,5,0);

        runtime.reset();
        while(runtime.seconds() < 3 ){
            shootermotor.setPower(-0.8);
        }

        runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
        runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }  runtime.reset();
        while(runtime.seconds() < 1.5){
            feedermotor.setPower(-0.7);
        }
        runtime.reset();
        while(runtime.seconds()<2){
            feedermotor.setPower(0);
        }
        resetEncoders();
        gyroDrive(0.4,10,0);

    }


}