/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 *
 *
 * https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Using-TensorFlow-Lite-for-the-Ultimate-Goal-Challenge
 */

@Autonomous(name = "Blue left", group = "Concept")
@Disabled

public class TensorFlowObjectDetectionWebcam extends LinearOpMode {

    /////////////////////////////////////////////////////////////////
    FtcDashboard dashboard;

    BNO055IMU imu;
    Orientation angles;
    // Declare OpMode members.
    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;
    private DcMotorEx shooter = null;

    double integral=0;
    public static PIDFCoefficients DrivetrainPID = new PIDFCoefficients(25,0.05,1.25,0);
    public static  double     P_DRIVE_COEFF = 0.00202674492;

    public static  double      Target = 50;
    public double greatest_dist = 0;

    static final double     COUNTS_PER_MOTOR_REV    =  746.6 ;// Normal drivetrain 704.86 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /////////////////////////////////////////////////////////////////
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    String rings;
    char Auto;
    private static final String VUFORIA_KEY = "AVuwooL/////AAABmVcNitcih0Fkhok8emRlRE1As/ewLXPEdUwV+ijYqYxoVRP+agLwn1kFcLodqjKMD3EXLlv6BKliVOYVVDJz3lIysaHnRsJ9CT9sIZZwQBJ2tmqLP52VM8qB0Kq7mhs+Qu7J5fz6ritQhbSEiFziJ/teJ+N2XkkvSNXTtwMfdDgXLJ1xaxjEmX+r0o4dJWcpU6ar4JX+S2U0LO3JkmQjxXXOOFBGtJiH8t1yw2t3QqJT5AjJxHhkcBlWOU6AQqHdKxiyRugdQa6fHWb9i0rCiv22lwTELzRxfZNckR+iZKTpngqHNzMpfZloGO8rdOe5IAcAe5BA3HU0nUMzglzgdPwWCEImFVSShV7lxhIVRa2c";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        initVuforia();
        initTfod();

        //////////////////////////////////////////////////////////////////////////////////////
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        left1  = (DcMotorEx)hardwareMap.get(DcMotor.class, "left1");
        left2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "left2");
        right1  = (DcMotorEx)hardwareMap.get(DcMotor.class,   "right1");
        right2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "right2");
        shooter = (DcMotorEx)hardwareMap.get(DcMotor.class, "shooter");

        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        left2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);
        right2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DrivetrainPID);


        //currentPID = left1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoders();
        //////////////////////////////////////////////////////////////////////////////////////

        telemetry.addLine("initialized");
        telemetry.update();

        waitForStart();

        //////////////////////// DON'T CHANGE ANYTHING BELOW /////////////////////////////////


        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0/9.0);// was 2.5
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
           // while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        rings = recognition.getLabel();
                        //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                          //      recognition.getLeft(), recognition.getTop());
                        //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                       //         recognition.getRight(), recognition.getBottom());
                      }
                      telemetry.update();
                    }
                }
           // }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

            if(rings == "Single"){
                Auto = 'B';
                telemetry.addLine("There is one ring, Running B");
            } else if (rings == "Quad"){
                Auto = 'C';
                telemetry.addLine("There are four rings, running C");
            } else {
                Auto = 'A';
                telemetry.addLine("There is none, running A");
            }

        //////////////////////// DON'T CHANGE ANYTHING ABOVE /////////////////////////////////


        gyroDrive(0.7,-Target,0, dashboardTelemetry);

        switch (Auto){
            case 'C':
                telemetry.addLine("Auto: C");
                break;
            case 'B':
                telemetry.addLine("Auto: B");
                break;
            default:
                telemetry.addLine("Auto: A");
                AutonomousA(dashboardTelemetry);

        }

    }
    public void AutonomousA(Telemetry dashboardTelemetry){
        resetEncoders();
       // gyroDrive(0.7,-Target,0, dashboardTelemetry);
    }
    public void gyroDrive(double speed, double distance, double angle,Telemetry dashboardTelemetry ) {

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
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left1.setPower(leftSpeed);
                left2.setPower(leftSpeed);

                right1.setPower(rightSpeed);
                right2.setPower(rightSpeed);

                dashboardTelemetry.addData("Left Speed", leftSpeed);
                dashboardTelemetry.addData("Right Speed", rightSpeed);
                print(moveCounts,dashboardTelemetry);

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
    public double getCurrentPosition(){
        return Math.abs(left1.getCurrentPosition() + left2.getCurrentPosition()
                + right1.getCurrentPosition() + right2.getCurrentPosition())/4;
        // Math.abs returns the absolute value of an argument
    }
    public double getErrorAngle(double targetAngle) {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
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

        dashboardTelemetry.addData("Angle", angles.firstAngle);
        dashboardTelemetry.addData("Distance", getCurrentPosition()/COUNTS_PER_INCH);
        dashboardTelemetry.update();

    }













    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
