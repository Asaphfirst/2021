package org.firstinspires.ftc.teamcode.MajorWork;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * This file
 */

@TeleOp(name="Major work", group="Linear Opmode")
//@Disabled
public class Major extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    private DcMotor lift = null;



    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "l1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "l2");
        rightDrive  = hardwareMap.get(DcMotor.class, "r1");
        rightDrive2  = hardwareMap.get(DcMotor.class, "r2");
        lift  = hardwareMap.get(DcMotor.class, "lift");


        double leftPower;
        double rightPower;

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){
                lift.setPower(-0.5);
            } else if(gamepad1.y){
                lift.setPower(0.5);
            } else{
                lift.setPower(0);
            }
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -0.35, 0.35);
            rightPower = Range.clip(drive - turn, -0.35, 0.35);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(-rightPower);
            leftDrive2.setPower(leftPower);
            rightDrive2.setPower(-rightPower);

        }


    }
}
