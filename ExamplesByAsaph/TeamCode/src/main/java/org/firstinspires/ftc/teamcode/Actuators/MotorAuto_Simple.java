package org.firstinspires.ftc.teamcode.Actuators;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * This file
 */

@TeleOp(name="Using a DC Motor", group="Linear Opmode")
//@Disabled
public class MotorAuto_Simple extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        // 28 counts of the encoder is one rotation of the motor shaft
        // 25 is the the gearbox reduction
        // This example rotates the wheel once
        leftDrive.setTargetPosition(28*25);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(opModeIsActive() && leftDrive.isBusy() ){ //isbusy returns true while the motor's encoder is still counting less than 28*25

            leftDrive.setPower(1); // This sets the max power that the motor can use.
        }

        leftDrive.setPower(0);

        }
    }
