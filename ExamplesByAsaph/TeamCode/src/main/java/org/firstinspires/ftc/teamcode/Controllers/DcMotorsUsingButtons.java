package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * This file
 */

@TeleOp(name="Using Buttons to Control motors", group="Linear Opmode")
@Disabled
public class DcMotorsUsingButtons extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor feeder = null;
    private DcMotor shooter = null;


    @Override
    public void runOpMode() {

        feeder  = hardwareMap.get(DcMotor.class, "feeder");
        shooter  = hardwareMap.get(DcMotor.class, "feeder");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.a){
                feeder.setPower(1);
            } else if(gamepad2.y){
                feeder.setPower(-1);
            } else{
                feeder.setPower(0);
            }

            if(gamepad2.dpad_up){
                shooter.setPower(1);
            } else if(gamepad2.dpad_down){
                shooter.setPower(-1);
            } else{
                shooter.setPower(0);
            }

        }


        }
    }
