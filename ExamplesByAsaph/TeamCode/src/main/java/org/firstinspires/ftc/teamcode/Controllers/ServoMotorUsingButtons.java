package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file
 */

@TeleOp(name="Using Buttons to Control Servo", group="Linear Opmode")
@Disabled
public class ServoMotorUsingButtons extends LinearOpMode {

    // Declare OpMode members.
    Servo servo;

    @Override
    public void runOpMode() {

        servo  = hardwareMap.get(Servo.class, "Claw");
        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.a){
                servo.setPosition(1);
            } else if(gamepad2.y){
                servo.setPosition(0);
            } else{
                servo.setPosition(0.5);
            }
        }
    }
}
