package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="PowerTester", group="Tests")
public class PowerTester extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Actual power", motor.getPower());
        }
    }
}
