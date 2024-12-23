package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoCalibrate", group="Tests")
public class ServoCalibrate extends LinearOpMode {
    Servo to_calibrate;

    double servo_position = 0;

    @Override
    public void runOpMode(){
        to_calibrate = hardwareMap.get(Servo.class, "to_calibrate");

        final double step = 0.05, small_step = 0.01;
        GamepadUtility util2 = new GamepadUtility(gamepad2);

        waitForStart();
        while(opModeIsActive()){
            util2.update();
            if(util2.rbumClick)
                servo_position = Math.min(1, servo_position + step);
            if(util2.lBumClick)
                servo_position = Math.max(0, servo_position - step);
            if(util2.ryClick > 0)
                servo_position = Math.min(1, servo_position + small_step);
            if(util2.ryClick < 0)
                servo_position = Math.max(0, servo_position - small_step);

            to_calibrate.setPosition(servo_position);
            telemetry.addData("Position", servo_position);
            telemetry.update();
        }
    }
}
