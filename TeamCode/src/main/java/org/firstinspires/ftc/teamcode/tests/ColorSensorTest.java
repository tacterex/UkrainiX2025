package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ColorSensorTest", group="Tests")
public class ColorSensorTest extends LinearOpMode {
    ColorSensor sensor;

    @Override
    public void runOpMode(){
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Alpha", sensor.alpha());
            telemetry.update();
        }
    }
}
