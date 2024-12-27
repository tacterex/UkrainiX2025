package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotBase;

@TeleOp(name="calibration", group="Calibration")
public class Calibration extends RobotBase {
    @Override
    public void runOpMode(){
        hardware_setup();

        waitForStart();
        while(opModeIsActive()) {
            calibrate();
            reset_arm();
        }
    }

    void calibrate(){

    }
}
