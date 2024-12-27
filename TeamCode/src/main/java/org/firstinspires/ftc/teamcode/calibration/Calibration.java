package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotBase;

@TeleOp(name="calibration", group="Calibration")
public class Calibration extends RobotBase {
    @Override
    public void init(){
        hardware_setup();
    }

    @Override
    public void preStart(){
        calibrate();
        reset_arm();
    }

    @Override
    public void postStartLoop(){}

    void calibrate(){

    }
}
