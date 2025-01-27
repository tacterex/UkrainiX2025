package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class Auto_Robot1_5 extends RobotBase{
    @Override
    public void init(){
        hardware_setup();

        hand_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hand_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void preStart(){
        zero_position = hand_motor.getCurrentPosition() + (int)(45 * ticks_in_degree);
        arm_target = hand_motor.getCurrentPosition() + 90 * ticks_in_degree;
        auto_start_position();
    }

    @Override
    public void postStartLoop(){
        extender.setPower(timer.milliseconds() <= 6000 ? -0.5 : 0);
        if(timer.milliseconds() >= 1500) update_arm();
    }
}
