package org.firstinspires.ftc.teamcode;

public class Auto_Robot1_5 extends RobotBase{
    @Override
    public void init(){
        hardware_setup();
    }

    @Override
    public void preStart(){
        zero_position = (int)(45 * ticks_in_degree);
        arm_target = hand_motor.getCurrentPosition() + 100 * ticks_in_degree;
        auto_start_position();
    }

    @Override
    public void postStartLoop(){
        extender.setPower(timer.milliseconds() <= 3000 ? -1 : 0);
        update_arm();
    }
}
