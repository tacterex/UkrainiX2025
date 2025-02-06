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
        set_arm_bound(-35, -35, 60);
        setAngle(7.5);
        auto_start_position();
        extend = 0;
        timer.reset();
    }

    @Override
    public void postStartLoop(){
        update_arm();
        update_extender();

        printData();

        if(timer.milliseconds() <= 10000){
            setAngle(7.5);
            return;
        }
        if(timer.milliseconds() <= 11000){
            rotator.setPosition(rotator_max);
            return;
        }
        if(timer.milliseconds() <= 11900){
            drivetrain.calculate(-1, 0, 0, 0.5);
            move();
            return;
        }
        if(timer.milliseconds() <= 13000){
            drivetrain.calculate(0, 0, 0, 0);
            move();
            return;
        }
        if(timer.milliseconds() <= 14000){
            setAngle(-6);
            return;
        }
        if(timer.milliseconds() <= 14500){
            grabber.setPosition(grabber_max);
            return;
        }
        if(timer.milliseconds() <= 15000){
            rotator.setPosition(rotator_mid);
            return;
        }
        if(timer.milliseconds() <= 16000){
            rotator.setPosition(rotator_mid);
            return;
        }
        if(timer.milliseconds() <= 17000){
            drivetrain.calculate(0.5, 0, 0, 0.5);
            move();
            return;
        }
        if(timer.milliseconds() <= 18000){
            rotator.setPosition(rotator_max);
            return;
        }
        if(timer.milliseconds() <= 19000){
            drivetrain.calculate(0, 0, 0, 0);
            move();
            setAngle(-35);
            return;
        }
    }
}
