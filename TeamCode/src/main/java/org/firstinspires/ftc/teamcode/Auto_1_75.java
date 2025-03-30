package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Auto_1_75 extends RobotBase{
    @Override
    public void init(){
        hardware_setup();
    }

    @Override
    public void preStart(){
        auto_start_position();
        reset();

        while(delayTimer.milliseconds() <= 3000) {continue;}

        moveForTime(600, -1, 0, 0);
        reset();
        specPlacer.setPosition(is_spec);
        specRotator.setPosition(specRotatorZero);
        while(delayTimer.milliseconds() <= 1500) {continue;}

        moveForTime(1000, -1, 0, 0);
        moveForTime(70, 1, 0, 0);
        delay(500);

        reset();
        specPlacer.setPosition(drop_spec);
        while(delayTimer.milliseconds() <= 1200) {continue;}

        reset();
        grab_spec();
        while(delayTimer.milliseconds() <= 500) {continue;}

        moveForTime(600, 1, 0, 0);
        grab_spec();
        specRotator.setPosition(specRotatorMax);
        specPlacer.setPosition(no_spec);
        moveForTime(3000,0, 1, 0);
        grab_spec();
        moveForTime(800, 1, 0, 0);
        reset();
        while(delayTimer.milliseconds() <= 2000){continue;}
        grab_spec();
        delay(1500);

        specRotator.setPosition(specRotatorZero);
        specPlacer.setPosition(is_spec);
        moveForTime(400, -1, 0, 0);
        moveForTime(3290,0, -1, 0);
        moveForTime(1500, -1, 0, 0);
        moveForTime(70, 1, 0, 0);
        delay(500);

        reset();
        specPlacer.setPosition(drop_spec);
        while(delayTimer.milliseconds() <= 1200) {continue;}

        reset();
        grab_spec();
        while(delayTimer.milliseconds() <= 500) {continue;}

        moveForTime(600, 1, 0, 0);
        grab_spec();
        specRotator.setPosition(specRotatorMax);
        specPlacer.setPosition(no_spec);
        moveForTime(3500,0, 1, 0);
        grab_spec();
        moveForTime(800, 1, 0, 0);
        reset();
        grab_spec();
    }

    @Override
    public void postStartLoop(){
        if(delayTimer.milliseconds() <= 1000) return;
        reset();
    }

    void reset(){
        delayTimer.reset();
    }

    void moveForTime(int ms, double f, double s, double t){
        reset();
        drivetrain.calculate(f, s, t, 0.5);
        while(delayTimer.milliseconds() <= ms) {move();}
        stop_move();
    }

    void stop_move(){
        drivetrain.calculate(0, 0, 0, 0);
        move();
    }

    void delay(int ms){
        delayTimer.reset();
        while(delayTimer.milliseconds() <= ms) {continue;}
    }
}
