package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RobotBase extends LinearOpMode {
    DcMotor r1, r2, l1, l2;
    DcMotor hand_motor;
    Servo grabber, adjuster, rotator;
    CRServo extender;

    // Servo positions
    final double grabber_zero = 0,
            grabber_max = 1,
            grabber_mid = (grabber_max + grabber_zero) / 2;

    final double rotator_zero = 0,
            rotator_max = 1,
            rotator_mid = (rotator_max + rotator_zero) / 2;

    final double[] adjuster_possible_positions = {0, 0.2, 0.4, 0.6, 0.8, 1};
    final int L = adjuster_possible_positions.length;
    int adjuster_position = 0;

    DrivetrainManager drivetrain;

    ElapsedTime timer;

    void hardware_setup(){
        l1 = hardwareMap.get(DcMotor.class, "l1");
        r1 = hardwareMap.get(DcMotor.class, "r1");
        l2 = hardwareMap.get(DcMotor.class, "l2");
        r2 = hardwareMap.get(DcMotor.class, "r2");
        hand_motor = hardwareMap.get(DcMotor.class, "hand_motor");
        extender = hardwareMap.get(CRServo.class, "extender");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(Servo.class, "grabber");

        drivetrain = new DrivetrainManager();

        timer = new ElapsedTime();
    }

    //Useful functions

    void move(){
        l1.setPower(drivetrain.l1);
        l2.setPower(drivetrain.l2);
        r1.setPower(drivetrain.r1);
        r2.setPower(drivetrain.r2);
    }

    void grab_sample(){
        grabber.setPosition(
                grabber.getPosition() >= grabber_mid ? grabber_zero : grabber_max
        );
    }

    void flip(){
        rotator.setPosition(
                rotator.getPosition() >= rotator_mid ? rotator_zero : rotator_max
        );
    }

    void adjust(int index){
        if(index != 0){
            adjuster_position =
                    Math.min(L-1,
                            Math.max(0,
                                    adjuster_position + index));
            adjuster.setPosition(adjuster_possible_positions[adjuster_position]);
        }
    }
}
