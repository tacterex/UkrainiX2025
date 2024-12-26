package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class RobotBase extends LinearOpMode {
    DcMotor r1, r2, l1, l2;
    DcMotor hand_motor;
    Servo grabber, adjuster, rotator;
    CRServo extender;

    // Servo positions
    final double grabber_zero = 0.3,
            grabber_max = 0.85,
            grabber_mid = (grabber_max + grabber_zero) / 2;

    final double rotator_zero = 0.06,
            rotator_max = 0.77,
            rotator_mid = (rotator_max + rotator_zero) / 2;

    public static double p_arm, i_arm, d_arm, f_arm;
    PIDController armController;
    public static int arm_target, zero_position;
    public final double ticks_in_degree = 28 * 100.0 / 360;

    final double[] adjuster_possible_positions = {0, 0.14, 0.26, 0.3, 0.4};
    final int L = adjuster_possible_positions.length;
    int adjuster_position = 0;

    DrivetrainManager drivetrain;

    ElapsedTime timer;
    double previous_timer;

    final int[][] gamepadColors = {
            {255, 0, 255},
            {255, 255, 0},
            {0, 200, 115},
            {125, 0, 255},
            {255, 0, 125},
            {255, 255, 255}
    };

    public void hardware_setup(){
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

        armController = new PIDController(p_arm, i_arm, d_arm);
        zero_position = StorageManager.load_calibration();
        arm_target = hand_motor.getCurrentPosition();

        timer = new ElapsedTime();
        previous_timer = 0;
    }

    void update_arm(){
        int arm_position = hand_motor.getCurrentPosition();
        double pid = armController.calculate(arm_position, arm_target);
        double ff = Math.cos(Math.toRadians((arm_target - zero_position)
                / ticks_in_degree)) * f_arm;
        double power = pid + ff;
        hand_motor.setPower(power);
    }

    void start_position(){
        adjust(-1);
        rotator.setPosition(rotator_zero);
        grabber.setPosition(grabber_zero);
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
            adjuster_position = Range.clip(
                    adjuster_position + index,
                    0,
                    L-1
            );
            adjuster.setPosition(adjuster_possible_positions[adjuster_position]);
            gamepad2.setLedColor(
                    gamepadColors[adjuster_position][0],
                    gamepadColors[adjuster_position][1],
                    gamepadColors[adjuster_position][2],
                    10
            );
        }
    }

    public void reset_arm(){
        zero_position = hand_motor.getCurrentPosition();
        StorageManager.save_calibration(zero_position);
    }
}
