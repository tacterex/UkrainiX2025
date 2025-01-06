package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class RobotBase extends OpMode {
    DcMotor r1, r2, l1, l2;
    DcMotor hand_motor;
    Servo grabber, adjuster, rotator;
    CRServo extender;

    // Servo positions
    final double grabber_zero = 0.2,
            grabber_max = 0.7,
            grabber_mid = (grabber_max + grabber_zero) / 2 - 0.0;
    private boolean is_grabber_mid;

    final double rotator_zero = 0.06,
            rotator_max = 0.76,
            rotator_mid = (rotator_max + rotator_zero) / 2;

    public static double p_arm = 0.0117, i_arm = 0.011 , d_arm = 0.00074, f_arm = 0.17;
    private PIDController armController;
    public static double arm_target;
    public int zero_position;
    public final double ticks_in_degree = 28 * 100.0 / 360 * 6 / 1.875;

    final double[] adjuster_possible_positions = {0.64, 0.5, 0.33};
    final int L = adjuster_possible_positions.length;
    int adjuster_position = 0;

    DrivetrainManager drivetrain;

    ElapsedTime timer;
    double previous_timer;

    final double[][] gamepadColors = {
            {1, 0.2, 0},
            {1, 1, 0},
            {0, 0.8, 0.4},
            {1, 0, 0.5},
            {0.5, 0, 1},
            {1, 1, 1}
    };

    private boolean first_cycle;

    public abstract void preStart();
    public abstract void postStartLoop();

    @Override
    public final void loop(){
        if(first_cycle){
            preStart();
            first_cycle = false;
        }
        else{
            postStartLoop();
        }
    }

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        previous_timer = timer.seconds();

        first_cycle = true;
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
        is_grabber_mid = false;
    }

    //Useful functions
    void move(){
        l1.setPower(drivetrain.l1);
        l2.setPower(drivetrain.l2);
        r1.setPower(drivetrain.r1);
        r2.setPower(drivetrain.r2);
    }

    void grab_sample(){
        if(is_grabber_mid) grabber.setPosition(grabber_zero);
        else
            grabber.setPosition(
                    grabber.getPosition() >= grabber_mid ? grabber_zero : grabber_max
            );
        is_grabber_mid = false;
    }

    void pos_specimen(){
        rotator.setPosition(rotator_mid);
    }

    void mid_grab(){
        grabber.setPosition(grabber_mid);
        is_grabber_mid = true;
    }

    void flip(){
        rotator.setPosition(
                rotator.getPosition() >= rotator_mid ? rotator_zero : rotator_max
        );
    }

    void adjust(int index){
        if(index > 0 || index == -1){
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
                    Gamepad.LED_DURATION_CONTINUOUS
            );
        }
    }

    public void reset_arm(){
        zero_position = hand_motor.getCurrentPosition();
        StorageManager.save_calibration(zero_position);
    }
}
