package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class RobotBase extends OpMode {
    DcMotor r1, r2, l1, l2;
    DcMotor hand_motor, hand_extender;
    Servo grabber, adjuster, rotator;
    CRServo extender;

    // Servo positions
    final double grabber_zero = 0.57,
            grabber_max = 1,
            grabber_mid = (grabber_max + grabber_zero) / 2 - 0.0;
    private boolean is_grabber_mid;

    final double rotator_zero = 0.11,
            rotator_max = 0.82,
            rotator_mid = (rotator_max + rotator_zero) / 2;

    public static double p_arm = 0.0072, i_arm = 0.011 , d_arm = 0.00038, f_arm = 0.1;
    private PIDController armController;
    public static double arm_target;
    public int zero_position, min_position = -90, max_position = 90;
    public final double ticks_in_degree = 28 * 48.0 / 360 * 109.2 / 20.2;

    final double[] adjuster_possible_positions = {0.66, 0.58, 0.47};
    final int L = adjuster_possible_positions.length;
    int adjuster_position = 0;

    int extend = 0, min_extend = 0, pre_max_extend = 0, max_extend = 0;
    private PIDController extender_controller;

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
        hand_extender = hardwareMap.get(DcMotor.class, "hand_extender");

        extender = hardwareMap.get(CRServo.class, "extender");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(Servo.class, "grabber");

        drivetrain = new DrivetrainManager();

        hand_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hand_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController = new PIDController(p_arm, i_arm, d_arm);
        zero_position = hand_motor.getCurrentPosition() + (int)(45 * ticks_in_degree);
        arm_target = hand_motor.getCurrentPosition();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hand_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hand_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extender_controller = new PIDController(0.15, 0, 0.0007);
        min_extend = hand_extender.getCurrentPosition();
        pre_max_extend = min_extend - 110;
        max_extend = max_extend - 198;

        adjuster_position = 0;

        timer = new ElapsedTime();
        previous_timer = timer.seconds();

        first_cycle = true;
    }

    void update_arm(){
        arm_target = Range.clip(arm_target, min_position, max_position);
        int arm_position = hand_motor.getCurrentPosition();
        double pid = armController.calculate(arm_position, arm_target);
        double ff = Math.cos(Math.toRadians((arm_target - zero_position)
                / ticks_in_degree)) * f_arm;
        double power = pid + ff;
        hand_motor.setPower(power);
    }

    void start_position(){
        adjust(2);
        rotator.setPosition(rotator_zero);
        grabber.setPosition(grabber_zero);
        is_grabber_mid = false;
        set_arm_bound(-35, -35, 60);
    }

    void auto_start_position(){
        adjust(2);
        rotator.setPosition(rotator_max);
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

    public void set_arm_bound(double cur_degree, double min_degree, double max_degree){
        zero_position = hand_motor.getCurrentPosition() - (int)(cur_degree * ticks_in_degree);
        min_position = zero_position + (int)(min_degree * ticks_in_degree);
        max_position = zero_position + (int)(max_degree * ticks_in_degree);
    }

    public double getAngle(){
        return (hand_motor.getCurrentPosition() - zero_position) / ticks_in_degree;
    }

    public void setAngle(double angle){
        arm_target = zero_position + (angle * ticks_in_degree);
    }

    void update_extender(){
        int t = 0;
        if(extend == 0)
            t = min_extend;
        else
            t = getAngle() >= 30 ? max_extend : pre_max_extend;
        int p = hand_extender.getCurrentPosition();
        double pid = extender_controller.calculate(p, t);
        hand_extender.setPower(pid);
    }

    void printData(){
        telemetry.addData("Arm target", arm_target);
        telemetry.addData("Arm pos", hand_motor.getCurrentPosition());
        telemetry.addData("Zero pos", zero_position);
        telemetry.addData("Adj pos", adjuster_position);
        telemetry.addData("Power", hand_motor.getPower());
        telemetry.addLine();
        telemetry.addData("Min", min_position);
        telemetry.addData("Max", max_position);
        telemetry.addLine();
        telemetry.addData("Ext pos", hand_extender.getCurrentPosition());
        telemetry.addData("Min_ext", min_extend);
        telemetry.addData("Pre_max_ext", pre_max_extend);
        telemetry.addData("Max_ext", max_extend);
        telemetry.addData("Ext", extend);
        telemetry.addData("Ext_pow", hand_extender.getPower());
        telemetry.update();
    }
}
