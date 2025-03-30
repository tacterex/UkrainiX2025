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
    DcMotor specimen_placer;
    Servo grabber, adjuster, rotator;
    Servo leftSpecGrabber, rightSpecGrabber, specRotator, specPlacer;
    CRServo extender;

    // Servo positions
    final double grabber_zero = 0.25,
            grabber_max = 0.75,
            grabber_mid = (grabber_max + grabber_zero) / 2 - 0.0;
    private boolean is_grabber_mid;

    final double rotator_zero = 0.14,
            rotator_max = 0.83,
            rotator_mid = (rotator_max + rotator_zero) / 2;

    final double leftSpecGrabberZero = 0.19,
            leftSpecGrabberMax = 0.42,
            leftSpecGrabberMid = (leftSpecGrabberMax + leftSpecGrabberZero) / 2;

    final double rightSpecGrabberZero = 0.42,
            rightSpecGrabberMax = 0.65,
            rightSpecGrabberMid = (rightSpecGrabberMax + rightSpecGrabberZero) / 2;

    final double specRotatorZero = 0.01,
            specRotatorMax = 0.73,
            specRotatorMid = (specRotatorMax + specRotatorZero) / 2;

    public static double p_arm = 0.0072, i_arm = 0.011 , d_arm = 0.00078, f_arm = 0.1;
    private PIDController armController;
    public static double arm_target;
    public int zero_position, min_position = -90, max_position = 90;
    public final double ticks_in_degree = 28 * 48.0 / 360 * 109.2 / 20.2;
    final double[] arm_angles = {-21.4, -14.5, 50, 60};
    int cur_target_angle = 0;

    final double[] adjuster_possible_positions = {0.67, 0.6, 0.485};
    final int L = adjuster_possible_positions.length;
    int adjuster_position = 0;

    int extend = 0, min_extend = 0, pre_max_extend = 0, max_extend = 0;
    private PIDController extender_controller;
    int delta;

    boolean specimen = false;
    final double no_spec = 0.705, is_spec = 0.4, drop_spec = 0.0;
    public static double p_spec = 0.022, i_spec = 0.28, d_spec = 0.00308, f_spec = 0.34;
    final int zero_spec = 36;
    final double ticks_in_degree_spec = 288.0 / 360;
    private PIDController specimen_controller;

    DrivetrainManager drivetrain;

    ElapsedTime timer, delayTimer;
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
        specimen_placer = hardwareMap.get(DcMotor.class, "spec_placer");

        adjuster = hardwareMap.get(Servo.class, "adjuster");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(Servo.class, "grabber");
        is_grabber_mid = false;

        leftSpecGrabber = hardwareMap.get(Servo.class, "lsg");
        rightSpecGrabber = hardwareMap.get(Servo.class, "rsg");
        specRotator = hardwareMap.get(Servo.class, "rot_s");
        specPlacer = hardwareMap.get(Servo.class, "sps");

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
        pre_max_extend = min_extend - 130;
        max_extend = max_extend - 230;
        delta = 0;

        specimen_placer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimen_placer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        specimen_controller = new PIDController(p_spec, i_spec, d_spec);

        adjuster_position = 0;

        timer = new ElapsedTime();
        delayTimer = new ElapsedTime();
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
        set_arm_bound(-35, -35, 65);
        set_preangle(1);

        specRotator.setPosition(specRotatorMax);
        rightSpecGrabber.setPosition(rightSpecGrabberZero);
        leftSpecGrabber.setPosition(leftSpecGrabberMax);
    }

    void auto_start_position(){
        specPlacer.setPosition(no_spec);
        specRotator.setPosition(specRotatorMax);
        rightSpecGrabber.setPosition(rightSpecGrabberZero);
        leftSpecGrabber.setPosition(leftSpecGrabberMax);
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

    void flip_spec(){
        specRotator.setPosition(
                specRotator.getPosition() >= specRotatorMid ? specRotatorZero : specRotatorMax
        );
    }

    void grab_spec(){
        leftSpecGrabber.setPosition(
                leftSpecGrabber.getPosition() >= leftSpecGrabberMid ? leftSpecGrabberZero : leftSpecGrabberMax
        );
        rightSpecGrabber.setPosition(
                rightSpecGrabber.getPosition() >= rightSpecGrabberMid ? rightSpecGrabberZero : rightSpecGrabberMax
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
            t = getAngle() >= 30 ? (max_extend - delta) : (pre_max_extend - delta);
        int p = hand_extender.getCurrentPosition();
        double pid = extender_controller.calculate(p, t);
        hand_extender.setPower(pid);
    }

    void update_specimen(){
        double t = (specimen ? is_spec : no_spec);
//        int p = specimen_placer.getCurrentPosition();
//        double pid = specimen_controller.calculate(p, t);
//        double ff = f_spec * Math.cos(Math.toRadians((t - zero_spec)
//                / ticks_in_degree_spec));
//        double power = pid + ff;
//        power = Range.clip(power, -0.7, 0.7);
//        specimen_placer.setPower(power);

        specPlacer.setPosition(t);
    }

    void set_preangle(int index){
        cur_target_angle = Range.clip(cur_target_angle + index,
                0, arm_angles.length - 1);
        setAngle(arm_angles[cur_target_angle]);
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
        telemetry.addLine();
        telemetry.addData("Spec pos", specimen_placer.getCurrentPosition());
        telemetry.addData("Spec target", (specimen ? is_spec : no_spec));
        telemetry.update();
    }
}
