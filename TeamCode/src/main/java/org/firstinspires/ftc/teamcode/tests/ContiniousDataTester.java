package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class ContiniousDataTester extends LinearOpMode {
    private double previous_timer;
    public static double value;
    private final double change_rate = 0.005;

    private ElapsedTime timer;

    @Override
    public void runOpMode(){
        value = 0;
        timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        previous_timer = timer.seconds();

        while(opModeIsActive()){
            value += (-gamepad1.left_stick_y) * change_rate;
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Value", value);
            telemetry.update();

            previous_timer = timer.seconds();
        }
    }
}
