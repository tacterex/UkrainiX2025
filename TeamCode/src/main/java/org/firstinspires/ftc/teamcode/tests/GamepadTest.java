package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GamepadUtility;

@TeleOp(name="GamepadTest", group="Tests")
public class GamepadTest extends LinearOpMode {
    GamepadUtility utility;
    ElapsedTime timer;

    double value = 0, previous_timer = 0;
    final double value_per_ms = 0.5;

    @Override
    public void runOpMode(){
        waitForStart();
        timer = new ElapsedTime();
        timer.reset();
        utility = new GamepadUtility(gamepad1);

        while(opModeIsActive()){
            utility.update();

            value += (timer.milliseconds() - previous_timer)
                    * (-utility.ly)
                    * value_per_ms;

            telemetry.addData("ly", utility.ly);
            telemetry.addData("Value", value);

            if(utility.rbumClick){
                gamepad1.rumble(100);
            }
            telemetry.update();
            previous_timer = timer.milliseconds();
        }
    }
}
