package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GamepadTest", group="Tests")
public class GamepadTest extends LinearOpMode {
    GamepadUtility utility;

    @Override
    public void runOpMode(){
        waitForStart();
        utility = new GamepadUtility(gamepad1);

        while(opModeIsActive()){
            utility.update();
            telemetry.addData("ly", utility.ly);
            if(utility.rbumClick){
                gamepad1.rumble(100);
            }
            telemetry.update();
        }
    }
}
