package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot1_5", group="FTC25")
public class TeleOp_Robot1_5 extends RobotBase{
    @Override
    public void runOpMode(){
        hardware_setup();
        GamepadUtility util1 = new GamepadUtility();

        waitForStart();
        while(opModeIsActive()){
            util1.update(gamepad1);
            drivetrain.calculate(util1.ly, util1.lx, util1.rx * 0.6, 1);
            move();
        }
    }
}
