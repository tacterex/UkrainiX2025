package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot1_5", group="FTC25")
public class TeleOp_Robot1_5 extends RobotBase{
    @Override
    public void runOpMode(){
        hardware_setup();
        GamepadUtility util1 = new GamepadUtility(gamepad1); // Gamepad Setup
        GamepadUtility util2 = new GamepadUtility(gamepad2);

        waitForStart();
        while(opModeIsActive()){
            util1.update();
            util2.update();

            //Driver1
            drivetrain.calculate(util1.ly, util1.lx, util1.rx * 0.6, 1);
            move();

            //Driver2
            hand_motor.setPower(util2.ly);
            extender.setPower(
                    (util2.cross ? 1 : 0) - (util2.triangle ? 1 : 0)
            );

            if(util2.rbumClick) grab_sample();
            if(util2.lBumClick) flip();
            adjust(util2.ryClick);
            timer.reset();
        }
    }
}
