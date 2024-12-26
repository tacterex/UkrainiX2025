package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot1_5", group="FTC25")
public class TeleOp_Robot1_5 extends RobotBase{
    GamepadUtility util1, util2;

    final double ticks_per_ms = 1;

    @Override
    public void runOpMode(){
        hardware_setup();
        util1 = new GamepadUtility(gamepad1); // Gamepad Setup
        util2 = new GamepadUtility(gamepad2);

        waitForStart();
        start_position();
        previous_timer = timer.milliseconds();

        while(opModeIsActive()){
            util1.update();
            util2.update();

            //Driver1
            drivetrain.calculate(util1.ly, util1.lx, util1.rx * 0.6, 1);
            move();

            //Driver2
            if(util2.dp_up || util2.dp_down)
                hand_motor.setPower(
                        (util2.dp_down ? 1 : 0) - (util2.dp_up ? 1 : 0)
                );
            else {
                arm_target += (int)(
                        util2.ly * ticks_per_ms * (timer.milliseconds() - previous_timer)
                );
                update_arm();
            }

            //hand_motor.setPower(-util2.ly);

            extender.setPower(
                    (util2.cross ? 1 : 0) - (util2.square ? 1 : 0)
            );

            if(util2.rbumClick){
                grab_sample();
                gamepad2.rumble(300);
            }
            if(util2.lBumClick) flip();
            adjust(-util2.ryClick);

            if(util2.home) reset_arm();

            telemetry.addData("Arm target", arm_target);
            telemetry.update();

            previous_timer = timer.milliseconds();
        }
    }
}
