package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TeleOp_Robot1_5 extends RobotBase{
    GamepadUtility util1, util2;

    final double ticks_per_s_up = 4752, ticks_per_s_down = 4252;

    @Override
    public void init(){
        hardware_setup();
        util1 = new GamepadUtility(gamepad1); // Gamepad Setup
        util2 = new GamepadUtility(gamepad2);
    }

    @Override
    public void preStart(){
        start_position();
        previous_timer = timer.seconds();
        set_arm_bound(-55, -45, 90);
    }

    @Override
    public void postStartLoop(){
        util1.update();
        util2.update();

        //Driver1
        drivetrain.calculate(util1.ly, util1.lx, util1.rx * 0.78, 1);
        move();

        //Driver2
        if (util2.dp_up || util2.dp_down)
            hand_motor.setPower(
                    (util2.dp_down ? 1 : 0) - (util2.dp_up ? 1 : 0)
            );
        else {
            arm_target += (
                    (-util2.ly) * (util2.ly > 0 ? ticks_per_s_down : ticks_per_s_up)
                            * (timer.seconds() - previous_timer)
            );
            update_arm();
        }

        //hand_motor.setPower(-util2.ly);

        extender.setPower(
                (util2.cross ? 1 : 0) - (util2.square ? 1 : 0)
        );

        if (util2.rbumClick) {
            grab_sample();
            gamepad2.rumble(300);
        }
        if(util2.rTrigClick){
            mid_grab();
            gamepad2.rumble(300);
        }

        if (util2.lBumClick) flip();
        adjust(-util2.ryClick);

        if (util2.home) {
            reset_arm();
            gamepad2.rumble(200);
        }
        if(util2.lTrigClick) pos_specimen();

        telemetry.addData("Arm target", arm_target);
        telemetry.addData("Arm pos", hand_motor.getCurrentPosition());
        telemetry.addData("Zero pos", zero_position);
        telemetry.addData("Adj pos", adjuster_position);
        telemetry.addData("Power", hand_motor.getPower());
        telemetry.update();

        previous_timer = timer.seconds();
    }
}
