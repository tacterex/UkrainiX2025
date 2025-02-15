package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TeleOp_Robot1_5 extends RobotBase{
    GamepadUtility util1, util2;

    final double ticks_per_s_up = 10200, ticks_per_s_down = 11500;

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
    }

    @Override
    public void postStartLoop(){
        util1.update();
        util2.update();

        //Driver1
        drivetrain.calculate(util1.ly, util1.lx, util1.rx * 0.78, 1);
        move();

        //Driver2
        if (util2.dp_up || util2.dp_down) {

        }
        else {
            arm_target += (
                    (-util2.ly) * (util2.ly > 0 ? ticks_per_s_down : ticks_per_s_up)
                            * (timer.seconds() - previous_timer)
            );
            update_arm();
        }

        if(util2.dpUpCLick){
            set_preangle(1);
        }
        if(util2.dpDownClick){
            set_preangle(-1);
        }

//        if(Math.abs(util2.ly) >= 0.05){
//            if(util2.ly > 0 && hand_motor.getCurrentPosition() < max_position){
//                hand_motor.setPower(-util2.ly);
//                arm_target = hand_motor.getCurrentPosition();
//            }
//            if(util2.ly < 0 && hand_motor.getCurrentPosition() > min_position){
//                hand_motor.setPower(-0.5 * util2.ly);
//                arm_target = hand_motor.getCurrentPosition();
//            }
//        }
//        else{
//            update_arm();
//        }

        //hand_motor.setPower(-util2.ly);
        if(util2.square)
            extend = 1;
        else if (util2.cross)
            extend = 0;
        update_extender();

        if(util2.triangleClick) delta += 5;
        if(util2.circleClick) delta -= 5;

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

        printData();

        previous_timer = timer.seconds();
    }
}
