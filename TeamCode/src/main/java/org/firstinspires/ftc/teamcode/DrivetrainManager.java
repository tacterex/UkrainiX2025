package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class DrivetrainManager {
    public double r1, r2, l1, l2;

    PIDController target_follower_forward,
            target_follower_side,
            target_follower_turn;
    public static double p_forward = 0, i_forward = 0, d_forward = 0,
            p_side = 0, i_side = 0, d_side = 0,
            p_turn = 0, i_turn = 0, d_turn = 0;

    public DrivetrainManager(){
        target_follower_forward = new PIDController(
                p_forward, i_forward, d_forward
        );
        target_follower_side = new PIDController(
                p_side, i_side, d_side
        );
        target_follower_turn = new PIDController(
                p_turn, i_turn, d_turn
        );
    }

    public void calculate(double forward, double side, double turn, double shift){
        // Input
        double theta = Math.atan2(-forward, side);
        double power = Math.hypot(  side, -forward);

        // Calculations
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double l1Power = power * cos / max + turn;
        double r1Power = power * sin / max - turn;
        double l2Power = power * sin / max + turn;
        double r2Power = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            l1Power /= power + Math.abs(turn);
            r1Power /= power + Math.abs(turn);
            l2Power /= power + Math.abs(turn);
            r2Power /= power + Math.abs(turn);
        }

        // Setting power
        l1 = -(Math.max(-1, Math.min(1, l1Power)) * shift);
        r1 = (Math.max(-1, Math.min(1, r1Power)) * shift);
        l2 = (Math.max(-1, Math.min(1, l2Power)) * shift);
        r2 = (Math.max(-1, Math.min(1, r2Power)) * shift);

    }
}
