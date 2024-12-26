package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadUtility {
    public double ly, lx, rx, ry;

    boolean isRbumClick = false, isLbumClick = false, isRyClick = false;
    public boolean rbumClick, lBumClick;
    public int ryClick;

    public boolean cross, triangle, square, circle,
            home,
            dp_up, dp_down;

    Gamepad g;

    public GamepadUtility(Gamepad gamepad){
        g = gamepad;
    }

    public void update(){
        ly = g.left_stick_y;
        lx = g.left_stick_x;
        rx = g.right_stick_x;
        ry = g.right_stick_y;

        lBumClick = g.left_bumper && !isLbumClick;
        rbumClick = g.right_bumper && !isRbumClick;
        ryClick = (int) Math.signum(ry) * ((Math.abs(ry) >= 0.8 && !isRyClick) ? 1 : 0);
        isLbumClick = g.left_bumper;
        isRbumClick = g.right_bumper;
        isRyClick = Math.abs(ry) >= 0.8;

        circle = g.circle;
        triangle = g.triangle;
        cross = g.cross;

        home = g.ps;

        dp_up = g.dpad_up;
        dp_down = g.dpad_down;
    }
}
