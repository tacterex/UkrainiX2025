package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadUtility {
    public double ly, lx, rx, ry;

    boolean isRbumClick = false, isLbumClick = false, isLyClick = false;
    public boolean rbumClick, lBumClick;
    public int lyClick;

    public boolean cross, triangle, square, circle;

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
        lyClick = (int) Math.signum(ly) * ((Math.abs(ly) >= 0.8 && !isLyClick) ? 1 : 0);
        isLbumClick = g.left_bumper;
        isRbumClick = g.right_bumper;
        isLyClick = Math.abs(ly) >= 0.8;

        circle = g.circle;
        triangle = g.triangle;
        cross = g.cross;
        square = g.square;
    }
}
