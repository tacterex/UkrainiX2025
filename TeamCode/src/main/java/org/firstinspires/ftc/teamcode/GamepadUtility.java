package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadUtility {
    public double ly, lx, rx, ry;

    private boolean isRbumClick = false, isLbumClick = false;
    public boolean rbumClick, lBumClick;

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
        isLbumClick = g.left_bumper;
        isRbumClick = g.right_bumper;

        circle = g.circle;
        triangle = g.triangle;
        cross = g.cross;
        square = g.square;
    }
}
