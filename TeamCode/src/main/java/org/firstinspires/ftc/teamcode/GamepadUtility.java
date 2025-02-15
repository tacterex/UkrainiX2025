package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadUtility {
    public double ly, lx, rx, ry;

    boolean isRbumClick = false, isLbumClick = false, isRyClick = false,
            isLTrigClick = false, isRTrigClick = false,
            isDpUpCLick = false, isDpDownClick = false,
            isDpRightClick = false, isDpLeftClick = false,
            isTriangleClick = false, isCircleClick = false;
    public boolean rbumClick, lBumClick,
            lTrigClick, rTrigClick,
            dpUpCLick, dpDownClick, dpRightClick, dpLeftClick,
            triangleClick, circleClick;
    public int ryClick;

    public boolean cross, triangle, square, circle,
            home,
            dp_up, dp_down, dp_right, dp_left,
            rTrig;

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
        lTrigClick = (g.left_trigger > 0) && !isLTrigClick;
        rTrigClick = (g.right_trigger > 0) && !isRTrigClick;
        dpUpCLick = g.dpad_up && !isDpUpCLick;
        dpDownClick = g.dpad_down && !isDpDownClick;
        dpRightClick = g.dpad_right && !isDpRightClick;
        dpLeftClick = g.dpad_left && !isDpLeftClick;
        triangleClick = g.triangle && ! isTriangleClick;
        circleClick = g.circle && !isCircleClick;

        isLbumClick = g.left_bumper;
        isRbumClick = g.right_bumper;
        isRyClick = Math.abs(ry) >= 0.8;
        isLTrigClick = g.left_trigger > 0;
        isRTrigClick = g.right_trigger > 0;
        isDpUpCLick = g.dpad_up;
        isDpDownClick = g.dpad_down;
        isDpRightClick = g.dpad_right;
        isDpLeftClick = g.dpad_left;
        isTriangleClick = g.triangle;
        isCircleClick = g.circle;

        circle = g.circle;
        triangle = g.triangle;
        cross = g.cross;
        square = g.square;

        home = g.ps;

        dp_up = g.dpad_up;
        dp_down = g.dpad_down;
        dp_right = g.dpad_right;
        dp_left = g.dpad_left;

        rTrig = g.right_trigger > 0;
    }
}
