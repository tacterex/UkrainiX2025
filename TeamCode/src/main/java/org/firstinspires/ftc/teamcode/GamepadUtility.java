package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadUtility {
    public double ly, lx, rx;
    public boolean rbum_click;

    public void update(Gamepad gamepad){
        ly = gamepad.left_stick_y;
        lx = gamepad.left_stick_x;
        rx = gamepad.right_stick_x;

        rbum_click = gamepad.right_bumper && !rbum_click;
    }
}
