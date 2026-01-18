package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StickyGamepad {
    private final Gamepad gamepad;

    private boolean a, b, x, y;
    private boolean dpad_up, dpad_down, dpad_left, dpad_right;
    private boolean left_bumper, right_bumper;
    private boolean start, back, guide;
    private boolean left_stick_button, right_stick_button;

    private boolean prev_a, prev_b, prev_x, prev_y;
    private boolean prev_dpad_up, prev_dpad_down, prev_dpad_left, prev_dpad_right;
    private boolean prev_left_bumper, prev_right_bumper;
    private boolean prev_start, prev_back, prev_guide;
    private boolean prev_left_stick_button, prev_right_stick_button;

    public StickyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        prev_a = a;
        prev_b = b;
        prev_x = x;
        prev_y = y;
        prev_dpad_up = dpad_up;
        prev_dpad_down = dpad_down;
        prev_dpad_left = dpad_left;
        prev_dpad_right = dpad_right;
        prev_left_bumper = left_bumper;
        prev_right_bumper = right_bumper;
        prev_start = start;
        prev_back = back;
        prev_guide = guide;
        prev_left_stick_button = left_stick_button;
        prev_right_stick_button = right_stick_button;

        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        start = gamepad.start;
        back = gamepad.back;
        guide = gamepad.guide;
        left_stick_button = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button;
    }

    // Current State
    public boolean a() {
        return a;
    }

    public boolean b() {
        return b;
    }

    public boolean x() {
        return x;
    }

    public boolean y() {
        return y;
    }

    public boolean dpad_up() {
        return dpad_up;
    }

    public boolean dpad_down() {
        return dpad_down;
    }

    public boolean dpad_left() {
        return dpad_left;
    }

    public boolean dpad_right() {
        return dpad_right;
    }

    public boolean left_bumper() {
        return left_bumper;
    }

    public boolean right_bumper() {
        return right_bumper;
    }

    public boolean start() {
        return start;
    }

    public boolean back() {
        return back;
    }

    public boolean guide() {
        return guide;
    }

    public boolean left_stick_button() {
        return left_stick_button;
    }

    public boolean right_stick_button() {
        return right_stick_button;
    }

    // PS4/PS5 aliases
    public boolean cross() {
        return a;
    }

    public boolean circle() {
        return b;
    }

    public boolean square() {
        return x;
    }

    public boolean triangle() {
        return y;
    }

    public boolean share() {
        return back;
    }

    public boolean options() {
        return start;
    }

    public boolean ps() {
        return guide;
    }

    // Just Pressed (Rising Edge)
    public boolean a_just_pressed() {
        return a && !prev_a;
    }

    public boolean b_just_pressed() {
        return b && !prev_b;
    }

    public boolean x_just_pressed() {
        return x && !prev_x;
    }

    public boolean y_just_pressed() {
        return y && !prev_y;
    }

    public boolean dpad_up_just_pressed() {
        return dpad_up && !prev_dpad_up;
    }

    public boolean dpad_down_just_pressed() {
        return dpad_down && !prev_dpad_down;
    }

    public boolean dpad_left_just_pressed() {
        return dpad_left && !prev_dpad_left;
    }

    public boolean dpad_right_just_pressed() {
        return dpad_right && !prev_dpad_right;
    }

    public boolean left_bumper_just_pressed() {
        return left_bumper && !prev_left_bumper;
    }

    public boolean right_bumper_just_pressed() {
        return right_bumper && !prev_right_bumper;
    }

    public boolean start_just_pressed() {
        return start && !prev_start;
    }

    public boolean back_just_pressed() {
        return back && !prev_back;
    }

    public boolean guide_just_pressed() {
        return guide && !prev_guide;
    }

    public boolean left_stick_button_just_pressed() {
        return left_stick_button && !prev_left_stick_button;
    }

    public boolean right_stick_button_just_pressed() {
        return right_stick_button && !prev_right_stick_button;
    }

    // PS Aliases for Just Pressed
    public boolean cross_just_pressed() {
        return a_just_pressed();
    }

    public boolean circle_just_pressed() {
        return b_just_pressed();
    }

    public boolean square_just_pressed() {
        return x_just_pressed();
    }

    public boolean triangle_just_pressed() {
        return y_just_pressed();
    }

    public boolean share_just_pressed() {
        return back_just_pressed();
    }

    public boolean options_just_pressed() {
        return start_just_pressed();
    }

    public boolean ps_just_pressed() {
        return guide_just_pressed();
    }
}
