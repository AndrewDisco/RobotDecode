package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Swerve TeleOp BLUE", group = "teleop")
public class SwerveTeleOpBlue extends SwerveTeleOpBase {

    private final Pose startPose = new Pose(48.000, 35.000, Math.toRadians(90));
    private final Pose RESET_POSE = new Pose(72.000, 72.000, Math.toRadians(90));

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected Pose getResetPose() {
        return RESET_POSE;
    }

    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}