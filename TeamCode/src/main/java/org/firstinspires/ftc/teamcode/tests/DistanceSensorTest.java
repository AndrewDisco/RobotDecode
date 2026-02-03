package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Distance Sensor Test", group = "test")
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Both culoare1 and culoare2 distance sensors have been removed from configuration.
        telemetry.addData("Status", "No distance sensors configured");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            idle();
        }
    }
}
