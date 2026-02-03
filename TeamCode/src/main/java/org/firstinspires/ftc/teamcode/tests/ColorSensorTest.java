package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "test")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "culoare1");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // telemetry.addData("Red", colorSensor.red());
            // telemetry.addData("Green", colorSensor.green());
            // telemetry.addData("Blue", colorSensor.blue());
            // telemetry.addData("Alpha", colorSensor.alpha());
            // telemetry.addData("ARGB", colorSensor.argb());
            // telemetry.update();
        }
    }
}
