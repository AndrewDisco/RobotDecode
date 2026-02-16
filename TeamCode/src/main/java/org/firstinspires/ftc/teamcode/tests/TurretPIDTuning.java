package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
@TeleOp(name = "Turret PID Tuning", group = "test")
public class TurretPIDTuning extends LinearOpMode {

    public static double TARGET_ANGLE = 0.0;

    // Limits verify
    public static double TEST_MAX_ANGLE = 90;

    // Toggle for manual power override (to test direction)
    public static boolean MANUAL_OVERRIDE = true;
    public static double MANUAL_POWER = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap, telemetry);
        turret.initialize();

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Turret PID Tuning Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Safety clamp for testing
            TARGET_ANGLE = Range.clip(TARGET_ANGLE, -TEST_MAX_ANGLE, TEST_MAX_ANGLE);

            if (MANUAL_OVERRIDE) {
                // Manual PID Tuning Mode
                // User sets TARGET_ANGLE in Dashboard, Turret tries to reach it.
                turret.loopAuto(TARGET_ANGLE);
            } else {
                // Disabled Mode - Safety
                turret.stop();
            }

            // telemetry.update() is handled by Turret generally, but we want verify.
            telemetry.addData("Target (Dash)", TARGET_ANGLE);
            telemetry.addData("Actual Angle", turret.getCurrentAngle());


            telemetry.update();

        }
    }
}
