package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "Spindexer Test", group = "test")
public class SpindexerTest extends OpMode {

    private Spindexer spindexer;
    private Intake intake; // Intake also needs spindexer now

    private boolean previousCrossState = false;
    private boolean intakeActive = false;

    // Toggle for Outtake Spin
    private boolean previousTriangleState = false;
    private boolean outtakeActive = false;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();



    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, spindexer);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instruct", "Cross: Intake (CW), Triangle: Outtake (CCW)");
    }

    @Override
    public void loop() {
        // Intake toggle logic (CW)
        boolean currentCrossState = gamepad1.cross;
        if (currentCrossState && !previousCrossState) {
            intakeActive = !intakeActive;
            if (intakeActive) {
                intake.intake(); // This spins spindexer CW via intake
                outtakeActive = false; // Priority logic or just separate?
            } else {
                intake.stop();
            }
        }
        previousCrossState = currentCrossState;

        // Outtake toggle logic (CCW)
        boolean currentTriangleState = gamepad1.triangle;
        if (currentTriangleState && !previousTriangleState) {
            outtakeActive = !outtakeActive;
            if (outtakeActive) {
                spindexer.spinOuttake();
                intakeActive = false; // Override intake
                intake.stop(); // Stop intake motor too
            } else {
                spindexer.stop();
            }
        }
        previousTriangleState = currentTriangleState;

        if (gamepad1.dpad_up) {
            spindexer.spinNextIntakePosition();
            intakeActive = false;
            outtakeActive = false;
        }
        if (gamepad1.dpad_right) {
            spindexer.spinNextOuttakePosition();
            intakeActive = false;
            outtakeActive = false;
        }
        if (gamepad1.dpad_down) {
            spindexer.stop();
            intakeActive = false;
            outtakeActive = false;
        }

        panelsTelemetry.addData("unghi", spindexer.getOutputAngle());
        panelsTelemetry.addData("target", spindexer.getTargetAngle());
        panelsTelemetry.addData("eroare", spindexer.getOutputAngle()-spindexer.getTargetAngle());
        panelsTelemetry.update();

        spindexer.update();
        intake.update(); // Just in case

        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Outtake Active", outtakeActive);
        telemetry.addData("Angle 1 (Deg)", spindexer.getOutputAngle());
        telemetry.update();
    }
}
