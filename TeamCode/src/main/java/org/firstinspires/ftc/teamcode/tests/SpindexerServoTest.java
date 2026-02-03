package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.StickyGamepad;

@TeleOp(name = "Spindexer Servo Test", group = "test")
public class SpindexerServoTest extends OpMode {

    private CRServo spindexer1;
    private CRServo spindexer2;

    private StickyGamepad stickyGamepad1;

    // 1 for spindexer1, 2 for spindexer2
    private int selectedServo = 1;

    @Override
    public void init() {
        spindexer1 = hardwareMap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwareMap.get(CRServo.class, "spindexer2");

        // Assuming both need reverse direction like in Spindexer.java, or forward for testing
        spindexer1.setDirection(CRServo.Direction.REVERSE);
        spindexer2.setDirection(CRServo.Direction.REVERSE);

        stickyGamepad1 = new StickyGamepad(gamepad1);

        telemetry.addData("Selected Servo", "spindexer1 (Press D-PAD RIGHT to toggle)");
        telemetry.update();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();

        // Toggle selected servo
        if (stickyGamepad1.dpad_right_just_pressed()) {
            if (selectedServo == 1) {
                selectedServo = 2;
            } else {
                selectedServo = 1;
            }
        }

        // Get power from right trigger (0.0 to 1.0)
        double triggerPower = gamepad1.right_trigger;

        // Apply power to selected servo, stop the other one
        if (selectedServo == 1) {
            spindexer1.setPower(triggerPower);
            spindexer2.setPower(0.0);
            telemetry.addData("Selected Servo", "spindexer1 (1)");
            telemetry.addData("Output Power 1", triggerPower);
            telemetry.addData("Output Power 2", 0.0);
        } else {
            spindexer1.setPower(0.0);
            spindexer2.setPower(triggerPower);
            telemetry.addData("Selected Servo", "spindexer2 (2)");
            telemetry.addData("Output Power 1", 0.0);
            telemetry.addData("Output Power 2", triggerPower);
        }

        telemetry.addData("Instructions", "D-PAD RIGHT: toggle servo | RIGHT TRIGGER: spin");
        telemetry.update();
    }
}
