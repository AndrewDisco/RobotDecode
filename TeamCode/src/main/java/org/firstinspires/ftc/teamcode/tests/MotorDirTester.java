package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Motor Test", group = "test")
public class MotorDirTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        DcMotor[] motors = new DcMotor[4];
        String[] names = {"flMotor", "frMotor", "blMotor", "brMotor"};
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, names[i]);
        }

        int selected = 0;
        boolean motorOn = false;
        boolean dpadUpPrev = false, dpadDownPrev = false, aPrev = false;

        telemetry.addLine("Use D-pad up/down to select motor. Press A to toggle power.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !dpadUpPrev) {
                selected = (selected + 1) % 4;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                selected = (selected + 3) % 4;
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            if (gamepad1.a && !aPrev) {
                motorOn = !motorOn;
            }
            aPrev = gamepad1.a;

            for (int i = 0; i < 4; i++) {
                motors[i].setPower((i == selected && motorOn) ? 0.5 : 0.0);
            }

            telemetry.addData("Selected Motor", names[selected]);
            telemetry.addData("Power", motorOn ? 0.5 : 0.0);
            telemetry.update();

            sleep(50);
        }

        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }
}