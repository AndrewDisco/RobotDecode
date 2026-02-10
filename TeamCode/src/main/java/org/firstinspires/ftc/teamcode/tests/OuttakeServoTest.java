package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Outtake Servo Test", group = "test")
public class OuttakeServoTest extends LinearOpMode {

    public static double elevationServoPosition = 0.5;

    private Servo elevationServo;

    @Override
    public void runOpMode() throws InterruptedException {
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");

        telemetry.addLine("Outtake Servo Test Ready");
        telemetry.addLine("Adjust elevationServoPosition from dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            elevationServo.setPosition(elevationServoPosition);

            telemetry.addData("Target Position", elevationServoPosition);
            telemetry.addData("Servo Position", elevationServo.getPosition());
            telemetry.update();
        }
    }
}
