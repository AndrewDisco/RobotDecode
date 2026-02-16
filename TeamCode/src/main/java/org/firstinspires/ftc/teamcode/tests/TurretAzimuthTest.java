package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Azimuth Servo Test", group = "test")
public class TurretAzimuthTest extends LinearOpMode {

    public static double azimuthServoPosition = 0.5;
    public static double elevationServoPosition = 0;

    private Servo azimuthServo;
    private Servo elevationServo;


    @Override
    public void runOpMode() throws InterruptedException {
        azimuthServo = hardwareMap.get(Servo.class, "azimuthServo");
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");


        telemetry.addLine("Outtake Servo Test Ready");
        telemetry.addLine("Adjust azimuthServo from dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            azimuthServo.setPosition(azimuthServoPosition);
            elevationServo.setPosition(elevationServoPosition);


            telemetry.addData("Target Position", azimuthServoPosition);
            telemetry.addData("Servo Position", azimuthServo.getPosition());

            telemetry.addData("Target elevation", elevationServo.getPosition());
            telemetry.update();
        }
    }
}
