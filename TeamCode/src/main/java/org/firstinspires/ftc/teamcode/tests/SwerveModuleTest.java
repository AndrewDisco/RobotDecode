package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveModule;

@TeleOp(name = "Swerve Module Test", group = "test")
public class SwerveModuleTest extends OpMode {

    private SwerveModule leftFrontPod;
    private SwerveModule rightFrontPod;
    private SwerveModule leftRearPod;
    private SwerveModule rightRearPod;

    private SwerveModule selectedModule;
    private String selectedModuleName;

    private SwerveConstants constants;

    private Outtake outtake;
    private Intake intake;
    private Spindexer spindexer;
    private Turret turret;
    
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        constants = new SwerveConstants();

        leftFrontPod = new SwerveModule(hardwareMap,
                constants.getLeftFrontServoName(),
                constants.getLeftFrontEncoderName(),
                constants.getLeftFrontMotorName(),
                constants.getLeftFrontTurnPID(),
                constants.getLeftFrontMotorDirection(),
                constants.getLeftFrontServoDirection(),
                constants.getLeftFrontPodAngleOffsetDeg(),
                constants.getLeftFrontPodXYOffsets(),
                constants.getLeftFrontEncoderMinVoltage(),
                constants.getLeftFrontEncoderMaxVoltage());

        rightFrontPod = new SwerveModule(hardwareMap,
                constants.getRightFrontServoName(),
                constants.getRightFrontEncoderName(),
                constants.getRightFrontMotorName(),
                constants.getRightFrontTurnPID(),
                constants.getRightFrontMotorDirection(),
                constants.getRightFrontServoDirection(),
                constants.getRightFrontPodAngleOffsetDeg(),
                constants.getRightFrontPodXYOffsets(),
                constants.getRightFrontEncoderMinVoltage(),
                constants.getRightFrontEncoderMaxVoltage());

        leftRearPod = new SwerveModule(hardwareMap,
                constants.getLeftRearServoName(),
                constants.getLeftRearEncoderName(),
                constants.getLeftRearMotorName(),
                constants.getLeftRearTurnPID(),
                constants.getLeftRearMotorDirection(),
                constants.getLeftRearServoDirection(),
                constants.getLeftRearPodAngleOffsetDeg(),
                constants.getLeftRearPodXYOffsets(),
                constants.getLeftRearEncoderMinVoltage(),
                constants.getLeftRearEncoderMaxVoltage());

        rightRearPod = new SwerveModule(hardwareMap,
                constants.getRightRearServoName(),
                constants.getRightRearEncoderName(),
                constants.getRightRearMotorName(),
                constants.getRightRearTurnPID(),
                constants.getRightRearMotorDirection(),
                constants.getRightRearServoDirection(),
                constants.getRightRearPodAngleOffsetDeg(),
                constants.getRightRearPodXYOffsets(),
                constants.getRightRearEncoderMinVoltage(),
                constants.getRightRearEncoderMaxVoltage());

        // Default to Left Front
        selectedModule = leftFrontPod;
        selectedModuleName = "Left Front";

        outtake = new Outtake(hardwareMap);
        spindexer = new Spindexer(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, spindexer);
        turret = new Turret(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Module Selection
        if (gamepad1.dpad_up) {
            selectedModule = leftFrontPod;
            selectedModuleName = "Left Front";
        } else if (gamepad1.dpad_right) {
            selectedModule = rightFrontPod;
            selectedModuleName = "Right Front";
        } else if (gamepad1.dpad_down) {
            selectedModule = rightRearPod;
            selectedModuleName = "Right Rear";
        } else if (gamepad1.dpad_left) {
            selectedModule = leftRearPod;
            selectedModuleName = "Left Rear";
        }

        // Control
        double drivePower = -gamepad1.left_stick_y;

        // Calculate target angle from right stick
        double targetAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        boolean ignoreServo = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) < 0.1;

        // Apply power to selected module using PID
        selectedModule.move(targetAngle, 0, ignoreServo, constants.getMotorCachingThreshold(),
                constants.getServoCachingThreshold());

        // Stop other modules to prevent accidents
        if (selectedModule != leftFrontPod)
            stopModule(leftFrontPod);
        if (selectedModule != rightFrontPod)
            stopModule(rightFrontPod);
        if (selectedModule != leftRearPod)
            stopModule(leftRearPod);
        if (selectedModule != rightRearPod)
            stopModule(rightRearPod);

        // Telemetry
        panelsTelemetry.addData("Selected Module", selectedModuleName);
        panelsTelemetry.addData("Drive Power", drivePower);
        panelsTelemetry.addData("Target Angle (rad)", targetAngle);
        panelsTelemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
        panelsTelemetry.addData("Encoder Voltage", selectedModule.getEncoderVoltage());
        panelsTelemetry.addData("Raw Angle", selectedModule.getRawAngleDeg());
        panelsTelemetry.addData("Offset Angle", Math.toDegrees(selectedModule.getAngleAfterOffsetRad()));
        panelsTelemetry.update();
        telemetry.update();
    }

    private void stopModule(SwerveModule module) {
        module.setMotorPower(0);
        module.setServoPower(0);
    }
}
