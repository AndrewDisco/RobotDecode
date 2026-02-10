package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.toRadians;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.utils.StickyGamepad;

import java.util.List;

@TeleOp(name = "Shooter Test", group = "test")
@Configurable
public class ShooterTester extends LinearOpMode {

    public static boolean enabled = false;

    // Target velocity in ticks per second (try 1500-2500 for flywheels)
    public static double targetVelocity = 0.0;

    // Max velocity for display purposes
    public static double MAX_VELOCITY = 2800.0;

    // Elevation Servo
    public static double elevationServoPos = 0.5;

    private Outtake outtake;
    private Intake intake;
    private Spindexer spindexer;
    private Turret turret;
    private RapidFireCommand rapidFireCommand;
    private StickyGamepad stickyGamepad1;

    // State for intake
    private boolean intakeActive = false;
    private boolean isAutoMode = true; // Default to Auto (Sensor) Mode

    // Pedro Pathing & Target Logic
    private Follower follower;
    private static final Pose START_POSE = new Pose(93.500, 35.000, toRadians(90));
    private static final Pose BLUE_TARGET = new Pose(13.07392, 133.72762, toRadians(90));
    private static final Pose RED_TARGET = new Pose(130.7392, 133.72762, toRadians(90));

    private boolean isBlueTarget = true;
    private boolean previousXState = false;
    private boolean feedActive = false;
    private boolean previousTriangleState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize Follower and Subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        spindexer = new Spindexer(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap, spindexer);
        rapidFireCommand = new RapidFireCommand(outtake, spindexer, intake);
        stickyGamepad1 = new StickyGamepad(gamepad1);

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            stickyGamepad1.update(); // Update sticky gamepad

            follower.update();
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            Pose currentPose = follower.getPose();

            // Target Toggle Logic (Gamepad1 X)
            boolean currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isBlueTarget = !isBlueTarget;
            }
            previousXState = currentXState;

            // --- INTAKE LOGIC (Right Bumper) ---
            if (stickyGamepad1.right_bumper_just_pressed()) {
                intakeActive = !intakeActive;
                if (intakeActive) {
                    intake.intakeOnlyMotors();
                    // Check Mode: Auto (Cycle) or Manual (Hold)
                    if (isAutoMode) {
                        spindexer.spinIntakeCycle();
                    } else {
                        spindexer.spinIntake();
                    }
                } else {
                    intake.stop();
                    spindexer.stop();
                }
            }

            // --- SPINDEXER MODE TOGGLE (DPAD RIGHT) ---
            if (stickyGamepad1.dpad_right_just_pressed()) {
                isAutoMode = !isAutoMode;
                // If intake is currently running, apply the new mode immediately
                if (intakeActive) {
                    if (isAutoMode) {
                        spindexer.spinIntakeCycle();
                    } else {
                        spindexer.spinIntake();
                    }
                }
            }

            // RapidFire Logic (Gamepad1 Triangle)
            boolean currentTriangleState = gamepad1.triangle;
            if (currentTriangleState && !previousTriangleState) {
                if (rapidFireCommand.isActive()) {
                    rapidFireCommand.stop();
                    intakeActive = false;
                    intake.stop();
                } else {
                    rapidFireCommand.start();
                    intakeActive = false;
                    intake.stop();
                }
            }
            previousTriangleState = currentTriangleState;

            // Update Subsystems
            spindexer.update();
            Pose currentVelocity = follower.poseTracker.getLocalizer().getVelocity();
            double targetFlywheel = outtake.getTargetVelocity();
            
            turret.loop(Turret.TurretMode.POSE_LOCK, currentPose, !isBlueTarget);

            if (enabled) {
                double odoDistance = turret.getOdometryDistance();
                outtake.updateShooterFromDistance(odoDistance);
            } else {
                outtake.setElevation(elevationServoPos);
                outtake.setVelocity(targetVelocity);
            }

            outtake.update();
            intake.update();
            rapidFireCommand.update();

            // Get current velocity from Outtake
            double flywheelVelocity = outtake.getLansator1Velocity();

            // Telemetry
            if (currentPose != null) {
                telemetry.addData("Active Target", isBlueTarget ? "BLUE" : "RED");
                telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, H: %.2f", currentPose.getX(), currentPose.getY(),
                        Math.toDegrees(currentPose.getHeading()));
            } else {
                telemetry.addData("Active Target", isBlueTarget ? "BLUE" : "RED");
                telemetry.addData("Current Pose", "NULL");
            }

            telemetry.addData("Enabled", enabled);
            telemetry.addData("Intake", intakeActive ? "ON" : "OFF");
            telemetry.addData("Spindexer Mode", isAutoMode ? "AUTO (SENSOR)" : "MANUAL (HOLD)");
            telemetry.addData("Rapid Fire", rapidFireCommand.isActive() ? "ACTIVE" : "IDLE");
            telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", flywheelVelocity);
            telemetry.addData("Velocity Error", "%.0f", targetVelocity - flywheelVelocity);
            telemetry.addData("Elevation Servo", "%.2f", elevationServoPos);

            telemetry.addData("Target Distance (Odo)", "%.2f", turret.getOdometryDistance());
            telemetry.addData("Limelight Distance", "%.2f", turret.getLimelight().getDistanceToTarget());

            telemetry.update();
        }

        // Stop subsystems when OpMode ends
        outtake.stop();
        intake.stop();
        spindexer.stop();
        turret.stop();
        if (rapidFireCommand.isActive()) {
            rapidFireCommand.stop();
        }
    }
}