package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.StickyGamepad;

import java.util.LinkedList;
import java.util.List;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

public abstract class SwerveTeleOpBase extends OpMode {

    protected Follower follower;
    protected Outtake outtake;
    protected Intake intake;
    protected Spindexer spindexer;
    protected Turret turret;
    protected Lift lift;
    protected RapidFireCommand rapidFireCommand;

    protected StickyGamepad stickyGamepad1;
    protected StickyGamepad stickyGamepad2;

    // Toggle States
    protected boolean intakeActive = false;
    protected boolean isAutoMode = true; // DEFAULT: Start in Auto (Sensor) Mode
    protected boolean autoShootActive = false;
    protected boolean wasSortingTriggered = false;

    protected TelemetryManager panelsTelemetry;
    protected FieldManager field;
    protected LinkedList<Pose> poseHistory = new LinkedList<>();
    protected static final int TRAIL_LENGTH = 50;

    // Abstract methods for alliance-specific configuration
    protected abstract Pose getStartPose();

    protected abstract Pose getResetPose();

    protected abstract boolean isRedAlliance();

    protected List<LynxModule> allHubs;
    
    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        SwerveConstants.servoCachingThreshold = 0.1;

        follower = Constants.createFollower(hardwareMap);
        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            follower.setStartingPose(getStartPose());
        }
        follower.startTeleopDrive();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        outtake = new Outtake(hardwareMap);
        spindexer = new Spindexer(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, spindexer);
        turret = new Turret(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        rapidFireCommand = new RapidFireCommand(outtake, spindexer, intake);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    @Override
    public void start() {
        setMotorsToFloat(); // Turn off braking in teleop
    }

    public void setMotorsToFloat() {
        if (Constants.swerveDrive != null) {
            Constants.swerveDrive.startTeleopDrive(false);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        stickyGamepad1.update();
        stickyGamepad2.update();

        updateLift(); // Update lift first for accurate position check

        if (lift.getPosition() > 1000) {
            // Safety Cut-off: Turn off other subsystems
            intake.stop();
            intakeActive = false;

            outtake.stop();
            autoShootActive = false;

            spindexer.stop();
            if (rapidFireCommand.isActive()) {
                rapidFireCommand.stop();
            }

            turret.stop();

            // Stop Swerve Drive
            follower.setTeleOpDrive(0, 0, 0, false);
            follower.update();
        } else {
            updateDrive();
            updateTurret();
            updateShooter();
            updateIntake();
            updateSpindexer();
            updateRapidFire();
        }
        
        updateOtherControls();

        drawField();
        updateTelemetry();
    }

    private void drawField() {
        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            poseHistory.add(currentPose);
            if (poseHistory.size() > TRAIL_LENGTH) {
                poseHistory.removeFirst();
            }

            // Draw Trail with fading opacity
            for (int i = 0; i < poseHistory.size() - 1; i++) {
                Pose p1 = poseHistory.get(i);
                Pose p2 = poseHistory.get(i + 1);

                // Calculate opacity: 0.0 (oldest) to 1.0 (newest)
                double progress = (double) i / (poseHistory.size() - 1);
                // Simple alpha mapping. Hex alpha is 00 to FF.
                int alpha = (int) (progress * 255);
                String alphaHex = String.format("%02X", alpha);
                String color = "#4CAF50" + alphaHex; // Green trail

                field.setStyle(new Style("", color, 2.0)); // Fill, Stroke, StrokeWidth
                field.moveCursor(p1.getX(), p1.getY());
                field.line(p2.getX(), p2.getY());
            }

            // Draw Robot
            field.setStyle(new Style("", "#3F51B5", 2.0)); // Blue Stroke
            field.moveCursor(currentPose.getX(), currentPose.getY());
            field.circle(9.0); // Robot Radius ~9 inches

            // Draw Heading Indicator
            double headingLen = 12.0;
            double x2 = currentPose.getX() + headingLen * Math.cos(currentPose.getHeading());
            double y2 = currentPose.getY() + headingLen * Math.sin(currentPose.getHeading());
            field.line(x2, y2);
        }
        field.update();
    }

    private void updateDrive() {
        follower.update();
        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            if (Constants.swerveDrive != null) {
                Constants.swerveDrive.xLock();
            }
        } else {
            if (isRedAlliance()) {
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            } else {
                // Blue Alliance uses (y, x) for translation to invert the field-centric forward direction
                follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            }
        }
    }

    protected Turret.TurretMode turretMode = Turret.TurretMode.POSE_LOCK;

    private void updateTurret() {
        // Toggle modes: POSE_LOCK ↔ MANUAL
        if (stickyGamepad2.dpad_up_just_pressed()) {
            turretMode = (turretMode == Turret.TurretMode.POSE_LOCK)
                    ? Turret.TurretMode.MANUAL
                    : Turret.TurretMode.POSE_LOCK;
        }

        turret.setMode(turretMode);
        
        // Handle manual offset tuning
        if (stickyGamepad2.dpad_left_just_pressed()) {
            turret.addManualOffset(-Turret.MANUAL_OFFSET_STEP);
        } else if (stickyGamepad2.dpad_right_just_pressed()) {
            turret.addManualOffset(Turret.MANUAL_OFFSET_STEP);
        }

        turret.update(follower.getPose(), isRedAlliance(), gamepad2.right_stick_x);
    }

    private void updateShooter() {
        // ------------------ SHOOTER TOGGLE LOGIC (LEFT BUMPER) ------------------
        if (stickyGamepad1.left_bumper_just_pressed() || stickyGamepad2.circle_just_pressed()) {
            autoShootActive = !autoShootActive;
        }

        if (autoShootActive) {
            gamepad1.rumble(100);
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                double distanceToTarget = turret.getOdometryDistance();

                outtake.updateShooterFromDistance(distanceToTarget);
            }
        } else if (gamepad1.share) {
            outtake.reverse();
            outtake.setElevation(0.0);
        } else {
            outtake.stop();
        }
        outtake.update();

    }

    private void updateIntake() {
        // ------------------ INTAKE LOGIC (RIGHT BUMPER) ------------------
        if (stickyGamepad1.right_bumper_just_pressed() || stickyGamepad2.cross_just_pressed()) {
            intakeActive = !intakeActive;
            if (intakeActive) {
                intake.intakeOnlyMotors();
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

        // ------------------ INTAKE OUT LOGIC (Dpad Down GP1) ------------------
        if (stickyGamepad1.dpad_down_just_pressed()) {
            intakeActive = !intakeActive;
            if (intakeActive) {
                intake.intakeOut();
            } else {
                intake.stop();
            }
        }

        intake.update();

        // Vibrate gamepad 2 when intake is active
        if (intakeActive) {
            gamepad2.rumble(100);
        }
    }

    private void updateSpindexer() {
        // ------------------ SPINDEXER MODE TOGGLE (DPAD RIGHT) ------------------
        if (stickyGamepad1.dpad_right_just_pressed()) {
            isAutoMode = !isAutoMode;

            if (intakeActive) {
                if (isAutoMode) {
                    spindexer.spinIntakeCycle();
                } else {
                    spindexer.spinIntake();
                }
            }
        }

        // ------------------ SPINDEXER MANUAL STEP (Right Bumper GP2 / Dpad Up GP1)
        // ------------------
        if (stickyGamepad2.right_bumper_just_pressed() ||
                (stickyGamepad1.dpad_up_just_pressed() && !isAutoMode)) {
            spindexer.spinNextIntakePosition();
        }

        // --------------- SPINDEXER OUTTAKE STEP (Left Bumper GP2) ------------------
        if (stickyGamepad2.left_bumper_just_pressed()) {
            spindexer.spinNextOuttakePosition();
        }

        // ------------------ MANUAL CYCLE TOGGLE (Square GP2) ------------------
        if (stickyGamepad2.square_just_pressed()) {
            spindexer.setAutoCycleEnabled(!spindexer.isAutoCycleEnabled());
        }

        spindexer.update();
    }

    private void updateRapidFire() {
        // ------------------ RAPID FIRE LOGIC ------------------
        if (stickyGamepad1.triangle_just_pressed() || stickyGamepad2.triangle_just_pressed()) {
            if (!rapidFireCommand.isActive()) {
                rapidFireCommand.start();
                intakeActive = false;
                intake.stop();
            } else {
                rapidFireCommand.stop();
            }
        }

        rapidFireCommand.update();
    }

    // ------------------ LIFT LOGIC (TRIGGERS GP2) ------------------
    private void updateLift() {
        // Right Trigger = Extend (Positive Power with REVERSE direction)
        // Left Trigger = Retract (Negative Power)

        double extendPower = gamepad2.right_trigger;
        double retractPower = gamepad2.left_trigger;
        
        // Sorting trigger: both triggers > 0.5
        if (extendPower > 0.5 && retractPower > 0.5) {
             if (!wasSortingTriggered) {
                 Limelight.Motif currentMotif = turret.getLimelight().getCurrentMotif();
                 spindexer.sortBalls(currentMotif);
                 wasSortingTriggered = true;
             }
             lift.setPower(0); // Stop lift while triggering sort
        } else {
             wasSortingTriggered = false;
             
             if (extendPower > 0.05) {
                 lift.setPower(extendPower);
             } else if (retractPower > 0.05) {
                 lift.setPower(-retractPower);
             } else {
                 // Stop / Hold
                 lift.setPower(0);
             }
        }

        lift.update();
    }

    private void updateOtherControls() {
        // ------------------ POSE RESET LOGIC ------------------
        if (stickyGamepad2.share_just_pressed()) {
            follower.setPose(getResetPose());
        }
    }

    private void updateTelemetry() {
        telemetry.addData("AutoShoot Mode", autoShootActive ? "ON" : "OFF");
        telemetry.addData("Intake", intakeActive ? "ON" : "OFF");
        telemetry.addData("Spindexer Mode", isAutoMode ? "AUTO (SENSOR)" : "MANUAL (HOLD)");
        telemetry.addData("Alliance", isRedAlliance() ? "RED" : "BLUE");

        telemetry.addData("Turret Mode", turretMode);
        telemetry.addData("Target Distance (Odo)", "%.2f", turret.getOdometryDistance());

        telemetry.addData("spindexer/targetAngle", spindexer.getTargetAngle());
        telemetry.addData("spindexer/currentAngle", spindexer.getOutputAngle());


        panelsTelemetry.update();
        telemetry.update();
    }
}