package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class Turret {
    private final Servo azimuthServo;
    private final AnalogInput azimuthEncoder;
    private final Servo elevationServo;
    private final Limelight limelight;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // --- Encoder Configuration ---
    // 1.77V = physical center of turret (used only for READING the encoder)
    public static double ENCODER_CENTER_VOLTAGE = 1.77;
    // Servo center is 0.5 (used for COMMANDING the servo)
    public static double SERVO_CENTER_POSITION = 0.5;
    // Axon MAX+ servo: 0-320° for position 0.0-1.0
    public static double SERVO_RANGE_DEG = 320.0;
    // Gear ratio: 85-tooth turret gear / 50-tooth servo gear
    public static double GEAR_RATIO = 85.0 / 50.0;

    // --- Limits ---
    public static double MAX_ANGLE = 100.0;
    public static double MIN_ANGLE = -100.0;

    // --- Backboard Compensation (POSE_LOCK only) ---
    public static double BACKBOARD_GAIN_LEFT_RED = 0.22;
    public static double BACKBOARD_GAIN_RIGHT_RED = 0.22;
    public static double BACKBOARD_GAIN_LEFT_BLUE = 0.20;
    public static double BACKBOARD_GAIN_RIGHT_BLUE = 0.07;

    // --- Turret Physical Offset ---
    // Turret center of rotation is this far AHEAD of robot center of rotation (inches)
    public static double TURRET_FORWARD_OFFSET = 1.77165;

    // --- Manual Mode ---
    // Full stick deflection maps to ±MANUAL_RANGE degrees (tune for comfort)
    public static double MANUAL_RANGE = 30.0;
    public static double JOYSTICK_DEADBAND = 0.05;

    // --- Vision Fine Correction (applied on top of POSE_LOCK) ---
    // Each frame, visionOffset accumulates a small fraction of the limelight error.
    // Lower = slower/smoother correction, higher = faster but may oscillate.
    public static double VISION_CORRECTION_GAIN = 0.05;
    // How fast the offset decays back to 0 when the target is lost (0.0 to 1.0)
    public static double VISION_DECAY_RATE = 0.95;

    // --- Manual Tuning Offset ---
    public static double MANUAL_OFFSET_STEP = 1; // Degrees to adjust per dpad click
    private double userManualOffset = 0.0;

    // --- Control Source ---
    public static boolean useAnalogEncoder = true;

    // --- State ---
    public enum TurretMode {
        POSE_LOCK,   // Odometry targeting + limelight fine correction
        MANUAL       // Joystick control
    }

    private TurretMode currentMode = TurretMode.POSE_LOCK;

    private double currentTurretAngle = 0.0;
    public static double targetAngle = 0.0;
    private double lastDistToGoal = 0.0;

    // Persistent vision correction offset — accumulates slowly over multiple frames
    private double visionOffset = 0.0;

    // Scoring Positions
    private static final Pose BLUE_SCORING_POS = new Pose(0, 144, 0);
    private static final Pose RED_SCORING_POS = new Pose(144, 144, 0);

    private Telemetry telemetry;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        azimuthServo = hardwareMap.get(Servo.class, "azimuthServo");
        azimuthEncoder = hardwareMap.get(AnalogInput.class, "azimuthEncoder");
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");

        azimuthServo.setDirection(Servo.Direction.REVERSE);

        limelight = new Limelight(hardwareMap, telemetry);
    }

    public void initialize() {
        // Init logic if needed
    }

    // =====================================================================
    //  Main Update — called every loop cycle
    // =====================================================================

    /**
     * @param robotPose     Current robot pose (PedroPathing Pose)
     * @param isRedAlliance Alliance color
     * @param manualInput   gamepad2.right_stick_x (only used in MANUAL mode)
     */
    public void update(Pose robotPose, boolean isRedAlliance, double manualInput) {
        // 1. Read current turret angle from encoder
        if (useAnalogEncoder) {
            currentTurretAngle = calculateAngleFromVoltage();
        } else {
            currentTurretAngle = targetAngle;
        }

        // 2. Update limelight (always, so data is fresh if we switch modes)
        limelight.update(isRedAlliance);

        // 3. Calculate target angle based on active mode
        switch (currentMode) {
            case POSE_LOCK:
                calculatePoseLock(robotPose, isRedAlliance);
                break;
            case MANUAL:
                calculateManual(manualInput);
                break;
        }

        // 4. Drive servo
        updateServoPosition();
    }

    // =====================================================================
    //  POSE_LOCK: odometry targeting + limelight fine correction
    // =====================================================================
    private void calculatePoseLock(Pose robotPose, boolean isRedAlliance) {
        if (robotPose == null) return;

        // Compute turret's actual field position (offset from robot center)
        double heading = robotPose.getHeading();
        double turretX = robotPose.getX() + TURRET_FORWARD_OFFSET * Math.cos(heading);
        double turretY = robotPose.getY() + TURRET_FORWARD_OFFSET * Math.sin(heading);

        Pose targetPose = isRedAlliance ? RED_SCORING_POS : BLUE_SCORING_POS;
        double dx = targetPose.getX() - turretX;
        double dy = targetPose.getY() - turretY;

        lastDistToGoal = Math.hypot(dx, dy);

        // Field-space angle to target (from turret position)
        double absAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(heading);
        double relativeAngle = absAngleToTarget - robotHeadingDeg;

        // Normalize to -180..180
        while (relativeAngle > 180)  relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        // --- Backboard approach-angle compensation ---
        double approachAngle = Math.atan2(dy, dx);
        double straightShotAngle = isRedAlliance ? Math.PI / 4 : 3 * Math.PI / 4;
        double deviation = approachAngle - straightShotAngle;
        double gain;
        if (isRedAlliance) {
            gain = (deviation < 0) ? BACKBOARD_GAIN_LEFT_RED : BACKBOARD_GAIN_RIGHT_RED;
        } else {
            gain = (deviation < 0) ? BACKBOARD_GAIN_LEFT_BLUE : BACKBOARD_GAIN_RIGHT_BLUE;
        }
        double backboardOffset = Math.toDegrees(deviation) * gain;

        double odoAngle = relativeAngle + backboardOffset;

        // --- Limelight fine correction ---
        // Slowly accumulate a persistent offset based on limelight error.
        // This smoothly corrects odometry drift without causing oscillation.
        if (limelight.isTargetSeen()) {
            double resultTx = limelight.getTx();
            // Each frame, nudge the offset by a tiny fraction of the error
            visionOffset += -resultTx * VISION_CORRECTION_GAIN;
        } else {
            // Smoothly decay the stale offset back to 0 when the target is lost
            visionOffset *= VISION_DECAY_RATE;
            
            // Snap to 0 if the offset is negligible to avoid micro-corrections
            if (Math.abs(visionOffset) < 0.1) {
                visionOffset = 0.0;
            }
        }

        targetAngle = odoAngle + visionOffset + userManualOffset;
    }

    // =====================================================================
    //  MANUAL: direct proportional joystick mapping
    //  Stick center (0) = turret center (0°), full left/right = ±MANUAL_RANGE
    // =====================================================================
    private void calculateManual(double manualInput) {
        if (Math.abs(manualInput) > JOYSTICK_DEADBAND) {
            targetAngle = -manualInput * MANUAL_RANGE;
        } else {
            targetAngle = 0.0;
        }
    }

    // =====================================================================
    //  Servo output
    // =====================================================================
    private void updateServoPosition() {
        double clampedTarget = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);

        double targetServoDegrees = clampedTarget * GEAR_RATIO;
        // Axon MAX+ maps 0.0-1.0 to 0-355°, not 0-360°
        double positionOffset = targetServoDegrees / SERVO_RANGE_DEG;

        double targetPosition = Range.clip(SERVO_CENTER_POSITION + positionOffset, 0.0, 1.0);
        azimuthServo.setPosition(targetPosition);

        if (telemetry != null) {
            telemetry.addData("Turret/Mode", currentMode);
            telemetry.addData("Turret/Target", clampedTarget);
            telemetry.addData("Turret/Current", currentTurretAngle);
            telemetry.addData("Turret/ServoPos", targetPosition);
            telemetry.addData("Turret/ResultTx", limelight.getResultTx());
        }

        panelsTelemetry.addData("turret/targetAngle", clampedTarget);
        panelsTelemetry.addData("turret/currentAngle", currentTurretAngle);
        panelsTelemetry.update();
    }

    // =====================================================================
    //  Encoder reading
    // =====================================================================
    private double calculateAngleFromVoltage() {
        double voltage = azimuthEncoder.getVoltage();
        double diffVoltage = voltage - ENCODER_CENTER_VOLTAGE;
        // Encoder maps 0-3.3V to 0-355° (Axon MAX+ range)
        double servoDegrees = (diffVoltage / 3.3) * SERVO_RANGE_DEG;
        return servoDegrees / GEAR_RATIO;
    }

    // =====================================================================
    //  Public API
    // =====================================================================

    public void setMode(TurretMode mode) {
        this.currentMode = mode;
    }

    public TurretMode getMode() {
        return currentMode;
    }

    public void addManualOffset(double offsetAmount) {
        this.userManualOffset += offsetAmount;
    }

    public void setElevation(double pos) {
        elevationServo.setPosition(pos);
    }

    public void stop() {
        limelight.stop();
    }

    public double getCurrentAngle() {
        return currentTurretAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getOdometryDistance() {
        return lastDistToGoal;
    }

    public Limelight getLimelight() {
        return limelight;
    }

    // --- Compatibility helpers ---

    /** Convenience for callers that set mode externally (auto opmodes, teleop). */
    public void loop(TurretMode mode, Pose robotPose, boolean isRedAlliance) {
        this.currentMode = mode;
        update(robotPose, isRedAlliance, 0.0);
    }

    /** Direct angle command for test opmodes. */
    public void loopAuto(double target) {
        this.currentMode = TurretMode.MANUAL;
        targetAngle = target;
        currentTurretAngle = calculateAngleFromVoltage();
        updateServoPosition();
    }
}