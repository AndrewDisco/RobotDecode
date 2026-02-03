package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Motif;

@Configurable
public class Spindexer {
    private final CRServo spindexer1;
    private final CRServo spindexer2;
    private final AnalogInput spindexer1Encoder;
    private HuskyLens huskyLens;

    public int[] slotColors = new int[3]; // 0=Unknown, 1=Green, 2=Purple

    // Constant for the Servo-to-Output ratio
    // 30T Servo -> 24T Spindexer means Output spins 1.25x faster
    private static final double GEAR_RATIO = 1; // 30.0 / 24.0;
    private static final double ANALOG_REF_V = 3.3;

    // Power constants
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = 1.0;
    public static double STOP_POWER = 0.0;

    // PID Constants
    public static double kP = 0.003;
    public static double kI = 0.00;
    public static double kD = 0.0002;
    public static double kF = 0.05;

    // Positions (Target Output Degrees)
    public static double POS_1_INTAKE = 80.0;
    public static double POS_2_INTAKE = 204.0;
    public static double POS_3_INTAKE = 320.0;

    public static double POS_1_OUTTAKE = 30.0;
    public static double POS_2_OUTTAKE = 150.0;
    public static double POS_3_OUTTAKE = 270.0;

    // Smart Intake Constants (HuskyLens-based)
    public static double HUSKY_SIZE_THRESHOLD = 50.0; // Block width or height to trigger next slot
    public static double POST_DETECTION_DELAY = 0.05; // Slightly delayed to ensure stability
    public static double DETECTION_DEBOUNCE = 0.4; // Time to ignore sensor after move

    private Telemetry telemetry;
    private boolean isPIDMode = false;
    private double targetAngle = 0.0;
    private double manualOffset = 0.0; // Added for manual tweaking
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private final ElapsedTime intakeCycleTimer = new ElapsedTime();
    private boolean autoCycleEnabled = false;

    // Sensor Timer
    private final ElapsedTime detectionTimer = new ElapsedTime();
    private boolean objectDetectedState = false;

    // Intake Cycle State
    private boolean isIntakeMode = false;
    private boolean isOuttakeMode = false;
    private int intakeIndex = 0;

    // State variables for angle tracking
    private double lastServoAngle = 0.0;
    private double currentSpindexerAngle = 0.0;
    private boolean initialized = false;
    private int lastDetectedColor = 0;

    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        spindexer1 = hardwareMap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwareMap.get(CRServo.class, "spindexer2");

        spindexer1.setDirection(CRServo.Direction.REVERSE);
        spindexer2.setDirection(CRServo.Direction.REVERSE);

        try {
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        } catch (Exception e) {
            huskyLens = null;
        }

        spindexer1Encoder = hardwareMap.get(AnalogInput.class, "spindexer1Encoder");

        initializeAngle();
    }

    private void initializeAngle() {
        double voltage = Math.max(0, spindexer1Encoder.getVoltage());
        lastServoAngle = (voltage / ANALOG_REF_V) * 360.0;

        // Assume initial alignment (Servo 0 = Spindexer 0)
        // If the servo is not at 0, we calculate the relative start
        currentSpindexerAngle = (lastServoAngle * GEAR_RATIO) % 360.0;
        initialized = true;
        // Default to safe mode
        isIntakeMode = false;
        isOuttakeMode = false;
        setTargetPosition(currentSpindexerAngle);
    }

    public void setPower(double power) {
        isPIDMode = false;
        spindexer1.setPower(power);
        spindexer2.setPower(power);
    }

    public void spinIntakeCycle() {
        // Start Intake Cycle
        isIntakeMode = true;
        isPIDMode = true;
        autoCycleEnabled = true; // Enable Sensor Logic
        intakeIndex = 0;
        intakeCycleTimer.reset();
        setTargetPosition(POS_1_INTAKE);
    }

    public void spinIntake() {
        // Go to closest position and stay there
        isIntakeMode = true;
        isOuttakeMode = false;
        isPIDMode = true;
        autoCycleEnabled = false; // Disable Sensor Logic
        intakeCycleTimer.reset();

        // Find closest slot
        double currentAngle = getOutputAngle();
        double[] positions = { POS_1_INTAKE, POS_2_INTAKE, POS_3_INTAKE };

        int closestSlot = 0;
        // Use shortest absolute distance to find the physical closest slot
        // Directional logic in PID will handle HOW we get there (CW only)
        double minDistance = Math.abs(getAngleDifference(positions[0] + manualOffset, currentAngle));

        for (int i = 1; i < positions.length; i++) {
            double distance = Math.abs(getAngleDifference(positions[i] + manualOffset, currentAngle));
            if (distance < minDistance) {
                minDistance = distance;
                closestSlot = i;
            }
        }

        intakeIndex = closestSlot;
        setTargetSlot(intakeIndex);
    }

    public void spinOuttake() {
        setPower(OUTTAKE_POWER);
    }

    public void stop() {
        isIntakeMode = false;
        isOuttakeMode = false;
        if (!isPIDMode) {
            setTargetPosition(getOutputAngle());
        }
    }

    public void setTargetSlot(int slot) {
        intakeIndex = slot % 3;
        switch (intakeIndex) {
            case 1:
                setTargetPosition(POS_2_INTAKE);
                break;
            case 2:
                setTargetPosition(POS_3_INTAKE);
                break;
            case 0:
            default:
                setTargetPosition(POS_1_INTAKE);
                break;
        }
    }

    public void spinNextIntakePosition() {
        // Move one slot forward (0 -> 1 -> 2 -> 0)
        // Corrected to +1 for Clockwise movement
        isIntakeMode = true;
        isOuttakeMode = false;
        intakeIndex = (intakeIndex + 2) % 3;
        setTargetSlot(intakeIndex);
    }

    /**
     * Goes to the nearest intake position using shortest path (CW or CCW).
     * Used for recovery after RapidFire when we don't want directional constraints.
     */
    public void goToNearestIntakePosition() {
        isIntakeMode = false;
        isOuttakeMode = false;

        double currentAngle = getOutputAngle();
        double[] positions = { POS_1_INTAKE, POS_2_INTAKE, POS_3_INTAKE };

        int closestSlot = 0;
        double minDistance = Math.abs(getAngleDifference(positions[0], currentAngle));

        for (int i = 1; i < positions.length; i++) {
            double distance = Math.abs(getAngleDifference(positions[i], currentAngle));
            if (distance < minDistance) {
                minDistance = distance;
                closestSlot = i;
            }
        }

        intakeIndex = closestSlot;
        setTargetPosition(positions[closestSlot]);
    }

    public void spinNextOuttakePosition() {
        // Move one slot forward (Outtake direction)
        isIntakeMode = false;
        isOuttakeMode = true;
        intakeIndex = (intakeIndex + 1) % 3;
        switch (intakeIndex) {
            case 1:
                setTargetPosition(POS_2_OUTTAKE);
                break;
            case 2:
                setTargetPosition(POS_3_OUTTAKE);
                break;
            case 0:
            default:
                setTargetPosition(POS_1_OUTTAKE);
                break;
        }
    }

    public void spinNextOuttakePositionCW() {
        // Move one slot forward to outtake position, but clockwise
        isIntakeMode = true;
        isOuttakeMode = false;
        intakeIndex = (intakeIndex + 1) % 3;
        switch (intakeIndex) {
            case 1:
                setTargetPosition(POS_2_OUTTAKE);
                break;
            case 2:
                setTargetPosition(POS_3_OUTTAKE);
                break;
            case 0:
            default:
                setTargetPosition(POS_1_OUTTAKE);
                break;
        }
    }

    public void setAutoCycleEnabled(boolean enabled) {
        this.autoCycleEnabled = enabled;
    }

    public boolean isAutoCycleEnabled() {
        return this.autoCycleEnabled;
    }

    public void setTargetPosition(double angle) {
        if (isPIDMode && Math.abs(targetAngle - angle) < 0.1)
            return;

        targetAngle = angle;
        isPIDMode = true;
        pidTimer.reset();
        integralSum = 0;
        lastError = getAngleDifference(targetAngle, getOutputAngle());
    }

    /**
     * Updates and returns the actual angle of the Spindexer (Output).
     * Uses cumulative delta tracking to handle the gear ratio wraps.
     */
    public double getOutputAngle() {
        double voltage = Math.max(0, spindexer1Encoder.getVoltage());
        double currentServoAngle = (voltage / ANALOG_REF_V) * 360.0;

        double delta = currentServoAngle - lastServoAngle;

        // Handle wrap-around (shortest path for delta)
        while (delta < -180)
            delta += 360;
        while (delta > 180)
            delta -= 360;

        lastServoAngle = currentServoAngle;

        // Add scaled delta to spindexer angle
        double spindexerDelta = delta * GEAR_RATIO;
        currentSpindexerAngle += spindexerDelta;

        // Optimize range to 0-360
        currentSpindexerAngle %= 360.0;
        if (currentSpindexerAngle < 0)
            currentSpindexerAngle += 360.0;

        return currentSpindexerAngle;
    }

    public double getAngleDeg() {
        return getOutputAngle();
    }

    public double getAngleDeg2() {
        return 0.0;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(getAngleDifference(targetAngle, getOutputAngle())) < tolerance;
    }

    public void setManualOffset(double offset) {
        this.manualOffset = offset;
    }

    public double getManualOffset() {
        return manualOffset;
    }

    public void update() {
        // Ensure sensor is updated every loop
        double currentAngle = getOutputAngle();
        // Removed unconditional distance sensor reads from here

        // Auto-cycling logic (Sensor) - strictly disabled if we are targeting the Outtake (even if IntakeMode is spoofed)
        if (isIntakeMode && !isOuttakeMode && autoCycleEnabled && isPIDMode && targetAngle != POS_1_OUTTAKE && targetAngle != POS_2_OUTTAKE && targetAngle != POS_3_OUTTAKE) {

            // Check Debounce: Ensure we haven't just moved (wait for servo to settle)
            if (intakeCycleTimer.seconds() > DETECTION_DEBOUNCE) {

                // Use HuskyLens to detect ball presence via block size
                boolean detected = false;
                int detectedColor = 0;

                if (huskyLens != null) {
                    HuskyLens.Block[] blocks = huskyLens.blocks();
                    if (blocks != null && blocks.length > 0) {
                        for (HuskyLens.Block block : blocks) {
                            if (block.width > HUSKY_SIZE_THRESHOLD || block.height > HUSKY_SIZE_THRESHOLD) {
                                detected = true;
                                detectedColor = block.id; // 1 = Green, 2 = Purple
                                break;
                            }
                        }
                    }
                }

                if (detected) {

                    if (!objectDetectedState) {
                        // Start Timer
                        objectDetectedState = true;
                        detectionTimer.reset();
                        lastDetectedColor = detectedColor;
                    } else {
                        // Update color while detecting
                        if (detectedColor != 0) {
                            lastDetectedColor = detectedColor;
                        }

                        // Keep Timer Running
                        if (detectionTimer.seconds() > POST_DETECTION_DELAY) {

                            // Save color to the correct slot
                            slotColors[intakeIndex] = lastDetectedColor;

                            // Ball detected, move to next slot
                            spinNextIntakePosition();

                            // Reset everything for next slot
                            objectDetectedState = false;
                            intakeCycleTimer.reset();
                        }
                    }
                } else {
                    // No large block detected -> Reset state
                    objectDetectedState = false;
                }
            } else {
                // We are currently moving/debouncing, ignore HuskyLens
                objectDetectedState = false;
            }
        }

        if (isPIDMode) {
            loopPID();
        }

        if (telemetry != null) {
            telemetry.addData("Spindexer/Slot 0", getColorName(slotColors[0]));
            telemetry.addData("Spindexer/Slot 1", getColorName(slotColors[1]));
            telemetry.addData("Spindexer/Slot 2", getColorName(slotColors[2]));
        }
    }

    private String getColorName(int colorId) {
        if (colorId == 1) return "Green";
        if (colorId == 2) return "Purple";
        return "Empty/Unknown";
    }

    private void loopPID() {
        double current = getOutputAngle();
        double shortestError = getAngleDifference(targetAngle + manualOffset, current);
        double error;

        if (isIntakeMode) {
            // Force Clockwise (Negative error/power) but allow small CCW corrections for
            // overshoot
            error = shortestError;
            // If error is STRONGLY positive (more than 25 degrees "ahead"), assume we need
            // to wrap around
            // Otherwise, if it's just slightly positive (e.g. 5 deg), it's just an
            // overshoot, so let it pull back.
            if (error > 25.0)
                error -= 360.0;

        } else if (isOuttakeMode) {
            if (isIntakeMode) { // Special strict case (Sorting forces IntakeMode flag for CW)
                error = shortestError;
            } else {
                // Force Counter-Clockwise (Positive error/power) but allow small CW corrections
                // for overshoot
                error = shortestError;
                // If error is STRONGLY negative (more than 25 degrees "behind"), assume we need
                // to wrap around
                // Otherwise, if it's just slightly negative (e.g. -5 deg), it's just an
                // overshoot, so let it pull back.
                if (error < -25.0)
                    error += 360.0;
            }

        } else {
            // Default shortest path
            error = shortestError;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt < 1e-6)
            return;

        integralSum += error * dt;

        double deltaError = error - lastError;
        while (deltaError > 180)
            deltaError -= 360;
        while (deltaError < -180)
            deltaError += 360;

        double derivative = deltaError / dt;
        lastError = error;

        double pidOut = (error * kP) + (integralSum * kI) + (derivative * kD);
        double ff = Math.signum(error) * kF;

        if (Math.abs(error) < 4.0) {
            pidOut = 0;
            ff = 0;
            integralSum = 0;
        }

        double power = Range.clip(pidOut + ff, -1.0, 1.0);

        // Safety Clamp: Prevent CCW movement (Positive Power/Increasing Angle) during
        // Intake
        // We only want Negative Power (CW/Decreasing Angle)
        if (isIntakeMode && power > 0) {
            power = 0;
        }

        spindexer1.setPower(power);
        spindexer2.setPower(power);
    }

    private double getAngleDifference(double target, double current) {
        double diff = target - current;
        while (diff > 180)
            diff -= 360;
        while (diff < -180)
            diff += 360;
        return diff;
    }

    public void sortBalls(Motif targetMotif) {
        if (targetMotif == Motif.NONE) return;

        // Sequence mappings (1=Green, 2=Purple)
        int[] targetSequence = new int[3];
        if (targetMotif == Motif.GPP) { // 21
            targetSequence = new int[]{1, 2, 2};
        } else if (targetMotif == Motif.PGP) { // 22
            targetSequence = new int[]{2, 1, 2};
        } else if (targetMotif == Motif.PPG) { // 33
            targetSequence = new int[]{2, 2, 1};
        }

        // Find which slot sequence matches the target sequence
        int bestFirstSlot = 0;
        for (int i = 0; i < 3; i++) {
            if (slotColors[i] == targetSequence[0] &&
                slotColors[(i + 1) % 3] == targetSequence[1] &&
                slotColors[(i + 2) % 3] == targetSequence[2]) {
                bestFirstSlot = i;
                break;
            }
        }
        
        // If no perfect match is found, it will default to slot 0 which is safe.
        // We might want to fallback to just finding the unique color (e.g., green):
        // (Assuming there's always exactly 1 green and 2 purple, or vice versa)
        if (bestFirstSlot == 0 && slotColors[0] != targetSequence[0]) {
             for (int i = 0; i < 3; i++) {
                 // Try shifting by replacing exact match with just matching the unique color.
                 // If GPP, green is at index 0. If slot i has green, slot i is the first slot.
                 if (targetMotif == Motif.GPP && slotColors[i] == 1) bestFirstSlot = i;
                 if (targetMotif == Motif.PGP && slotColors[(i + 1) % 3] == 1) bestFirstSlot = i;
                 if (targetMotif == Motif.PPG && slotColors[(i + 2) % 3] == 1) bestFirstSlot = i;
             }
        }

        // Set to Outtake Mode but trick it by explicitly keeping `isIntakeMode = true` 
        // to force the PID constraint into strictly traveling clockwise. 
        // We also want to set `spinNextOuttakePositionCW()` style logic.
        isIntakeMode = true; // TRICK THIS TRUE to force CW (negative power)
        isOuttakeMode = false;
        intakeIndex = bestFirstSlot;

        switch (intakeIndex) {
            case 1:
                setTargetPosition(POS_2_OUTTAKE);
                break;
            case 2:
                setTargetPosition(POS_3_OUTTAKE);
                break;
            case 0:
            default:
                setTargetPosition(POS_1_OUTTAKE);
                break;
        }
    }

    private double getClockwiseDistance(double target, double current) {
        // Clockwise = Decreasing Angle (Current - Target)
        double diff = current - target;
        // Normalize to 0-360 range
        while (diff < 0)
            diff += 360;
        while (diff >= 360)
            diff -= 360;
        return diff;
    }
}