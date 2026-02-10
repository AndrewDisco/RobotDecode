package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Configurable
public class Outtake {

    public static double shooterPower = 0.75;
    public static double idlePower = 0.0;

    // Motor directions (0 = FORWARD, 1 = REVERSE)
    public static int lansator1Direction = 0;
    public static int lansator2Direction = 1;
    // PID coefficients (from ShooterTester)
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.00001;
    public static double kS = 0.0435;
    public static double kV = 0.000275;

    public static double MAX_VELOCITY = 1800.0;
    // New variable to prevent flywheel from spinning too slow during targeting
    public static double MIN_VELOCITY = 1150.0;

    // --- Long Distance Tuning ---
    // If distance > 75, apply this adjustment
    public static double LONG_DISTANCE_THRESHOLD = 75.0;
    public static double LONG_DISTANCE_ADJUSTMENT = 0.0;

    // --- Short Distance Tuning (NEW) ---
    // If distance < 60, apply this adjustment.
    // If shooting too high at close range, make this NEGATIVE (e.g., -5.0)
    public static double SHORT_DISTANCE_THRESHOLD = 70.0;
    public static double SHORT_DISTANCE_ADJUSTMENT = 0.0;
    // ----------------------------

    // --- Ball Tracking & Hood Adjustment (Configurable) ---
    public static double VELOCITY_DROP_THRESHOLD = 150.0;
    public static long BALL_DETECTION_COOLDOWN_MS = 100;
    public static long BALL_COUNT_RESET_TIMEOUT_MS = 1000;
    public static double THIRD_BALL_HOOD_DROP = 0.02;

    // Ball tracking state
    private int ballsShotCount = 0;
    private long lastBallDetectTime = 0;
    private double highestVelocitySinceLastDrop = 0;

    private double targetVelocity = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    // Smoothing for Distance
    // Median Filter Window Size (5 is good for rejecting vibration spikes)
    private static final int FILTER_WINDOW_SIZE = 5;
    private final ArrayList<Double> distanceBuffer = new ArrayList<>();

    public static double DISTANCE_SMOOTHING_FACTOR = 0.15; // Lower = smoother but slower
    private double lastSmoothedDistance = -1.0;

    private final DcMotorEx lansator1;
    private final DcMotorEx lansator2;
    private final Servo elevationServo;
    public static boolean useBrakeMode = false;
    private boolean manualActive = false;

    // Tuning Offsets
    public static double flywheelOffset = 0.0;
    public static double hoodOffset = 0.0;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public Outtake(HardwareMap hardwareMap) {

        lansator1 = hardwareMap.get(DcMotorEx.class, "lansator1");
        lansator2 = hardwareMap.get(DcMotorEx.class, "lansator2");
        elevationServo = hardwareMap.get(Servo.class, "elevationServo");

        // Expand PWM range for Axon servos to allow full travel and reach higher positions
        if (elevationServo instanceof ServoImplEx) {
            ((ServoImplEx) elevationServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        
        // Initialize servo to a safe position to wake it up and prevent startup faults
        // at extreme bounds (1.0 / 2500us).
        setElevation(0.5);

        // Use RUN_WITHOUT_ENCODER for custom PID
        lansator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lansator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        updateDirections();

        updateBrakeMode();

        stop();
    }

    public void update() {
        if (!manualActive) {
            if (targetVelocity > 0) {
                double vel1 = Math.abs(lansator1.getVelocity());
                double vel2 = Math.abs(lansator2.getVelocity());
                double currentVelocity = vel1; // Default to motor 1

                // Fallback: If lansator1 is dead (0 vel) but we expect it to move (target > 50)
                // and lansator2 is moving
                if (targetVelocity > 50 && vel1 < 10 && vel2 > 50) {
                    currentVelocity = vel2;
                }
                // --- BALL DETECTION LOGIC ---
                if (currentVelocity > highestVelocitySinceLastDrop) {
                    highestVelocitySinceLastDrop = currentVelocity;
                }

                long currentTime = System.currentTimeMillis();

                if ((highestVelocitySinceLastDrop - currentVelocity) >= VELOCITY_DROP_THRESHOLD) {
                    if ((currentTime - lastBallDetectTime) > BALL_DETECTION_COOLDOWN_MS) {
                        ballsShotCount++;
                        lastBallDetectTime = currentTime;
                        highestVelocitySinceLastDrop = currentVelocity;
                    }
                }

                if (currentTime - lastBallDetectTime > BALL_COUNT_RESET_TIMEOUT_MS && ballsShotCount > 0) {
                    ballsShotCount = 0;
                    highestVelocitySinceLastDrop = currentVelocity;
                }

                double error = targetVelocity - currentVelocity;
                integralSum += error;
                double derivative = error - lastError;
                lastError = error;

                double pid = (kP * error) + (kI * integralSum) + (kD * derivative);
                double ff = kS + (kV * targetVelocity);
                double power = pid + ff;

                power = Math.max(0, Math.min(1, power)); // Clamp 0-1

                setPower(power);
            } else {
                setPower(0);
                integralSum = 0;
                lastError = 0;
                ballsShotCount = 0;
                highestVelocitySinceLastDrop = 0;
            }
        }

        panelsTelemetry.addData("outtake/currentVelocity", Math.abs(lansator1.getVelocity()));
        panelsTelemetry.addData("outtake/hoodAngle", lastBaseElevation);
    }

    public void setVelocity(double ticksPerSecond) {
        this.manualActive = false;
        if (this.targetVelocity == 0 && ticksPerSecond > 0) {
            ballsShotCount = 0;
            highestVelocitySinceLastDrop = 0;
            lastBallDetectTime = System.currentTimeMillis();
        }
        this.targetVelocity = ticksPerSecond;
    }

    private double elevationOffset = 0.0;

    public void setElevation(double position) {
        lastBaseElevation = position;
        // Clamp to ensure safety
        double constrainedPos = clamp(position + elevationOffset + hoodOffset, 0.2, 0.99);
        elevationServo.setPosition(constrainedPos);
    }

    public void updateShooterFromDistance(double distance) {
        // --- 1. Median Filter (Reject Spikes) ---
        // Only accept non-zero readings into the buffer, unless we have no data yet
        if (distance > 1.0 || distanceBuffer.isEmpty()) {
            distanceBuffer.add(distance);
        }

        // Keep buffer size fixed
        if (distanceBuffer.size() > FILTER_WINDOW_SIZE) {
            distanceBuffer.remove(0);
        }

        // Calculate Median
        List<Double> sortedBuffer = new ArrayList<>(distanceBuffer);
        Collections.sort(sortedBuffer);

        double medianDistance;
        if (sortedBuffer.isEmpty()) {
            medianDistance = distance;
        } else {
            medianDistance = sortedBuffer.get(sortedBuffer.size() / 2);
        }

        // --- 2. Low Pass Filter (Smooth Vibration) ---
        if (lastSmoothedDistance == -1.0) {
            lastSmoothedDistance = medianDistance;
        } else {
            lastSmoothedDistance = (medianDistance * DISTANCE_SMOOTHING_FACTOR)
                    + (lastSmoothedDistance * (1 - DISTANCE_SMOOTHING_FACTOR));
        }

        ShooterState state = calculateShooterState(lastSmoothedDistance);
        setVelocity(state.velocity);
        setElevation(state.servoPos);
    }

    public void addElevationOffset(double delta) {
        elevationOffset += delta;
        setElevation(lastBaseElevation);
    }

    // We need to track the base elevation to re-apply it with offset.
    private double lastBaseElevation = 0.5; // Default ?

    public void resetElevationOffset() {
        elevationOffset = 0.0;
        setElevation(lastBaseElevation);
    }

    private static class ShooterState {
        double velocity;
        double servoPos;

        public ShooterState(double velocity, double servoPos) {
            this.velocity = velocity;
            this.servoPos = servoPos;
        }
    }

    // Linear interpolation
    // Closest distance lookup
    private ShooterState calculateShooterState(double distance) {

        // --- 3-ZONE DISTANCE LOGIC ---
        double calculationDistance = distance;

        if (distance > LONG_DISTANCE_THRESHOLD) {
            // Zone 1: Long Range (Far)
            calculationDistance += LONG_DISTANCE_ADJUSTMENT;
        }
        else if (distance < SHORT_DISTANCE_THRESHOLD) {
            // Zone 2: Short Range (Close)
            // If shooting too high, set SHORT_DISTANCE_ADJUSTMENT to a negative value (e.g. -5.0)
            calculationDistance += SHORT_DISTANCE_ADJUSTMENT;
        }
        // Zone 3: Mid Range (Between thresholds) uses raw distance

        // Flywheel Linear Regression
        double calculatedSpeed = (5.0 * calculationDistance) + 850.0;

        // Apply Clamp with MIN_VELOCITY
        double finalSpeed = clamp(calculatedSpeed + flywheelOffset, MIN_VELOCITY, MAX_VELOCITY);

        // Hood Linear Regression
        double calculatedPos = (0.005 * calculationDistance) - 0.0772727;

        if (ballsShotCount >= 2) {
            calculatedPos -= THIRD_BALL_HOOD_DROP;
        }

        double finalPos = clamp(calculatedPos, 0.2, 0.99);

        return new ShooterState(finalSpeed, finalPos);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void shoot() {
        manualActive = true;
        setPower(shooterPower);
        this.targetVelocity = 0;
    }

    public void reverse() {
        manualActive = true;
        this.targetVelocity = 0;
        setPower(-0.7);
    }

    public void stop() {
        manualActive = false;
        this.targetVelocity = 0;
        setPower(idlePower);
    }

    public void runWithButton(boolean buttonPressed) {
        if (buttonPressed) {
            shoot();
        } else {
            stop();
        }
    }

    public void setPower(double power) {
        lansator1.setPower(power);
        lansator2.setPower(power);
    }

    public void setPower(double power1, double power2) {
        lansator1.setPower(power1);
        lansator2.setPower(power2);
    }

    public void updateDirections() {
        lansator1.setDirection(intToDirection(lansator1Direction));
        lansator2.setDirection(intToDirection(lansator2Direction));
    }

    public void updateBrakeMode() {
        DcMotor.ZeroPowerBehavior behavior = useBrakeMode ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;

        lansator1.setZeroPowerBehavior(behavior);
        lansator2.setZeroPowerBehavior(behavior);
    }

    public void updateConfig() {
        updateDirections();
        updateBrakeMode();
    }

    public double getLansator1Power() {
        return lansator1.getPower();
    }

    public double getLansator2Power() {
        return lansator2.getPower();
    }

    public double getLansator1Velocity() {
        return lansator1.getVelocity();
    }

    public double getLansator2Velocity() {
        return lansator2.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isReady(double tolerance) {
        return targetVelocity > 0 && Math.abs(targetVelocity - Math.abs(getLansator1Velocity())) < tolerance;
    }

    private DcMotorSimple.Direction intToDirection(int direction) {
        return direction == 0 ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }

    public int getLansator1Position() {
        return lansator1.getCurrentPosition();
    }

    public int getLansator2Position() {
        return lansator2.getCurrentPosition();
    }

    public String debugString() {
        return "Outake{" +
                "\nlansator1Power=" + getLansator1Power() +
                "\nlansator2Power=" + getLansator2Power() +
                "\nlansator1Velocity=" + getLansator1Velocity() +
                "\nlansator2Velocity=" + getLansator2Velocity() +
                "\ntargetVelocity=" + targetVelocity +
                "\nballsShotCount=" + ballsShotCount +
                "\nhighestVelocitySinceLastDrop=" + highestVelocitySinceLastDrop +
                "\n}";
    }

    public double getCurrentDraw() {
        return lansator1.getCurrent(CurrentUnit.AMPS) + lansator2.getCurrent(CurrentUnit.AMPS);
    }
}