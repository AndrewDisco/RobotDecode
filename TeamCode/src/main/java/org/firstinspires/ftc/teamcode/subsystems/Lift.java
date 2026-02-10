package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Lift {
    private final DcMotorEx ridicare;
    private final Telemetry telemetry;

    // --- PID & Feedforward Constants ---
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kG = 0.1; // Gravity feedforward (tune this!)

    // --- Limits ---
    public static int MAX_EXTENSION_LIMIT = 76800;
    public static int SOFT_LIMIT_THRESHOLD = 10; // Slow down when within 2000 ticks of max

    // --- State ---
    private int targetPosition = 0;
    private boolean isHolding = false;
    private boolean tensionEnabled = false; // Prevents instant tension on startup

    // --- PID Internal ---
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        ridicare = hardwareMap.get(DcMotorEx.class, "ridicare");

        ridicare.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use braking to help hold position
        ridicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders on init
        ridicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
    }

    /**
     * Main control loop. Call this every cycle in your OpMode.
     */
    public void update() {
        int currentPosition = getPosition();
        double power = 0;

        if (isHolding && tensionEnabled) {
            // PID Control to hold position
            power = calculatePID(targetPosition, currentPosition);
        } else {
            // If not holding, we might be in a manual state where power is 0 but we want to drift?
            // Usually, if we stop manual input, we switch to holding.
            // So this block might be empty if setPower handles the switch.
            // See setPower logic below.
        }

        // Apply power (if holding)
        if (isHolding && tensionEnabled) {
            ridicare.setPower(power);
        }

        // Telemetry
        telemetry.addData("Lift/Position (Encoder)", currentPosition);
        telemetry.addData("Lift/Target", targetPosition);
        telemetry.addData("Lift/Power", ridicare.getPower());
        telemetry.addData("Lift/Holding", isHolding);
        telemetry.addData("Lift/Tension Active", tensionEnabled);
    }

    /**
     * Sets the manual power for the lift.
     * Automatically handles soft limits and switching to hold mode.
     * @param power Input power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        int currentPosition = getPosition();

        // Enable tension once D-Pad Up (positive power) is pressed
        if (power > 0.05) {
            tensionEnabled = true;
        }

        // 1. Deadband / Hold Logic
        if (Math.abs(power) < 0.05) {
            if (!isHolding && tensionEnabled) {
                // Just stopped moving -> lock current position
                targetPosition = currentPosition;
                isHolding = true;
                timer.reset();
                integralSum = 0;
                lastError = 0;
            } else if (!tensionEnabled) {
                // Ensure motor is truly off before tension is enabled
                ridicare.setPower(0);
            }
            // If already holding, update() will handle the PID
            // So we don't set motor power here directly if holding
            return;
        }

        // We are moving manually
        isHolding = false;

        // 2. Soft Limits (Upward motion protection)
        if (power > 0) { // Going UP
            int distToMax = MAX_EXTENSION_LIMIT - currentPosition;

            if (distToMax <= 0) {
                // Hit the hard limit
                power = 0;
            } else if (distToMax < SOFT_LIMIT_THRESHOLD) {
                // Inside soft limit zone -> scale power down specifically
                // Map distance (0 to threshold) to (0.1 to 1.0) roughly
                double scale = (double) distToMax / SOFT_LIMIT_THRESHOLD;
                // Clip scale to be safe, though math above implies 0..1
                scale = Range.clip(scale, 0.1, 1.0);
                power *= scale;
            }
        }

        // 3. Apply Manual Power
        ridicare.setPower(power);
    }

    private double calculatePID(int target, int current) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        if (dt == 0) return 0; // Prevent divide by zero on first loop

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID + Gravity Feedforward
        // kG is constant power to hold against gravity
        double output = (error * kP) + (integralSum * kI) + (derivative * kD) + kG;

        return Range.clip(output, -1.0, 1.0);
    }

    public void stop() {
        ridicare.setPower(0);
        isHolding = false;
    }

    public int getPosition() {
        return ridicare.getCurrentPosition();
    }
}