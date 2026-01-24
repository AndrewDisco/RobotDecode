package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SwerveModule {
    private final AnalogInput turnEncoder; // for rotation of servo
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;

    private PIDFController turnPID;

    // Angle offset in radians applied to raw encoder angle
    private final double angleOffsetRad;
    private final double xOffset;
    private final double yOffset;

    private Telemetry telemetry;
    private final String servoLabel;

    // REV analog reference voltage bounds for this specific pod
    private final double encoderMinVoltage;
    private final double encoderMaxVoltage;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;

    /**
     * Constructs the Swerve Pod
     *
     * @param servoName        Name of the pod's rotation servo
     * @param encoderName      Name of the pod's encoder input
     * @param motorName        Name of the pod's drive motor
     * @param pidfCoefficients PIDF coefficients for the pod's rotation control
     * @param driveDirection   Direction of the drive motor
     * @param servoDirection   Direction of the servo
     * @param angleOffsetDeg   In degrees, the negative of what the encoder reads
     *                         when the pod is facing
     *                         forward
     * @param offsets          Array of the pod's x and y offsets from the robot
     *                         center (units don't
     *                         matter, just relative size of x and y)
     * @param encoderMinVoltage The minimum voltage the analog encoder reports
     * @param encoderMaxVoltage The maximum voltage the analog encoder reports
     */
    public SwerveModule(HardwareMap hardwareMap, String servoName, String encoderName, String motorName,
                        PIDFCoefficients pidfCoefficients, DcMotorSimple.Direction driveDirection,
                        CRServo.Direction servoDirection, double angleOffsetDeg, double[] offsets,
                        double encoderMinVoltage, double encoderMaxVoltage) {
        this.turnServo = hardwareMap.get(CRServo.class, servoName);
        this.turnEncoder = hardwareMap.get(AnalogInput.class, encoderName);
        this.driveMotor = hardwareMap.get(DcMotorEx.class, motorName);

        this.turnPID = new PIDFController(pidfCoefficients);
        this.angleOffsetRad = Math.toRadians(angleOffsetDeg);
        this.xOffset = offsets[0];
        this.yOffset = offsets[1];

        this.encoderMinVoltage = encoderMinVoltage;
        this.encoderMaxVoltage = encoderMaxVoltage;

        this.servoLabel = servoName;

        setMotorToFloat();

        driveMotor.setDirection(driveDirection);
        turnServo.setDirection(servoDirection);

        turnServo.setPower(0);
    }

    public void setPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        this.turnPID = new PIDFController(pidfCoefficients);
    }

    public void setServoPower(double power) {
        lastTurnPower = power;
        turnServo.setPower(power);
    }

    public void setMotorPower(double power) {
        lastDrivePower = power;
        driveMotor.setPower(power);
    }

    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Commands pod to a wheel heading (radians) with a drive power [0, 1]
     */
    public void move(double targetAngleRad, double drivePower, boolean ignoreServoAngleChanges,
                     double motorCachingThreshold, double servoCachingThreshold) {
        // Convert hardware angle to radians and normalize
        double actualRad = getAngleAfterOffsetRad();
        actualRad = MathFunctions.normalizeAngle(actualRad);

        // Target angle is already correctly aligned with 0 = forward
        double desiredRad = targetAngleRad;
        desiredRad = MathFunctions.normalizeAngle(desiredRad);

        // Shortest-path error in radians (signed)
        double mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90° (pi/2)
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
            drivePower = -drivePower;

            mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
            dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        lastTargetRad = desiredRad;
        lastActualRad = actualRad;
        lastErrorRad = errorRad;

        // Setpoint close to current so PID follows shortest path
        double setpointRad = actualRad + errorRad;

        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(MathFunctions.getTurnDirection(actualRad, desiredRad));
        }

        turnPID.updateError(setpointRad - actualRad);
        double turnPower = MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

        if (ignoreServoAngleChanges) {
            lastTurnPower = 0;
            turnServo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }

    public double getXOffset() {
        return xOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    /**
     * Returns the current pod heading after applying the configured offset, in radians.
     *
     * @return heading in radians
     */
    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - angleOffsetRad;
    }

    /**
     * Returns the raw encoder angle in radians, in [0, 2pi].
     *
     * @return raw encoder angle in radians
     */
    public double getRawAngleRad() {
        double v = turnEncoder.getVoltage();
        if (encoderMaxVoltage == encoderMinVoltage)
            return 0;
        double normalized = (v - encoderMinVoltage) / (encoderMaxVoltage - encoderMinVoltage);
        normalized = MathFunctions.clamp(normalized, 0, 1);
        return normalized * (2.0 * Math.PI);
    }
    
    // Kept for outer compatibility if needed
    public double getRawAngleDeg() {
        return Math.toDegrees(getRawAngleRad());
    }

    public String debugString() {
        double rawAngleRad = getRawAngleRad();
        double offsetAngleRad = getAngleAfterOffsetRad();
        return servoLabel + " {" + "\ncurrent raw angle (rad/deg) = " + rawAngleRad + " / " + Math.toDegrees(rawAngleRad)
                + "\ncurrent angle after offset (rad/deg) = " + offsetAngleRad + " / " + Math.toDegrees(offsetAngleRad)
                + "\nservo Power = " + turnServo.getPower()
                + "\ndrive Power = " + driveMotor.getPower()
                + "\n}";
    }

    public double getEncoderVoltage() {
        return turnEncoder.getVoltage();
    }

    public double getServoPower() {
        return turnServo.getPower();
    }

    // Debug values
    private double lastTargetRad = 0;
    private double lastActualRad = 0;
    private double lastErrorRad = 0;

    public double getLastTargetRad() {
        return lastTargetRad;
    }

    public double getLastActualRad() {
        return lastActualRad;
    }

    public double getLastErrorRad() {
        return lastErrorRad;
    }

    public double getCurrentDraw() {
        return driveMotor.getCurrent(CurrentUnit.AMPS);
    }
}