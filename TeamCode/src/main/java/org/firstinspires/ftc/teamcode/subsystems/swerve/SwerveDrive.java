package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

public class SwerveDrive extends CustomDrivetrain {
    public SwerveConstants constants;

    private boolean useBrakeModeInTeleOp;
    private double motorCachingThreshold;
    private double servoCachingThreshold;
    private double staticFrictionCoefficient;

    private final SwerveModule leftFrontPod;
    private final SwerveModule rightFrontPod;
    private final SwerveModule leftRearPod;
    private final SwerveModule rightRearPod;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastRotation = 0;

    private final VoltageSensor voltageSensor;

    private final SwerveModule[] pods;

    private final HardwareMap hardwareMap;

    public SwerveDrive(HardwareMap hardwareMap, SwerveConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        pods = new SwerveModule[4];
        updateConstants();

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

        pods[0] = leftFrontPod;
        pods[1] = rightFrontPod;
        pods[2] = leftRearPod;
        pods[3] = rightRearPod;
    }

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {
        lastForward = forward;
        lastStrafe = strafe;
        lastRotation = rotation;

        // stores forward and strafe values as the translation vector with max magnitude
        // of 1
        Vector rawTrans = new Vector(Range.clip(Math.hypot(strafe, forward), 0, 1), Math.atan2(strafe, forward));

        boolean ignoreTrans = rawTrans.getMagnitude() < 0.05;
        boolean ignoreRotation = Math.abs(rotation) < 0.05;

        double rotationScalar = (ignoreRotation) ? 0 : rotation;

        Vector[] podVectors = new Vector[pods.length];

        for (int i = 0; i < pods.length; i++) {
            SwerveModule pod = pods[i];

            Vector translationVector = ignoreTrans ? new Vector(0, 0) : rawTrans;

            // Map pod hardware offsets (X=Right, Y=Forward) to Pedro coordinates (X=Forward, Y=Left)
            // PedroX = pod.getYOffset()
            // PedroY = -pod.getXOffset()
            Vector rotationVector = new Vector(rotationScalar, Math.atan2(-pod.getXOffset(), pod.getYOffset()));

            // this gets the perpendicular vector for the wheel
            rotationVector.rotateVector(Math.PI / 2);

            podVectors[i] = translationVector.plus(rotationVector);
        }

        // finding if any vector has magnitude > maxPowerScaling
        double maxMagnitude = maxPowerScaling;
        for (Vector podVector : podVectors) {
            // voltage compensation impl copied straight from mecanum basically
            if (voltageCompensation) {
                double voltageNormalized = getVoltageNormalized();
                podVector.times(voltageNormalized);
            }
            maxMagnitude = Math.max(maxMagnitude, podVector.getMagnitude());
        }

        for (int podNum = 0; podNum < pods.length; podNum++) {
            Vector finalVector = podVectors[podNum].times(maxPowerScaling / maxMagnitude); // Normalizing if necessary
            // while preserving relative
            // sizes

            // 2*Pi-theta because servos have positive clockwise rotation, while our angles
            // are counterclockwise
            pods[podNum].move(finalVector.getTheta(), finalVector.getMagnitude(),
                    ignoreTrans && ignoreRotation, motorCachingThreshold, servoCachingThreshold); // seing if motor /
            // servo caching is an
            // issue
        }
    }

    public void xLock() {
        for (SwerveModule pod : pods) {
            double theta = Math.atan2(-pod.getXOffset(), pod.getYOffset()) + Math.PI / 2.0;
            pod.move(theta, 0, false, motorCachingThreshold, servoCachingThreshold);
        }
    }

    @Override
    public void updateConstants() {
        this.useBrakeModeInTeleOp = constants.getUseBrakeModeInTeleOp();
        if (!maxPowerOverridden) {
            this.maxPowerScaling = constants.getMaxPower();
        }
        this.motorCachingThreshold = constants.getMotorCachingThreshold();
        this.servoCachingThreshold = constants.getServoCachingThreshold();
        this.voltageCompensation = constants.getUseVoltageCompensation();
        this.nominalVoltage = constants.getNominalVoltage();
        this.staticFrictionCoefficient = constants.getStaticFrictionCoefficient();

        if (pods != null && pods[0] != null) {
            pods[0].setPIDFCoefficients(constants.getLeftFrontTurnPID());
            pods[1].setPIDFCoefficients(constants.getRightFrontTurnPID());
            pods[2].setPIDFCoefficients(constants.getLeftRearTurnPID());
            pods[3].setPIDFCoefficients(constants.getRightRearTurnPID());
        }
    }

    @Override
    public void breakFollowing() {
        for (SwerveModule pod : pods) {
            pod.setMotorPower(0);
            pod.setMotorToBreak();
            // pod.disableServo();
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            for (SwerveModule pod : pods) {
                pod.setMotorToBreak();
            }
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            for (SwerveModule pod : pods) {
                pod.setMotorToBreak();
            }
        } else {
            for (SwerveModule pod : pods) {
                pod.setMotorToFloat();
            }
        }
    }

    @Override
    public double xVelocity() {
        return constants.getXVelocity();
    }

    @Override
    public double yVelocity() {
        return constants.getYVelocity();
    }

    @Override
    public void setXVelocity(double xMovement) {
        SwerveConstants.xVelocity = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        SwerveConstants.yVelocity = yMovement;
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient))
                / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    @Override
    public String debugString() {
        return "Swerve{" +
                "\nforward input=" + lastForward +
                "\n, strafe input=" + lastStrafe +
                "\n, rotation input=" + lastRotation +
                "\n, unrotated translationVector x" + lastTranslationalVector.getXComponent() +
                "\n, unrotated translationVector y" + lastTranslationalVector.getYComponent() +
                "\n, correctivePower x" + lastCorrectivePower.getXComponent() +
                "\n, correctivePower y" + lastCorrectivePower.getYComponent() +
                "\n, pathingPower x" + lastPathingPower.getXComponent() +
                "\n, pathingPower y" + lastPathingPower.getYComponent() +
                "\n, headingPower magnitude" + lastHeadingPower.getMagnitude() +
                "\n, headingPower direction" + lastHeadingPower.getTheta() +
                "\nrobot heading" + lastHeading +
                "\nmaxPowerScaling" + maxPowerScaling +
                "\nmaxPowerOverridden" + maxPowerOverridden +
                // ", leftFront=" + leftFrontPod.debugString() +
                // ", rightFront=" + rightFrontPod.debugString() +
                // ", rightRear=" + rightRearPod.debugString() +
                "\n}";
    }

    // Getter methods for encoder positions
    public double getLeftFrontEncoderPosition() {
        return leftFrontPod.getRawAngleDeg();
    }

    public double getRightFrontEncoderPosition() {
        return rightFrontPod.getRawAngleDeg();
    }

    public double getLeftRearEncoderPosition() {
        return leftRearPod.getRawAngleDeg();
    }

    public double getRightRearEncoderPosition() {
        return rightRearPod.getRawAngleDeg();
    }

    public double getLeftFrontEncoderVoltage() {
        return leftFrontPod.getEncoderVoltage();
    }

    public double getRightFrontEncoderVoltage() {
        return rightFrontPod.getEncoderVoltage();
    }

    public double getLeftRearEncoderVoltage() {
        return leftRearPod.getEncoderVoltage();
    }

    public double getRightRearEncoderVoltage() {
        return rightRearPod.getEncoderVoltage();
    }

    public SwerveModule[] getModules() {
        return pods;
    }

    public double getCurrentDraw() {
        double totalCurrent = 0;
        for (SwerveModule pod : pods) {
            totalCurrent += pod.getCurrentDraw();
        }
        return totalCurrent;
    }

    private boolean maxPowerOverridden = false;

    public void setMaxPower(double maxPower) {
        this.maxPowerScaling = maxPower;
        this.maxPowerOverridden = true;
    }

    public void resetMaxPower() {
        this.maxPowerOverridden = false;
        updateConstants();
    }
}