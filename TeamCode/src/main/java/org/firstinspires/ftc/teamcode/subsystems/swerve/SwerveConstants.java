package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SwerveConstants {

    public static double xVelocity = 66.01314274344858; // TODO: Change based on your robot
    public static double yVelocity = 67.55022184116635; // TODO: Change based on your robot

    // Motor and servo settings, trebuie sa fie static pentru @Configurable
    public static boolean useBrakeModeInTeleOp = false;
    public static double maxPower = 1.0;
    public static double motorCachingThreshold = 0.01;
    public static double servoCachingThreshold = 0.05;
    public static boolean useVoltageCompensation = false;
    public static double nominalVoltage = 12.0;
    public static double staticFrictionCoefficient = 0.1;

    // Hardware names
    public static String leftFrontMotorName = "flMotor";
    public static String leftFrontServoName = "flServo";
    public static String leftFrontEncoderName = "flEncoder";

    public static String rightFrontMotorName = "frMotor";
    public static String rightFrontServoName = "frServo";
    public static String rightFrontEncoderName = "frEncoder";

    public static String leftRearMotorName = "blMotor";
    public static String leftRearServoName = "blServo";
    public static String leftRearEncoderName = "blEncoder";

    public static String rightRearMotorName = "brMotor";
    public static String rightRearServoName = "brServo";
    public static String rightRearEncoderName = "brEncoder";

    // Encoder Voltage References
    public static double leftFrontEncoderMaxVoltage = 3.241;
    public static double leftFrontEncoderMinVoltage = 0.009;

    public static double rightFrontEncoderMaxVoltage = 3.247;
    public static double rightFrontEncoderMinVoltage = 0.003;

    public static double leftRearEncoderMaxVoltage = 3.234;
    public static double leftRearEncoderMinVoltage = 0.036;

    public static double rightRearEncoderMaxVoltage = 3.257;
    public static double rightRearEncoderMinVoltage = 0.059;

    // PID
    public static double leftFrontKp = 0.25;
    public static double leftFrontKi = 0.0;
    public static double leftFrontKd = 0.00001; // 0.00000001;
    public static double leftFrontKf = 0.0;

    public static double rightFrontKp = 0.25;
    public static double rightFrontKi = 0.0;
    public static double rightFrontKd = 0.00001; // 0.00000001;
    public static double rightFrontKf = 0.0;

    public static double leftRearKp = 0.25;
    public static double leftRearKi = 0.0;
    public static double leftRearKd = 0.00001; // 0.00000001;
    public static double leftRearKf = 0.0;

    public static double rightRearKp = 0.25;
    public static double rightRearKi = 0.0;
    public static double rightRearKd = 0.00001; // 0.00000001;
    public static double rightRearKf = 0.0;

    // Pod angle offsets, aka swerve offset
    public static double leftFrontPodAngleOffsetDeg = 60.0;
    public static double rightFrontPodAngleOffsetDeg = 173.8;
    public static double leftRearPodAngleOffsetDeg = 338.0;
    public static double rightRearPodAngleOffsetDeg = 19.9;

    // Pod XY offsets
    public static double leftFrontPodX = -0.17125;
    public static double leftFrontPodY = 0.15925;

    public static double rightFrontPodX = 0.17125;
    public static double rightFrontPodY = 0.15925;

    public static double leftRearPodX = -0.17125;
    public static double leftRearPodY = -0.15925;

    public static double rightRearPodX = 0.17125;
    public static double rightRearPodY = -0.15925;

    // Motor directions as integers (0 = FORWARD, 1 = REVERSE)
    public static int leftFrontMotorDirection = 1;
    public static int rightFrontMotorDirection = 0;
    public static int leftRearMotorDirection = 1;
    public static int rightRearMotorDirection = 0;

    // Servo directions as integers (0 = FORWARD, 1 = REVERSE)
    public static int leftFrontServoDirection = 0;
    public static int rightFrontServoDirection = 0;
    public static int leftRearServoDirection = 0;
    public static int rightRearServoDirection = 0;

    public SwerveConstants() {
    }

    // Helper to get PIDFCoefficients from static values
    public PIDFCoefficients getLeftFrontTurnPID() {
        return new PIDFCoefficients(leftFrontKp, leftFrontKi, leftFrontKd, leftFrontKf);
    }

    public PIDFCoefficients getRightFrontTurnPID() {
        return new PIDFCoefficients(rightFrontKp, rightFrontKi, rightFrontKd, rightFrontKf);
    }

    public PIDFCoefficients getLeftRearTurnPID() {
        return new PIDFCoefficients(leftRearKp, leftRearKi, leftRearKd, leftRearKf);
    }

    public PIDFCoefficients getRightRearTurnPID() {
        return new PIDFCoefficients(rightRearKp, rightRearKi, rightRearKd, rightRearKf);
    }

    // Helper methods to get pod offsets as arrays
    public double[] getLeftFrontPodXYOffsets() {
        return new double[] { leftFrontPodX, leftFrontPodY };
    }

    public double[] getRightFrontPodXYOffsets() {
        return new double[] { rightFrontPodX, rightFrontPodY };
    }

    public double[] getLeftRearPodXYOffsets() {
        return new double[] { leftRearPodX, leftRearPodY };
    }

    public double[] getRightRearPodXYOffsets() {
        return new double[] { rightRearPodX, rightRearPodY };
    }

    // Convert integer to motor direction
    private static DcMotorEx.Direction intToMotorDirection(int direction) {
        return direction == 0 ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
    }

    // Integer to servo direction
    private static CRServo.Direction intToServoDirection(int direction) {
        return direction == 0 ? CRServo.Direction.FORWARD : CRServo.Direction.REVERSE;
    }

    // Getters for motor direction
    public DcMotorEx.Direction getLeftFrontMotorDirection() {
        return intToMotorDirection(leftFrontMotorDirection);
    }

    public DcMotorEx.Direction getRightFrontMotorDirection() {
        return intToMotorDirection(rightFrontMotorDirection);
    }

    public DcMotorEx.Direction getLeftRearMotorDirection() {
        return intToMotorDirection(leftRearMotorDirection);
    }

    public DcMotorEx.Direction getRightRearMotorDirection() {
        return intToMotorDirection(rightRearMotorDirection);
    }

    public CRServo.Direction getLeftFrontServoDirection() {
        return intToServoDirection(leftFrontServoDirection);
    }

    public CRServo.Direction getRightFrontServoDirection() {
        return intToServoDirection(rightFrontServoDirection);
    }

    public CRServo.Direction getLeftRearServoDirection() {
        return intToServoDirection(leftRearServoDirection);
    }

    public CRServo.Direction getRightRearServoDirection() {
        return intToServoDirection(rightRearServoDirection);
    }

    // Getters for static values
    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public boolean getUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public double getServoCachingThreshold() {
        return servoCachingThreshold;
    }

    public boolean getUseVoltageCompensation() {
        return useVoltageCompensation;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public String getLeftFrontServoName() {
        return leftFrontServoName;
    }

    public String getLeftFrontEncoderName() {
        return leftFrontEncoderName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public String getRightFrontServoName() {
        return rightFrontServoName;
    }

    public String getRightFrontEncoderName() {
        return rightFrontEncoderName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public String getLeftRearServoName() {
        return leftRearServoName;
    }

    public String getLeftRearEncoderName() {
        return leftRearEncoderName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public String getRightRearServoName() {
        return rightRearServoName;
    }

    public String getRightRearEncoderName() {
        return rightRearEncoderName;
    }

    public double getLeftFrontPodAngleOffsetDeg() {
        return leftFrontPodAngleOffsetDeg;
    }

    public double getRightFrontPodAngleOffsetDeg() {
        return rightFrontPodAngleOffsetDeg;
    }

    public double getLeftRearPodAngleOffsetDeg() {
        return leftRearPodAngleOffsetDeg;
    }

    public double getRightRearPodAngleOffsetDeg() {
        return rightRearPodAngleOffsetDeg;
    }

    public double getLeftFrontEncoderMaxVoltage() {
        return leftFrontEncoderMaxVoltage;
    }

    public double getLeftFrontEncoderMinVoltage() {
        return leftFrontEncoderMinVoltage;
    }

    public double getRightFrontEncoderMaxVoltage() {
        return rightFrontEncoderMaxVoltage;
    }

    public double getRightFrontEncoderMinVoltage() {
        return rightFrontEncoderMinVoltage;
    }

    public double getLeftRearEncoderMaxVoltage() {
        return leftRearEncoderMaxVoltage;
    }

    public double getLeftRearEncoderMinVoltage() {
        return leftRearEncoderMinVoltage;
    }

    public double getRightRearEncoderMaxVoltage() {
        return rightRearEncoderMaxVoltage;
    }

    public double getRightRearEncoderMinVoltage() {
        return rightRearEncoderMinVoltage;
    }
}