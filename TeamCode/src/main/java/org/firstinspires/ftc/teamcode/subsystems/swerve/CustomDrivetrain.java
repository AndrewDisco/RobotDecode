package org.firstinspires.ftc.teamcode.subsystems.swerve;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;

/**
 * This is the CustomDrivetrain class. This is an abstract class that extends
 * the Drivetrain class.
 * It is intended to be used as a base class for custom drivetrain
 * implementations.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public abstract class CustomDrivetrain extends Drivetrain {
    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;
    private boolean fieldCentric = true;

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the drivetrain.
     * Intended to work exactly like an arcade drive would in a typical TeleOp, this
     * method can be a copy pasted from
     * a robot-centric arcade drive implementation.
     *
     * @param forward  the forward power value, which would typically be
     *                 -gamepad1.left_stick_y in a normal arcade drive setup.
     * @param strafe   the strafe power value, which would typically be
     *                 gamepad1.left_stick_x in a normal arcade drive setup.
     * @param rotation the rotation power value, which would typically be
     *                 gamepad1.right_stick_x in a normal arcade drive setup.
     */
    public abstract void arcadeDrive(double forward, double strafe, double rotation);

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower,
                                   double robotHeading) {
        
        // For swerve, we DO NOT scale down pathingPower or headingPower here.
        // SwerveDrive's arcadeDrive naturally normalizes all pods, which preserves
        // the proportion between translation and rotation. Mecanum-style priority
        // scaling zeroes out translation when rotating at max speed, which causes
        // the robot to effectively stop moving in the XY plane.
        
        Vector combinedMovement = correctivePower.plus(pathingPower);
        
        return new double[] {
                combinedMovement.getXComponent(),
                combinedMovement.getYComponent(),
                headingPower.dot(new Vector(1, robotHeading))
        };
    }

    private boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    private Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus
                ? Math.min(findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling))
                : findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);
        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;

        lastTranslationalVector = translationalVector; // before rotation
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;

        if (fieldCentric) {
            translationalVector.rotateVector(-robotHeading); // this should make it field centric when field centric is
            // desired and robot centric otherwise
        }
        arcadeDrive(translationalVector.getXComponent(), translationalVector.getYComponent(), calculatedDrive[2]);
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {
    }
}