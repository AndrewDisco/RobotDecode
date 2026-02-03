package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class Intake {

    public static double intakePower = 1;
    public static double idlePower = 0.0;

    public static int intakeDirection = 1;

    private final Spindexer spindexer;
    private final DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap, Spindexer spindexer) {
        this.spindexer = spindexer;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        updateDirection();
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void intake() {
        setPower(intakePower);
        if (spindexer != null) {
            spindexer.spinIntake();
        }
    }

    public void intakeOut() {
        setPower(-intakePower);

        if (spindexer != null) {
            spindexer.stop();
        }
    }

    public void intakeOnlyMotors() {
        setPower(intakePower);
    }

    public void intakeSmart() {
        setPower(intakePower);
        if (spindexer != null) {
            spindexer.spinIntakeCycle();
        }
    }

    public void stop() {
        setPower(idlePower);
        if (spindexer != null) {
            spindexer.stop();
        }
    }

    public void update() {
        // No more cycling logic needed
    }

    public void runWithButton(boolean buttonPressed) {
        if (buttonPressed) {
            intake();
        } else {
            stop();
        }
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void updateDirection() {
        intakeMotor.setDirection(intakeDirection == 0 ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getCurrentDraw() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}
