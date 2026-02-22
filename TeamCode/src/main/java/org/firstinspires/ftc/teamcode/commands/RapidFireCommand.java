package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class RapidFireCommand {
    private final Outtake outtake;
    private final Spindexer spindexer;
    private final Intake intake;

    private enum State {
        IDLE,
        START,
        RAMP_UP,
        FIRING,
        DONE
    }

    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // Tolerance for Flywheel velocity (ticks/sec)
    private static final double FLYWHEEL_TOLERANCE = 150.0;
    // Tolerance for Spindexer alignment (degrees)
    private static final double SPINDEXER_TOLERANCE = 5.0;

    public RapidFireCommand(Outtake outtake, Spindexer spindexer, Intake intake) {
        this.outtake = outtake;
        this.spindexer = spindexer;
        this.intake = intake;
    }

    private int shotsAdjusted = 0;
    private final ElapsedTime shotTimer = new ElapsedTime();

    public void start() {
        if (state == State.IDLE || state == State.DONE) {
            // Ensure we override any Intake mode
            spindexer.stop();
            state = State.START;
            intake.intakeOnlyMotors();

            outtake.resetElevationOffset();
            shotsAdjusted = 0;
        }
    }

    public void stop() {
        state = State.IDLE;
        spindexer.goToNearestIntakePosition();
        intake.setPower(0);
        outtake.resetElevationOffset();
        // We do not stop outtake here as it might be in manual mode or controlled
        // elsewhere
    }

    public boolean isActive() {
        return state != State.IDLE && state != State.DONE;
    }

    public void update() {
        switch (state) {
            case IDLE:
            case DONE:
                break;

            case START:
                // Wait for flywheel to be ready
                if (outtake.isReady(FLYWHEEL_TOLERANCE)) {
                    intake.intakeOnlyMotors();
                    state = State.RAMP_UP;
                    timer.reset();
                }
                break;

            case RAMP_UP:
                spindexer.setPower(0.25);
                if (timer.seconds() > 0.2) {
                    state = State.FIRING;
                }
                break;

            case FIRING:
                // Continuous spin
                spindexer.spinOuttake();
                intake.intakeOnlyMotors(); // Ensure intake helps feed
                break;
        }
    }

    public String getStatus() {
        return "outtake encoder: " + outtake.getLansator1Velocity() + " " + state;
    }
}
