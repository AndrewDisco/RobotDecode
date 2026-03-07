package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "auto red far", group = "auto")
@Configurable
@Disabled
public class AutoRedFar extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Subsystems
    private Outtake outtake;
    private Spindexer spindexer;
    private Intake intake;
    private Turret turret;
    private RapidFireCommand rapidFire;

    // Tuning constants for shooting sequence
    public static double TIME_SHOOT_SEQUENCE = 4.0; // Fallback time if velocity doesn't recover
    public static double MIN_FIRE_TIME = 2.0; // Minimum time to fire before checking velocity

    // Path Durations (Tunable)
    public static double timeShoot1 = 1.5; // Travel time to shoot position
    public static double timeEnableOuttake = 1.0; // Reduced time to wait for spin up if we check velocity

    // Poses - Mirrored from AutoBlueFar
    private final Pose startPose = mirror(new Pose(56.000, 8.000, Math.toRadians(90)));
    private final Pose preloadShootPose = mirror(new Pose(55.000, 13.000, Math.toRadians(95)));
    private final Pose closeSpikeMarkAlignPose = mirror(new Pose(48.000, 36.000, Math.toRadians(180)));
    private final Pose spikeMarkIntakeFinalPose = mirror(new Pose(12.000, 36.000, Math.toRadians(180)));
    private final Pose shoot2Pose = mirror(new Pose(48.000, 13.000, Math.toRadians(113)));
    private final Pose humanPlayerIntakeAlignPose = mirror(new Pose(19.000, 9.000, Math.toRadians(191)));
    private final Pose humanPlayerIntakePose = mirror(new Pose(12.000, 11.000, Math.toRadians(191)));
    private final Pose shoot3Pose = mirror(new Pose(48.000, 13.000, Math.toRadians(113)));
    private final Pose leavePose = mirror(new Pose(48.000, 35.000, Math.toRadians(90)));

    // Paths
    private PathChain preloadshoot;
    private PathChain closespikemarkalign;
    private PathChain spikemarkintakefinal;
    private PathChain shoot2;
    private PathChain humanplayerintakealign;
    private PathChain humanplayerintake;
    private PathChain shoot3;
    private PathChain leave;

    private Pose mirror(Pose p) {
        return new Pose(141.5 - p.getX(), p.getY(), MathFunctions.normalizeAngle(Math.PI - p.getHeading()));
    }

    public void buildPaths() {
        preloadshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preloadShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadShootPose.getHeading())
                .build();

        closespikemarkalign = follower.pathBuilder()
                .addPath(new BezierLine(preloadShootPose, closeSpikeMarkAlignPose))
                .setLinearHeadingInterpolation(preloadShootPose.getHeading(), closeSpikeMarkAlignPose.getHeading())
                .build();

        spikemarkintakefinal = follower.pathBuilder()
                .addPath(new BezierLine(closeSpikeMarkAlignPose, spikeMarkIntakeFinalPose))
                .setLinearHeadingInterpolation(closeSpikeMarkAlignPose.getHeading(),
                        spikeMarkIntakeFinalPose.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(spikeMarkIntakeFinalPose, shoot2Pose))
                .setLinearHeadingInterpolation(spikeMarkIntakeFinalPose.getHeading(), shoot2Pose.getHeading())
                .build();

        humanplayerintakealign = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, humanPlayerIntakeAlignPose))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), humanPlayerIntakeAlignPose.getHeading())
                .build();

        humanplayerintake = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerIntakeAlignPose, humanPlayerIntakePose))
                .setLinearHeadingInterpolation(humanPlayerIntakeAlignPose.getHeading(),
                        humanPlayerIntakePose.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerIntakePose, shoot3Pose))
                .setLinearHeadingInterpolation(humanPlayerIntakePose.getHeading(), shoot3Pose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, leavePose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start pathing to first shot position
                follower.followPath(preloadshoot, true);
                setPathState(1);
                break;
            case 1:
                // Traveling to preloadshoot
                if (!follower.isBusy()) {
                    // Arrived at shooting position.
                    prepareShot();
                    setPathState(10);
                }
                break;

            // --- SHOOTING SEQUENCE START (Preload) ---
            case 10:
                prepareShot(); // Keep updating target while spinning up
                // Wait for Outtake to reach target velocity
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(11);
                }
                break;
            case 11:
                prepareShot(); // Keep updating target while firing
                // Rapid Fire with Velocity & Time Check
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    // Proceed to next path
                    intake.intakeSmart();
                    follower.followPath(closespikemarkalign, true);
                    setPathState(2);
                }
                break;
            // --- SHOOTING SEQUENCE END ---

            case 2:
                // Traveling to close spike mark align
                if (!follower.isBusy()) {
                    follower.followPath(spikemarkintakefinal, true);
                    Constants.swerveDrive.setMaxPower(0.3);
                    setPathState(3);
                }
                break;
            case 3:
                // Traveling to spike mark intake final
                if (!follower.isBusy()) {
                    // Reached intake position. Dwell to ensure intake.
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        follower.followPath(shoot2, true);
                        Constants.swerveDrive.setMaxPower(0.5);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                // Traveling to shoot2
                if (!follower.isBusy()) {
                    intake.stop();
                    prepareShot();
                    setPathState(20);
                }
                break;

            // --- SECOND SHOOTING SEQUENCE START ---
            case 20:
                prepareShot(); // Keep updating target while spinning up
                // Wait for Outtake
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(21);
                }
                break;
            case 21:
                prepareShot(); // Keep updating target while firing
                // Rapid Fire
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    intake.intakeSmart();
                    follower.followPath(humanplayerintakealign, true);
                    Constants.swerveDrive.setMaxPower(0.5);
                    setPathState(5);
                }
                break;
            // --- SECOND SHOOTING SEQUENCE END ---

            case 5:
                // Traveling to human player intake align
                if (!follower.isBusy()) {
                    follower.followPath(humanplayerintake, true);
                    Constants.swerveDrive.setMaxPower(0.3);
                    setPathState(51);
                }
                break;
            case 51:
                // Traveling to human player intake final
                if (!follower.isBusy()) {
                    // Dwell to intake
                    if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                        follower.followPath(shoot3, true);
                        Constants.swerveDrive.setMaxPower(0.6);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                // Traveling to shoot3
                if (!follower.isBusy()) {
                    intake.stop();
                    prepareShot();
                    setPathState(30);
                }
                break;

            // --- THIRD SHOOTING SEQUENCE START ---
            case 30:
                prepareShot(); // Keep updating target while spinning up
                // Wait for Outtake
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(31);
                }
                break;
            case 31:
                prepareShot(); // Keep updating target while firing
                // Rapid Fire
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    follower.followPath(leave, true);
                    setPathState(7);
                }
                break;
            // --- THIRD SHOOTING SEQUENCE END ---

            case 7:
                // Traveling to leave
                if (!follower.isBusy()) {
                    setPathState(8); // Done
                }
                break;
            case 8:
                // End of auto
                break;
        }
    }

    private void prepareShot() {
        // Updated to use Odometry distance like TeleOp for better reliability
        double distance = turret.getOdometryDistance();
        outtake.updateShooterFromDistance(distance);
    }

    private void startRapidFire() {
        rapidFire.start();
        pathTimer.resetTimer();
    }

    private boolean checkRapidFireDone() {
        double elapsedTime = pathTimer.getElapsedTimeSeconds();
        // Fallback timeout
        if (elapsedTime > TIME_SHOOT_SEQUENCE)
            return true;

        // Velocity check - only after minimum fire time
        if (elapsedTime > MIN_FIRE_TIME) {
            // If outtake is ready (velocity recovered), we are done
            return outtake.isReady(150);
        }
        return false;
    }

    private void stopRapidFire() {
        rapidFire.stop();
        outtake.stop();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize subsystems
        outtake = new Outtake(hardwareMap);
        spindexer = new Spindexer(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, spindexer);
        turret = new Turret(hardwareMap, telemetry);
        rapidFire = new RapidFireCommand(outtake, spindexer, intake);

        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update(); // Update PID
        spindexer.update(); // Update info
        intake.update(); // Update intake
        turret.loop(Turret.TurretMode.POSE_LOCK, follower.getPose(), true); // Update Turret (Red Alliance)
        rapidFire.update(); // Update RapidFire logic

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter Vel", outtake.getLansator1Velocity());
        telemetry.addData("Spindexer Angle", spindexer.getAngleDeg());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            PoseStorage.currentPose = follower.getPose();
        }
    }
}