package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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

@Autonomous(name = "auto red", group = "auto")
@Configurable
@Disabled
public class AutoRed extends OpMode {

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
    public static double TIME_SHOOT_SEQUENCE = 4.0;
    public static double MIN_FIRE_TIME = 2.0;
    public static double timeEnableOuttake = 1.0;

    // Poses
    private final Pose preloadStartPose = new Pose(117.000, 131.000, Math.toRadians(37));

    private final Pose preloadShootPose = new Pose(95.000, 90.000, Math.toRadians(0));
    private final Pose spikeMark1Pose = new Pose(123.000, 90.000, Math.toRadians(0));
    private final Pose shoot2Pose = new Pose(90.000, 83.000, Math.toRadians(0));
    private final Pose spikeMark2Align = new Pose(98.000, 63.000, Math.toRadians(0));
    private final Pose spikeMark2Pose = new Pose(126.000, 63.000, Math.toRadians(0));
    private final Pose shoot3Pose = new Pose(90.000, 83.000, Math.toRadians(0));
    private final Pose leavePose = new Pose(105.000, 56.000, Math.toRadians(0));

    public PathChain preloadintimpcemergeinspate;
    public PathChain spikemark1;
    public PathChain shoot2;
    public PathChain spikemark2AlignPath;
    public PathChain spikemark2;
    public PathChain shoot3;
    public PathChain leave;


    public void buildPaths() {
        preloadintimpcemergeinspate = follower.pathBuilder().addPath(
                        new BezierLine(
                                preloadStartPose,
                                preloadShootPose))
                .setLinearHeadingInterpolation(preloadStartPose.getHeading(), preloadShootPose.getHeading())
                .build();

        spikemark1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                preloadShootPose,
                                spikeMark1Pose))
                .setLinearHeadingInterpolation(preloadShootPose.getHeading(), spikeMark1Pose.getHeading())
                .build();

        shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                spikeMark1Pose,
                                shoot2Pose))
                .setLinearHeadingInterpolation(spikeMark1Pose.getHeading(), shoot2Pose.getHeading())
                .build();

        spikemark2AlignPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot2Pose,
                                spikeMark2Align))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), spikeMark2Align.getHeading())
                .build();

        spikemark2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                spikeMark2Align,
                                spikeMark2Pose))
                .setLinearHeadingInterpolation(spikeMark2Align.getHeading(), spikeMark2Pose.getHeading())
                .build();

        shoot3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                spikeMark2Pose,
                                shoot3Pose))
                .setLinearHeadingInterpolation(spikeMark2Pose.getHeading(), shoot3Pose.getHeading())
                .build();

        leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot3Pose,
                                leavePose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start pathing to first shot position (Preload)
                follower.followPath(preloadintimpcemergeinspate, true);
                setPathState(1);
                break;
            case 1:
                // Traveling
                prepareShot(); // Turn outtake on while moving
                if (!follower.isBusy()) {
                    setPathState(10); // Go to Shoot logic
                }
                break;

            // --- SHOOTING SEQUENCE 1 (Preload) ---
            case 10:
                prepareShot(); // Keep updating target while spinning up
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(11);
                }
                break;
            case 11:
                prepareShot(); // Keep updating target while firing
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    setPathState(12);
                }
                break;
            case 12:
                // Immediately enable intake smart
                intake.intakeSmart();
                // Go straight to spike mark 1
                follower.followPath(spikemark1, true);
                Constants.swerveDrive.setMaxPower(0.30);
                setPathState(2);
                break;
            // --- END SEQUENCE 1 ---

            case 2:
                // Traveling to spikemark1
                if (!follower.isBusy()) {
                    // Logic: Wait for path end AND minimum time of 0.2s
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(shoot2, true);
                        Constants.swerveDrive.setMaxPower(1.0);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                // Traveling to shoot2
                prepareShot(); // Turn outtake on while moving
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(20);
                }
                break;

            // --- SHOOTING SEQUENCE 2 ---
            case 20:
                prepareShot(); // Keep updating target while spinning up
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(21);
                }
                break;
            case 21:
                prepareShot(); // Keep updating target while firing
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    // Next is spikemark2 then pickup.
                    // Follow spikemark2 first
                    follower.followPath(spikemark2AlignPath, true);
                    setPathState(22);
                }
                break;
            case 22:
                // Traveling to spikeMark2Align
                if (!follower.isBusy()) {
                    // Turn on intake here (arrived at Align, about to go to SpikeMark2)
                    intake.intakeSmart();

                    follower.followPath(spikemark2, true);
                    Constants.swerveDrive.setMaxPower(0.30);
                    setPathState(4);
                }
                break;
            // --- END SEQUENCE 2 ---

            case 4:
                // Traveling spikemark2
                if (!follower.isBusy()) {
                    // Logic mirroring spikemark1 (Case 2):
                    // Proceed if path is done AND 0.2s has passed.
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(shoot3, true);
                        Constants.swerveDrive.setMaxPower(1.0);
                        setPathState(6);
                    }
                }
                break;

            // Case 5 was the wait state, now removed/skipped.

            case 6:
                // Traveling to shoot3
                prepareShot(); // Turn outtake on while moving
                if (!follower.isBusy()) {
                    intake.stop();
                    Constants.swerveDrive.resetMaxPower();
                    setPathState(30);
                }
                break;

            // --- SHOOTING SEQUENCE 3 ---
            case 30:
                prepareShot(); // Keep updating target while spinning up
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(31);
                }
                break;
            case 31:
                prepareShot(); // Keep updating target while firing
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    follower.followPath(leave, true);
                    setPathState(7);
                }
                break;
            // --- END SEQUENCE 3 ---

            case 7:
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                // Done
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
        if (elapsedTime > TIME_SHOOT_SEQUENCE)
            return true;
        if (elapsedTime > MIN_FIRE_TIME) {
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
        follower.setStartingPose(preloadStartPose);

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
        outtake.update();
        spindexer.update();
        intake.update();
        turret.loop(Turret.TurretMode.POSE_LOCK, follower.getPose(), true); // Red Alliance
        rapidFire.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter Vel", outtake.getLansator1Velocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            PoseStorage.currentPose = follower.getPose();
        }
    }
}