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

@Autonomous(name = "auto red 15", group = "auto")
@Configurable
@Disabled
public class AutoRed15 extends OpMode {

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
    public static double timeEnableOuttake = 0.2;

    // Poses
    private final Pose preloadStartPose = new Pose(117.000, 131.000, Math.toRadians(37));
    private final Pose preloadShootPose = new Pose(93.000, 84.000, Math.toRadians(0));
    private final Pose spikeMark1Pose = new Pose(126.000, 84.000, Math.toRadians(0));
    private final Pose shoot2Pose = new Pose(93.000, 84.000, Math.toRadians(0));
    private final Pose spikeMark2Align = new Pose(98.000, 59.500, Math.toRadians(0));
    private final Pose spikeMark2Pose = new Pose(127.000, 59.500, Math.toRadians(0));
    private final Pose shoot3Pose = new Pose(93.000, 84.000, Math.toRadians(0));
    
    // Added for AutoRed15
    private final Pose opengate1Pose = new Pose(123.000, 62.000, Math.toRadians(10));
    private final Pose pickupgate1StartPose = new Pose(123.000, 58.000, Math.toRadians(0));
    private final Pose pickupgate1Pose = new Pose(132.000, 58.000, Math.toRadians(70));
    private final Pose awayfromgatePose = new Pose(125.000, 58.000, Math.toRadians(45));
    private final Pose shoot4Pose = new Pose(87.000, 103.000, Math.toRadians(45));

    // Paths
    public PathChain preloadShootPath;
    public PathChain spikeMark1Path;
    public PathChain shoot2Path;
    public PathChain spikeMark2AlignPath;
    public PathChain spikeMark2Path;
    public PathChain shoot3Path;
    public PathChain opengate1Path;
    public PathChain pickupgate1Path;
    public PathChain awayfromgatePath;
    public PathChain shoot4Path;

    public void buildPaths() {
        preloadShootPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                preloadStartPose,
                                preloadShootPose))
                .setLinearHeadingInterpolation(preloadStartPose.getHeading(), preloadShootPose.getHeading())
                .build();

        spikeMark1Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                preloadShootPose,
                                spikeMark1Pose))
                .setLinearHeadingInterpolation(preloadShootPose.getHeading(), spikeMark1Pose.getHeading())
                .build();

        shoot2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                spikeMark1Pose,
                                shoot2Pose))
                .setLinearHeadingInterpolation(spikeMark1Pose.getHeading(), shoot2Pose.getHeading())
                .build();

        spikeMark2AlignPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot2Pose,
                                spikeMark2Align))
                .setLinearHeadingInterpolation(shoot2Pose.getHeading(), spikeMark2Align.getHeading())
                .build();

        spikeMark2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                spikeMark2Align,
                                spikeMark2Pose))
                .setLinearHeadingInterpolation(spikeMark2Align.getHeading(), spikeMark2Pose.getHeading())
                .build();

        shoot3Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                spikeMark2Pose,
                                shoot3Pose))
                .setLinearHeadingInterpolation(spikeMark2Pose.getHeading(), shoot3Pose.getHeading())
                .build();

        opengate1Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                shoot3Pose,
                                opengate1Pose))
                .setLinearHeadingInterpolation(shoot3Pose.getHeading(), opengate1Pose.getHeading())
                .build();

        pickupgate1Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                pickupgate1StartPose,
                                pickupgate1Pose))
                .setLinearHeadingInterpolation(pickupgate1StartPose.getHeading(), pickupgate1Pose.getHeading())
                .build();

        awayfromgatePath = follower.pathBuilder().addPath(
                        new BezierLine(
                                pickupgate1Pose,
                                awayfromgatePose))
                .setLinearHeadingInterpolation(pickupgate1Pose.getHeading(), awayfromgatePose.getHeading())
                .build();

        shoot4Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                awayfromgatePose,
                                shoot4Pose))
                .setLinearHeadingInterpolation(awayfromgatePose.getHeading(), shoot4Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloadShootPath, true);
                setPathState(1);
                break;
            case 1:
                prepareShot();
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            // --- SHOOTING SEQUENCE 1 ---
            case 10:
                prepareShot();
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(11);
                }
                break;
            case 11:
                prepareShot();
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    setPathState(12);
                }
                break;
            case 12:
                intake.intakeSmart();
                follower.followPath(spikeMark1Path, true);
                Constants.swerveDrive.setMaxPower(1);
                setPathState(2);
                break;

            case 2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(shoot2Path, true);
                        Constants.swerveDrive.setMaxPower(1.0);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                prepareShot();
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(20);
                }
                break;

            // --- SHOOTING SEQUENCE 2 ---
            case 20:
                prepareShot();
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(21);
                }
                break;
            case 21:
                prepareShot();
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    follower.followPath(spikeMark2AlignPath, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    intake.intakeSmart();
                    follower.followPath(spikeMark2Path, true);
                    Constants.swerveDrive.setMaxPower(1);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(shoot3Path, true);
                        Constants.swerveDrive.setMaxPower(1.0);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                prepareShot();
                if (!follower.isBusy()) {
                    intake.stop();
                    Constants.swerveDrive.resetMaxPower();
                    setPathState(30);
                }
                break;

            // --- SHOOTING SEQUENCE 3 ---
            case 30:
                prepareShot();
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(31);
                }
                break;
            case 31:
                prepareShot();
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    follower.followPath(opengate1Path, true);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
                    intake.intakeSmart();
                    follower.followPath(pickupgate1Path, true);
                    Constants.swerveDrive.setMaxPower(1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(awayfromgatePath, true);
                        Constants.swerveDrive.setMaxPower(1.0);
                        setPathState(33);
                    }
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(shoot4Path, true);
                        Constants.swerveDrive.setMaxPower(1.0);
                        setPathState(34);
                    }
                }
                break;

            case 34:
                prepareShot();
                if (!follower.isBusy()) {
                    intake.stop();
                    Constants.swerveDrive.resetMaxPower();
                    setPathState(40);
                }
                break;

            // --- SHOOTING SEQUENCE 4 ---
            case 40:
                prepareShot();
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(41);
                }
                break;
            case 41:
                prepareShot();
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    setPathState(8);
                }
                break;

            case 8:
                // Done
                break;
        }
    }

    private void prepareShot() {
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