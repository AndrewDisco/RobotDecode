package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;

@Autonomous(name = "AutoTest", group = "auto")
@Configurable
@Disabled
public class AutoTest extends OpMode {

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
    private final Pose startPose = new Pose(103.000, 9.000, Math.toRadians(0));
    private final Pose path1EndPose = new Pose(127.000, 10.000, Math.toRadians(0));
    private final Pose path2EndPose = new Pose(130.100, 10.000, Math.toRadians(-50));
    private final Pose path3EndPose = new Pose(130.200, 10.000, Math.toRadians(50));
    private final Pose path4EndPose = new Pose(135.000, 10.000, Math.toRadians(0));
    private final Pose path5EndPose = new Pose(100.000, 14.000, Math.toRadians(0));

    // Paths
    public PathChain path1;
    public PathChain path2;
    public PathChain path3;
    public PathChain path4;
    public PathChain path5;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                path1EndPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                path1EndPose,
                                path2EndPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                path2EndPose,
                                path3EndPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(50))
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                path3EndPose,
                                path4EndPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                path4EndPose,
                                path5EndPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start by preparing to shoot the preload
                setPathState(10);
                break;

            // --- SHOOTING SEQUENCE 1 (Preload) ---
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
                    
                    // After the preload is shot, move out to collect balls
                    intake.intakeSmart(); // RapidFire stop turns off the intake, so turn it back on
                    follower.followPath(path1, true);
                    setPathState(1);
                }
                break;

            // --- PATHING SEQUENCE ---
            case 1:
                // Minor delay to prevent instantly skipping before follower registers as busy
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Give it half a second to complete the first rock to -50 degrees
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Give it half a second to complete the second rock to 50 degrees
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                // Short move back towards 0 heading
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                // Move back towards the line to shoot the gathered balls
                // Minor delay to prevent instantly skipping before follower registers as busy
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(20);
                }
                break;

            // --- SHOOTING SEQUENCE 2 (Gathered Balls) ---
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
                    setPathState(-1); // Finished
                }
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
        follower.setStartingPose(startPose);

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
        intake.intakeSmart(); // Always turned on and never turned off during this path
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();
        spindexer.update();
        intake.update();
        turret.loop(Turret.TurretMode.POSE_LOCK, follower.getPose(), true);
        rapidFire.update();
        
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
