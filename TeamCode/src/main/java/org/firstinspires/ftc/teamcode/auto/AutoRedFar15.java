package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commands.RapidFireCommand;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "Auto Red Far 15", group = "auto")
@Configurable
public class AutoRedFar15 extends OpMode {

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
    public static double TIME_SHOOT_SEQUENCE = 1.6;
    public static double MIN_FIRE_TIME = 1.3;
    public static double timeEnableOuttake = 0.1;

    // Power Modifiers
    public double path2Power = 1; // Controls the maximum power allowed for Path 2

    // Path Timers
    public static double path1Timer = 1.6;
    public static double path2Timer = 0.8;
    public static double path3Timer = 1.0;
    public static double path4Timer = 2.0;
    public static double path5Timer = 1.5;
    public static double path6Timer = 1.0;
    public static double path7Timer = 2.0;
    public static double path8Timer = 1.8;
    public static double path9Timer = 0.8;
    public static double path10Timer = 1.0;
    public static double path11Timer = 2.0;

    public static double path12Timer = 1.8;
    public static double path13Timer = 0.8;
    public static double path14Timer = 1.0;
    public static double path15Timer = 2.0;
    public static double path16Timer = 3.0;

    // Poses
    private final Pose startPose = new Pose(103.000, 9.000, Math.toRadians(0));
    
    private final Pose path1ControlPose = new Pose(116.000, 16.000);
    private final Pose path1EndPose = new Pose(133.000, 23.000, Math.toRadians(-90));
    
    private final Pose path2EndPose = new Pose(135.000, 10.000, Math.toRadians(-90));
    
    private final Pose path4EndPose = new Pose(126.000, 11.000, Math.toRadians(-90));
    
    private final Pose path3ControlPose = new Pose(108.000, 16.000);
    // Setting end heading slightly off -270 (-269.9) to prevent CCW wrapping
    private final Pose path3EndPose = new Pose(89.000, 16.000, Math.toRadians(-269.9));
    
    private final Pose path5ControlPose = new Pose(94.000, 29.000);
    private final Pose path5EndPose = new Pose(98.000, 35.000, Math.toRadians(0));
    
    private final Pose path6EndPose = new Pose(137.000, 35.000, Math.toRadians(0));

    private final Pose path8ControlPose = new Pose(114.000, 28.000);
    private final Pose path8EndPose = new Pose(133.000, 26.000, Math.toRadians(-89));
    
    private final Pose path9EndPose = new Pose(135.000, 12.000, Math.toRadians(-90));
    
    private final Pose path10EndPose = new Pose(126.000, 12.000, Math.toRadians(-90));
    
    private final Pose path11EndPose = new Pose(89.000, 16.000, Math.toRadians(-269.9));


    private final Pose path16EndPose = new Pose(133.000, 18.000, Math.toRadians(90));

    // Paths
    public PathChain path1;
    public PathChain path2;
    public PathChain path4;
    public PathChain path3;
    public PathChain path5;
    public PathChain path6;
    public PathChain path7;
    public PathChain path8;
    public PathChain path9;
    public PathChain path10;
    public PathChain path11;
    public PathChain path12;
    public PathChain path13;
    public PathChain path14;
    public PathChain path15;
    public PathChain path16;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                path1ControlPose,
                                path1EndPose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), path1EndPose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path1EndPose.getX(), path1EndPose.getY()),
                                new Pose(path2EndPose.getX(), path2EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path1EndPose.getHeading(), path2EndPose.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path2EndPose.getX(), path2EndPose.getY()),
                                new Pose(path4EndPose.getX(), path4EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path2EndPose.getHeading(), path4EndPose.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(path4EndPose.getX(), path4EndPose.getY()),
                                path3ControlPose,
                                new Pose(path3EndPose.getX(), path3EndPose.getY())
                        )
                )
                // When going backwards to the left side, we want the nose to swing CW down to -270 (-90 back degree view)
                // Using linear heading interpolation to the new -269.9 degree heading to force CW rotation
                .setLinearHeadingInterpolation(path4EndPose.getHeading(), path3EndPose.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(path3EndPose.getX(), path3EndPose.getY()),
                                path5ControlPose,
                                new Pose(path5EndPose.getX(), path5EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-270), path5EndPose.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path5EndPose.getX(), path5EndPose.getY()),
                                new Pose(path6EndPose.getX(), path6EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path5EndPose.getHeading(), path6EndPose.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path6EndPose.getX(), path6EndPose.getY()),
                                new Pose(path3EndPose.getX(), path3EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path6EndPose.getHeading(), Math.toRadians(90))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(path3EndPose.getX(), path3EndPose.getY()),
                                path8ControlPose,
                                new Pose(path8EndPose.getX(), path8EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), path8EndPose.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path8EndPose.getX(), path8EndPose.getY()),
                                new Pose(path9EndPose.getX(), path9EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path8EndPose.getHeading(), path9EndPose.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path9EndPose.getX(), path9EndPose.getY()),
                                new Pose(path10EndPose.getX(), path10EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path9EndPose.getHeading(), path10EndPose.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path10EndPose.getX(), path10EndPose.getY()),
                                new Pose(path11EndPose.getX(), path11EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path10EndPose.getHeading(), path11EndPose.getHeading())
                .build();

        path12 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(path11EndPose.getX(), path11EndPose.getY()),
                                path8ControlPose,
                                new Pose(path8EndPose.getX(), path8EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), path8EndPose.getHeading())
                .build();

        path13 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path8EndPose.getX(), path8EndPose.getY()),
                                new Pose(path9EndPose.getX(), path9EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path8EndPose.getHeading(), path9EndPose.getHeading())
                .build();

        path14 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path9EndPose.getX(), path9EndPose.getY()),
                                new Pose(path10EndPose.getX(), path10EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path9EndPose.getHeading(), path10EndPose.getHeading())
                .build();

        path15 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path10EndPose.getX(), path10EndPose.getY()),
                                new Pose(path11EndPose.getX(), path11EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path10EndPose.getHeading(), path11EndPose.getHeading())
                .build();

        path16 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(path11EndPose.getX(), path11EndPose.getY()),
                                new Pose(path16EndPose.getX(), path16EndPose.getY())
                        )
                )
                .setLinearHeadingInterpolation(path11EndPose.getHeading(), path16EndPose.getHeading())
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
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) || pathTimer.getElapsedTimeSeconds() > path1Timer) {
                    follower.setMaxPower(path2Power); // Reduce speed for Path 2 using Pedro's native limit
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path2Timer) {
                    follower.setMaxPower(1.0); // Reset speed to default full power
                    follower.followPath(path4, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path4Timer) {
                    follower.followPath(path3, true);
                    setPathState(4);
                }
                break;
            case 4:
                prepareShot(); // Pre-spool the outtake to save time
                // Minor delay to ensure path is finished before transitioning to shooting
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) || pathTimer.getElapsedTimeSeconds() > path3Timer) {
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
                    
                    // After the first sequence is shot, move out to collect spikemark balls
                    intake.intakeSmart(); 
                    follower.followPath(path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path5Timer) {
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path6Timer) {
                    follower.followPath(path7, true);
                    setPathState(7);
                }
                break;
            case 7:
                prepareShot(); // Pre-spool the outtake to save time
                // Minor delay to ensure path is finished before transitioning to shooting
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) || pathTimer.getElapsedTimeSeconds() > path7Timer) {
                    setPathState(30); 
                }
                break;

            // --- SHOOTING SEQUENCE 3 (Spikemark Balls) ---
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
                    
                    // After spikemark balls are shot, head back to the wall a second time
                    intake.intakeSmart(); 
                    follower.followPath(path8, true);
                    setPathState(8);
                }
                break;

            // --- SECOND WALL COLLECTION SEQUENCE ---
            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path8Timer) {
                    follower.followPath(path9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path9Timer) {
                    follower.followPath(path10, true);
                    setPathState(100);
                }
                break;
            case 100:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path10Timer) {
                    follower.followPath(path11, true);
                    setPathState(110);
                }
                break;
            case 110:
                prepareShot(); // Pre-spool the outtake to save time
                // Minor delay to ensure path is finished before transitioning to shooting
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) || pathTimer.getElapsedTimeSeconds() > path11Timer) {
                    setPathState(40); 
                }
                break;

            // --- SHOOTING SEQUENCE 4 (Second Gathered Wall Balls) ---
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
                    
                    // After the second wall balls are shot, head back to the wall a third time
                    intake.intakeSmart();
                    follower.followPath(path12, true);
                    setPathState(12);
                }
                break;

            // --- THIRD WALL COLLECTION SEQUENCE ---
            case 12:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path12Timer) {
                    follower.followPath(path13, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path13Timer) {
                    follower.followPath(path14, true);
                    setPathState(140);
                }
                break;
            case 140:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path14Timer) {
                    follower.followPath(path15, true);
                    setPathState(150);
                }
                break;
            case 150:
                prepareShot(); // Pre-spool the outtake to save time
                // Minor delay to ensure path is finished before transitioning to shooting
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.1) || pathTimer.getElapsedTimeSeconds() > path15Timer) {
                    setPathState(50); 
                }
                break;

            // --- SHOOTING SEQUENCE 5 (Third Gathered Wall Balls) ---
            case 50:
                prepareShot();
                if (pathTimer.getElapsedTimeSeconds() > timeEnableOuttake && outtake.isReady(150)) {
                    startRapidFire();
                    setPathState(51);
                }
                break;
            case 51:
                prepareShot();
                if (checkRapidFireDone()) {
                    stopRapidFire();
                    
                    // Head to park
                    follower.followPath(path16, true);
                    setPathState(16);
                }
                break;

            // --- PARKING ---
            case 16:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > path16Timer) {
                    setPathState(-1);
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

    private void stopRapidFire() {
        rapidFire.stop();
        outtake.stop();
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
        intake.intakeSmart(); // Keeps intake running the entire match
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

    @Override
    public void stop() {
        if (follower != null) {
            PoseStorage.currentPose = follower.getPose();
        }
    }
}
