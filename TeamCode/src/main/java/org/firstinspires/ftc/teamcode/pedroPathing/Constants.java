package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

public class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
                        .forwardZeroPowerAcceleration(-400)
                        .lateralZeroPowerAcceleration(-400)
                        .useSecondaryDrivePIDF(true)
                        .useSecondaryHeadingPIDF(true)
                        .useSecondaryTranslationalPIDF(true)
                        .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.009, 0))
                        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.0009, 0.01))
                        .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.018, 0.008, 0.2))
                        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.035, 0.1))
                        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0035, 0, 0.00001, 0.6, 0.13))
                        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.000005, 0.6, 0.13))
                        .mass(13.7);

        public static SwerveConstants swerveConstants = new SwerveConstants();
        public static SwerveDrive swerveDrive;

        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                        .forwardPodY(24.2)
                        .strafePodX(-180)
                        .distanceUnit(DistanceUnit.MM)
                        .hardwareMapName("pinpoint")
                        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static Follower createFollower(HardwareMap hardwareMap) {
                swerveDrive = new SwerveDrive(hardwareMap, swerveConstants);
                return new FollowerBuilder(followerConstants, hardwareMap)
                                .pathConstraints(pathConstraints)
                                .pinpointLocalizer(localizerConstants)
                                .setDrivetrain(swerveDrive)
                                .build();
        }
}