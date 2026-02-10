package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class Limelight {

    public enum Motif {
        NONE, GPP, PGP, PPG
    }

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static int BLUE_TAG_ID = 20;
    public static int RED_TAG_ID = 24;

    // Obelisk IDs (Center field tags) - defined for reference, filtered out by
    // logic below
    public static int OBELISK_TAG_ID_1 = 21;
    public static int OBELISK_TAG_ID_2 = 22;
    public static int OBELISK_TAG_ID_3 = 23;

    // Camera mounting details (You MUST allow the user to tune these)
    public static double CAMERA_HEIGHT_INCHES = 10.465; // Adjust this!
    public static double TARGET_HEIGHT_INCHES = 29.0; // Adjust this!
    public static double CAMERA_PITCH_DEGREES = 15.0; // Adjust this! (Angle of camera relative to ground)

    private double tx = 0.0;
    private double ty = 0.0;
    private double resultTx = 0.0; // Pipeline-level tx (what the dashboard shows)
    private boolean targetSeen = false;
    private double distanceToTarget = 0.0;
    
    // Store latest seen motif
    private Motif currentMotif = Motif.NONE;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1); // Assuming AprilTag pipeline is 1
        limelight.start();
        limelight.setPollRateHz(100);
    }

    public void update(boolean isRedAlliance) {
        int targetId = isRedAlliance ? RED_TAG_ID : BLUE_TAG_ID;
        LLResult result = limelight.getLatestResult();

        targetSeen = false;
        tx = 0.0;
        ty = 0.0;
        resultTx = 0.0;
        distanceToTarget = 0.0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                
                // Track Obelisk Motifs
                if (id == OBELISK_TAG_ID_1) {
                    currentMotif = Motif.GPP; // Assuming 21 = GPP
                } else if (id == OBELISK_TAG_ID_2) {
                    currentMotif = Motif.PGP; // Assuming 22 = PGP
                } else if (id == OBELISK_TAG_ID_3) {
                    currentMotif = Motif.PPG; // Assuming 33 = PPG
                }

                if (id == targetId && !targetSeen) {
                    // Per-tag values
                    tx = tag.getTargetXDegreesNoCrosshair();
                    ty = tag.getTargetYDegreesNoCrosshair();
                    // Pipeline-level tx (what the dashboard shows)
                    resultTx = result.getTx();
                    targetSeen = true;

                    // Calculate Distance
                    double angleToGoalRadians = Math.toRadians(CAMERA_PITCH_DEGREES + ty);
                    double heightDiff = TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

                    if (Math.tan(angleToGoalRadians) != 0) {
                        distanceToTarget = heightDiff / Math.tan(angleToGoalRadians);
                    }
                }
            }
        }

        if (telemetry != null) {
            telemetry.addData("Limelight/Seen", targetSeen);
            // telemetry.addData("Limelight/tx", tx);
            // telemetry.addData("Limelight/ty", ty);
            telemetry.addData("Limelight/Distance", distanceToTarget);
            telemetry.addData("Limelight/Motif", currentMotif.toString());
        }

        panelsTelemetry.addData("limelight/distance", distanceToTarget);
        panelsTelemetry.update();
    }

    public boolean isTargetSeen() {
        return targetSeen;
    }

    public double getTx() {
        return tx;
    }

    /** Pipeline-level tx — the offset the limelight dashboard shows. */
    public double getResultTx() {
        return resultTx;
    }

    public double getTy() {
        return ty;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public Motif getCurrentMotif() {
        return currentMotif;
    }

    public void stop() {
        limelight.stop();
    }
}
