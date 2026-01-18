package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

// SERVOS
// spindexer1, spindexer2
// azimuthServo, elevationServo

// ANALOG INPUT ENCODERS
// spindexer1Encoder, spindexer2Encoder
// azimuthEncoder, elevationEncoder

@TeleOp(name = "Encoder Test", group = "test")
public class EncoderTester extends LinearOpMode {

    private CRServo spindexer1, spindexer2;
    private Servo azimuthServo;
    private CRServo flServo, frServo, blServo, brServo;
    private AnalogInput spindexer1Encoder, spindexer2Encoder, azimuthEncoder;
    private AnalogInput flEncoder, frEncoder, blEncoder, brEncoder;

    private HardwareDevice[] servos;
    private AnalogInput[] encoders;
    private String[] names;
    private double[] maxRecordedVoltages;
    private double[] minRecordedVoltages;
    private int currentIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        spindexer1 = hardwareMap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwareMap.get(CRServo.class, "spindexer2");
        azimuthServo = hardwareMap.get(Servo.class, "azimuthServo");
        flServo = hardwareMap.get(CRServo.class, "flServo");
        frServo = hardwareMap.get(CRServo.class, "frServo");
        blServo = hardwareMap.get(CRServo.class, "blServo");
        brServo = hardwareMap.get(CRServo.class, "brServo");

        spindexer1Encoder = hardwareMap.get(AnalogInput.class, "spindexer1Encoder");
        spindexer2Encoder = hardwareMap.get(AnalogInput.class, "spindexer2Encoder");
        azimuthEncoder = hardwareMap.get(AnalogInput.class, "azimuthEncoder");
        flEncoder = hardwareMap.get(AnalogInput.class, "flEncoder");
        frEncoder = hardwareMap.get(AnalogInput.class, "frEncoder");
        blEncoder = hardwareMap.get(AnalogInput.class, "blEncoder");
        brEncoder = hardwareMap.get(AnalogInput.class, "brEncoder");

        // Group them for easy iteration
        servos = new HardwareDevice[]{spindexer1, spindexer2, azimuthServo, flServo, frServo, blServo, brServo};
        encoders = new AnalogInput[]{spindexer1Encoder, spindexer2Encoder, azimuthEncoder, flEncoder, frEncoder, blEncoder, brEncoder};
        names = new String[]{"Spindexer 1", "Spindexer 2", "Azimuth", "FL Swerve", "FR Swerve", "BL Swerve", "BR Swerve"};
        
        maxRecordedVoltages = new double[encoders.length];
        minRecordedVoltages = new double[encoders.length];
        for (int i = 0; i < encoders.length; i++) {
            maxRecordedVoltages[i] = 0.0;
            minRecordedVoltages[i] = 3.3; // Default high value to find minimum
        }

        telemetry.addLine("Ready to start");
        telemetry.addLine("Use D-Pad Up/Down to select servo");
        telemetry.addLine("Use Triggers to apply power");
        telemetry.update();

        waitForStart();

        boolean lastUp = false;
        boolean lastDown = false;

        while (opModeIsActive()) {
            // Handle selection
            boolean currentUp = gamepad1.dpad_up;
            boolean currentDown = gamepad1.dpad_down;

            if (currentUp && !lastUp) {
                currentIndex = (currentIndex + 1) % servos.length;
            } else if (currentDown && !lastDown) {
                currentIndex--;
                if (currentIndex < 0) currentIndex = servos.length - 1;
            }

            lastUp = currentUp;
            lastDown = currentDown;

            // Control power
            double power = gamepad1.right_trigger - gamepad1.left_trigger;

            // Stop all servos first to ensure only one moves
            for (HardwareDevice servo : servos) {
                if (servo instanceof CRServo) ((CRServo) servo).setPower(0);
                else ((Servo) servo).setPosition(0.5); // 0.5 is stopped/center
            }

            // Apply power or position to selected servo
            if (servos[currentIndex] instanceof CRServo) {
                ((CRServo) servos[currentIndex]).setPower(power);
            } else {
                ((Servo) servos[currentIndex]).setPosition(0.5 + (power / 2.0));
            }

            // Update Max and Min Recorded Voltage
            for(int i=0; i<encoders.length; i++) {
                double v = encoders[i].getVoltage();
                if (v > maxRecordedVoltages[i]) {
                    maxRecordedVoltages[i] = v;
                }
                if (v < minRecordedVoltages[i]) {
                    minRecordedVoltages[i] = v;
                }
            }

            // Telemetry
            telemetry.addData("Selected", names[currentIndex]);
            telemetry.addData("Power Applied", "%.2f", power);
            telemetry.addData("Encoder Voltage", "%.3f", encoders[currentIndex].getVoltage());
            // Max voltage is usually 3.3V for the control hub analog ports
            telemetry.addData("Encoder Max Voltage (Hardware)", "%.3f", encoders[currentIndex].getMaxVoltage());
            telemetry.addData("Max Recorded Voltage", "%.3f", maxRecordedVoltages[currentIndex]);
            telemetry.addData("Min Recorded Voltage", "%.3f", minRecordedVoltages[currentIndex]);

            telemetry.addLine("\n--- All Encoder Values ---");
            for(int i=0; i<names.length; i++) {
                 telemetry.addData(names[i], "Curr: %.3f V | Min: %.3f V | Max: %.3f V", encoders[i].getVoltage(), minRecordedVoltages[i], maxRecordedVoltages[i]);
            }

            telemetry.update();
        }
    }
}
