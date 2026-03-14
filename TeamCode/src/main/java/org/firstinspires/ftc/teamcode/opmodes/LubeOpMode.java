package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;

@TeleOp(name = "Lube OpMode", group = "test")
@Configurable
public class LubeOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors using exact hardware map names from Subsystems
        DcMotorEx lansator1 = hardwareMap.get(DcMotorEx.class, "lansator1");
        DcMotorEx lansator2 = hardwareMap.get(DcMotorEx.class, "lansator2");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Use built-in SDK velocity control
        lansator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lansator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lansator1.setDirection(DcMotorSimple.Direction.FORWARD);
        lansator2.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {


            lansator1.setPower(0.025);
            lansator2.setPower(0.025);
            intake.setPower(0.005);

        }
    }
}
