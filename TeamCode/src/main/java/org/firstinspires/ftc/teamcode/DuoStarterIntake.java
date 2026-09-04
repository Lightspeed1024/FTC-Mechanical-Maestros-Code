package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.BasicIntake;

@TeleOp(name = "Duo Starter Intake", group = "TeleOp")
public class DuoStarterIntake extends OpMode {

    BasicIntake intake = new BasicIntake();

    @Override
    public void init() {
        intake.init(hardwareMap);

        telemetry.addLine("Duo Starter Intake Ready");
        telemetry.addLine("Right Bumper  = Intake");
        telemetry.addLine("Left Bumper = Outtake");
        telemetry.update();
    }

    @Override
    public void loop() {

        // A button = Intake
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {

            intake.spinIntake(1.0);

            telemetry.addLine("INTAKING");

            // B button = Outtake
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {

            intake.spinIntake(-1.0);

            telemetry.addLine("OUTTAKING");

            // No button, or both buttons = Stop
        } else {

            intake.spinIntake(0.0);

            telemetry.addLine("STOPPED");
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        // Make sure the intake is off when TeleOp stops
        intake.spinIntake(0.0);
    }
}