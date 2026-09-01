package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Duo Starter Intake", group = "TeleOp")
public class DuoStarterIntake extends OpMode {

    private DcMotor intake1;
    private DcMotor intake2;

    @Override
    public void init() {

        // Connect the two intake motors
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        // The motors are mounted opposite each other
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        // Stop the motors when power is 0
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Duo Starter Intake Ready");
        telemetry.addLine("Right Bumper  = Intake");
        telemetry.addLine("Left Bumper = Outtake");
        telemetry.update();
    }

    @Override
    public void loop() {

        // A button = Intake
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {

            intake1.setPower(1.0);
            intake2.setPower(1.0);

            telemetry.addLine("INTAKING");

            // B button = Outtake
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {

            intake1.setPower(-1.0);
            intake2.setPower(-1.0);

            telemetry.addLine("OUTTAKING");

            // No button, or both buttons = Stop
        } else {

            intake1.setPower(0);
            intake2.setPower(0);

            telemetry.addLine("STOPPED");
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        // Make sure the intake is off when TeleOp stops
        intake1.setPower(0);
        intake2.setPower(0);
    }
}