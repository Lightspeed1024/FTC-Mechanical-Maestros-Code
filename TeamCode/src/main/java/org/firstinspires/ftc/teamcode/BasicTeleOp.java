package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Declare motors
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.dcMotor.get("rightMotor");

        // Reverse the right motor
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the driver to press PLAY
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // Left stick controls forward and backward
            double drive = -gamepad1.left_stick_y;

            // Right stick controls turning
            double turn = -gamepad1.right_stick_x;

            // Calculate motor powers
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Keep powers between -1 and 1
            double max = Math.max(
                    Math.abs(leftPower),
                    Math.abs(rightPower)
            );

            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            // Set motor powers
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Display motor power on the Driver Station
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}