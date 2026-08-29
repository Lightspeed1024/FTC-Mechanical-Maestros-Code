package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;

@TeleOp
public class BasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Declare motors
        BasicDrivetrain drivetrain = new BasicDrivetrain();


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
            drivetrain.setLeftMotorSpeed(leftPower);
            drivetrain.setRightMotorSpeed(rightPower);

            // Display motor power on the Driver Station
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}