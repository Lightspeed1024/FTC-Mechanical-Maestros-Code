package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.BasicIntake;

@TeleOp
public class BasicTeleOp extends LinearOpMode {
    BasicDrivetrain drivetrain = new BasicDrivetrain();
    BasicIntake intake = new BasicIntake();
    private final BasicDrivetrain.Motor leftMotor = BasicDrivetrain.Motor.LEFT_MOTOR;
    private final BasicDrivetrain.Motor rightMotor = BasicDrivetrain.Motor.RIGHT_MOTOR;

    @Override
    public void runOpMode() throws InterruptedException{

        // Declare motors
        drivetrain.init(this, hardwareMap);

        // Wait for the driver to press PLAY
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // Left stick controls forward and backward
            double drive = -gamepad1.left_stick_y;

            // Right stick controls turning
            double turn = -gamepad1.right_stick_x * 0.5;

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
            drivetrain.setMotorSpeed(leftMotor, leftPower);
            drivetrain.setMotorSpeed(rightMotor, rightPower);

            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                intake.spinIntake(1.0);
            }
            else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                intake.spinIntake(-1.0);
            }
            else {
                intake.spinIntake(0.0);
            }

            // Display motor power on the Driver Station
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Intake Speed", intake.getSpeed());
            telemetry.addData("Left Ticks", drivetrain.getCurrentPosition(leftMotor));
            telemetry.addData("Right Ticks", drivetrain.getCurrentPosition(rightMotor));
            telemetry.update();
        }
    }
}