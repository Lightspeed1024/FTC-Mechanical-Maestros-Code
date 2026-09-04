package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;

@TeleOp(name = "Basic TeleOp", group = "Drive")
public class BasicTeleOp extends LinearOpMode {
    private final BasicDrivetrain drivetrain = new BasicDrivetrain();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double normalSpeed = 1.0;
    private static final double slowSpeed = 0.35;
    private static final double deadZone = 0.06;

    private static final double turnSpeed = 0.80;
    private static final double fastTurnSpeed = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap);

        telemetry.addLine("Robot is ready");
        telemetry.addLine("Hold left bumper for slow mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            drivetrain.stop();
            return;
        }

        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                // Prevent loop lag from causing a sudden power change.
                double loopTime = Math.min(loopTimer.seconds(), 0.10);
                loopTimer.reset();

                // FTC returns negative Y when the left stick is pushed forward.
                double drive = fixJoystick(-gamepad1.left_stick_y);
                double turn = fixJoystick(gamepad1.right_stick_x);

                // Cubing gives more precise control while preserving direction.
                drive = drive * drive * drive;
                turn = turn * turn * turn;

                boolean slowMode = gamepad1.left_bumper;
                double speedLimit = slowMode ? slowSpeed : normalSpeed;

                // Turning becomes less sensitive while driving quickly.
                double turnLimit = mix(turnSpeed, fastTurnSpeed, Math.abs(drive));
                turn *= turnLimit;

                double wantedLeftPower = drive + turn;
                double wantedRightPower = drive - turn;

                // Keep both powers in range without changing their relative ratio.
                double biggestPower = Math.max(
                        Math.abs(wantedLeftPower),
                        Math.abs(wantedRightPower)
                );

                if (biggestPower > 1.0) {
                    wantedLeftPower /= biggestPower;
                    wantedRightPower /= biggestPower;
                }

                wantedLeftPower *= speedLimit;
                wantedRightPower *= speedLimit;

                drivetrain.setSmoothDrivePower(
                        wantedLeftPower,
                        wantedRightPower,
                        loopTime
                );

                telemetry.addData("Drive Mode", slowMode ? "SLOW" : "NORMAL");

                telemetry.addData(
                        "Power",
                        "Left: %.2f  Right: %.2f",
                        drivetrain.getLeftPower(),
                        drivetrain.getRightPower()
                );

                telemetry.addData(
                        "Encoders",
                        "Left: %d  Right: %d",
                        drivetrain.getLeftTicks(),
                        drivetrain.getRightTicks()
                );

                telemetry.update();
                idle();
            }
        } finally {
            drivetrain.stop();
        }
    }

    private double fixJoystick(double stickValue) {
        double amount = Math.abs(stickValue);

        if (amount <= deadZone) {
            return 0.0;
        }

        double fixedAmount = (amount - deadZone) / (1.0 - deadZone);
        return Math.copySign(fixedAmount, stickValue);
    }

    /**
     * Returns a value between start and end based on amount.
     *
     * An amount of 0 returns start, 1 returns end, and 0.5 returns
     * the value halfway between them.
     */
    private double mix(double start, double end, double amount) {
        amount = Range.clip(amount, 0.0, 1.0);
        return start + (end - start) * amount;
    }
}
