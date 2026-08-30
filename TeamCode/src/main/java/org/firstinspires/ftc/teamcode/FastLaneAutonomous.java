package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;

@Autonomous
public class FastLaneAutonomous extends LinearOpMode {
    BasicDrivetrain drivetrain = new BasicDrivetrain();
    private final BasicDrivetrain.Motor leftMotor = BasicDrivetrain.Motor.LEFT_MOTOR;
    private final BasicDrivetrain.Motor rightMotor = BasicDrivetrain.Motor.RIGHT_MOTOR;

    static final double     DRIVE_SPEED             = 1.0;      // Full speed ahead!
    static final double     TURN_SPEED              = 0.7;      // Slightly slower turning for precision but faster than TeleOp

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                drivetrain.getCurrentPosition(leftMotor),
                drivetrain.getCurrentPosition(rightMotor));
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        if (isStopRequested()) return;

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        drivetrain.driveInches(DRIVE_SPEED, 24, 24, 5.0);
        drivetrain.turnDegrees(TURN_SPEED, -90, 4.0);
        drivetrain.driveInches(DRIVE_SPEED, 18, 18, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);  // pause to display final telemetry message.
    }
}
