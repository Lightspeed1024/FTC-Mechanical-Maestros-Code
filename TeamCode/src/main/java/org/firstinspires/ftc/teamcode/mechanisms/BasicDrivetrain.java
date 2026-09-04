package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BasicDrivetrain {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    public enum Motor {LEFT_MOTOR, RIGHT_MOTOR}
    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;     // Rev HD Hex motor with 20:1 planetary gearbox
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331;  // traction wheels (90mm)
    static final double     TRACK_WIDTH_INCHES      = 16.0;     // Customize to our robot
    static final double     TURN_CIRCUMFERENCE      = Math.PI * TRACK_WIDTH_INCHES;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public void init(LinearOpMode opMode, HardwareMap hwMap) {
        this.opMode = opMode;
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  If turning is desired, use the turnDegrees method instead.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     * @param speed       The speed to drive at.
     * @param leftInches  The distance for the left side to drive. Set negative for reverse movement.
     * @param rightInches The distance for the right side to drive. Set negative for reverse movement.
     * @param timeoutS    The time to complete the action.
     *                    If the action has not been done when the timer runs out, movement stops.
     */
    public void driveInches(double speed,
                            double leftInches, double rightInches,
                            double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Currently at", " at %7d :%7d",
                        leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            opMode.sleep(250);   // optional pause after each move.
        }
    }

    /**
     * Method to perform a relative turn using degrees. Positive is clockwise.
     * It converts degrees to the appropriate turn and calls driveInches using those measurements.
     * @param degrees   The degrees to turn the robot.
     * @param speed     The turning speed.
     * @param timeoutS  The amount of time after to stop movement even if action is unfinished.
     */
    public void turnDegrees(double speed, double degrees, double timeoutS) {
        double inches = (degrees / 360.0) * TURN_CIRCUMFERENCE;
        driveInches(speed, inches, -inches, timeoutS);
    }

    public void setMotorSpeed(Motor motor, double speed) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setPower(speed); break;
            case RIGHT_MOTOR: rightMotor.setPower(speed); break;
        }
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftMotor.getCurrentPosition();
            case RIGHT_MOTOR: return rightMotor.getCurrentPosition();
        }
        return 0;
    }

    public void setTargetPosition(Motor motor, int target) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setTargetPosition(target); break;
            case RIGHT_MOTOR: rightMotor.setTargetPosition(target); break;
        }
    }

    public void setPower(Motor motor, double power) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setPower(power); break;
            case RIGHT_MOTOR: rightMotor.setPower(power); break;
        }
    }

    public void setMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setMode(mode); break;
            case RIGHT_MOTOR: rightMotor.setMode(mode); break;
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftMotor.isBusy();
            case RIGHT_MOTOR: return rightMotor.isBusy();
        }
        return false;
    }
}
