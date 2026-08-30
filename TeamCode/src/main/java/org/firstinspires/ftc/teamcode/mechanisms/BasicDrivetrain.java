package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicDrivetrain {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    public enum Motor {LEFT_MOTOR, RIGHT_MOTOR}

    public void init(HardwareMap hwMap) {
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setMotorSpeed(Motor motor, double speed) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setPower(speed);
            case RIGHT_MOTOR: rightMotor.setPower(speed);
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
            case LEFT_MOTOR: leftMotor.setTargetPosition(target);
            case RIGHT_MOTOR: rightMotor.setTargetPosition(target);
        }
    }

    public void setPower(Motor motor, double power) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setPower(power);
            case RIGHT_MOTOR: rightMotor.setPower(power);
        }
    }

    public void setMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case LEFT_MOTOR: leftMotor.setMode(mode);
            case RIGHT_MOTOR: rightMotor.setMode(mode);
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftMotor.isBusy();
            case RIGHT_MOTOR: return rightMotor.isBusy();
        }
    }
}
