package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicDrivetrain {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public void init(HardwareMap hwMap) {
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLeftMotorSpeed(double speed) {
        leftMotor.setPower(speed);
    }

    public void setRightMotorSpeed(double speed) {
        rightMotor.setPower(speed);
    }

    public double getRightCurrentTicks() {
        return rightMotor.getCurrentPosition();
    }

    public double getLeftCurrentTicks() {
        return leftMotor.getCurrentPosition();
    }
}
