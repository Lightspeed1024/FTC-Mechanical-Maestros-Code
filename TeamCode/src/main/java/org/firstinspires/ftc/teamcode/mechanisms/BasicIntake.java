package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicIntake {
    private DcMotor intake;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void spinIntake(double speed) {
        intake.setPower(speed);
    }

    public double getSpeed() {
        return intake.getPower();
    }
}