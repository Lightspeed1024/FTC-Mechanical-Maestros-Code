package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Duo Starter Intake", group = "TeleOp")
public class DuoStarterIntake extends OpMode {

    DcMotor intake1;
    DcMotor intake2;

    @Override
    public void init() {

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("A = Intake");
        telemetry.addLine("B = Outtake");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.a && !gamepad1.b) {

            intake1.setPower(1);
            intake2.setPower(1);

        } else if (gamepad1.b && !gamepad1.a) {

            intake1.setPower(-1);
            intake2.setPower(-1);

        } else {

            intake1.setPower(0);
            intake2.setPower(0);
        }
    }
}
