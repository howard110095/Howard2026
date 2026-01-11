package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public DcMotor intake;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
    }

    public void on() {
        intake.setPower(-1);
    }
    public void off() {
        intake.setPower(0);
    }
    public void out() {
        intake.setPower(-1);
    }
}
