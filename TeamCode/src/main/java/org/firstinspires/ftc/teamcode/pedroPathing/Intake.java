package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public DcMotorEx intake;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
    }

    public void on() {
        intake.setPower(-1);
    }
    public void off() {
        intake.setPower(0);
    }
    public void out() {
        intake.setPower(1);
    }
}
