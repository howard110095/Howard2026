package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    public DcMotor shooterU,shooterD;

    public CRServo shooterSpinner1 , shooterSpinner2;
    public AnalogInput aimAnalogInput;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterSpinner1 = hardwareMap.get(CRServo.class, "aimX1");
        shooterSpinner2 = hardwareMap.get(CRServo.class, "aimX2");
        aimAnalogInput = hardwareMap.get(AnalogInput.class, "aimAnalogInput");

        shooterU = hardwareMap.get(DcMotor.class, "shooterU");
        shooterD = hardwareMap.get(DcMotor.class, "shooterD");

        shooterU.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterU.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterU.setPower(0);
        shooterD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setPower(0);
    }

    public void onShoot(){
        shooterU.setPower(1);
        shooterD.setPower(1);
    }

    public void off(){
        shooterU.setPower(0);
        shooterD.setPower(0);
    }

    public void onTurn(double power) {
        shooterSpinner1.setPower(power);
        shooterSpinner2.setPower(power);
    }
    public double getPose() {
        return (double) aimAnalogInput.getVoltage() / aimAnalogInput.getMaxVoltage() * 360.0;
    }
}
