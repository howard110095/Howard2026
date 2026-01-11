package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Elevator {
    public DcMotor elevator;
    public Servo arm;

    // 建構子
    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        arm = hardwareMap.get(Servo.class, "arm");
        elevator = hardwareMap.get(DcMotor.class, "feed");
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(0);
    }

    // 你可以加一些方法來控制 spinner，例如：
    public void up() {
        elevator.setPower(1);
        arm.setPosition(0.4);
    }

    public void off() {
        elevator.setPower(0);
        arm.setPosition(0.7);
    }

    public void servoPose(double pose) {
        pose = clamp(pose, 0, 1);
        arm.setPosition(pose);
    }
}
