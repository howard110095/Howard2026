package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Foot {
    public Servo leftFoot, rightFoot;

    public Foot(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFoot = hardwareMap.get(Servo.class, "leftfoot");
        rightFoot = hardwareMap.get(Servo.class, "rightfoot");
    }

    public void robotUp() {
        leftFoot.setPosition(0.01);
        rightFoot.setPosition(0.01);
    }

    public void robotDown() {
        rightFoot.setPosition(0.99);
        leftFoot.setPosition(0.99);
    }
}



