package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

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
           // white
        leftFoot.setPosition(0.99);
        rightFoot.setPosition(0.01);
//           // black
//        leftFoot.setPosition(0.01);
//        rightFoot.setPosition(0.01);
    }

    public void robotDown() {
         //white
        leftFoot.setPosition(0.01);
        rightFoot.setPosition(0.99);
//         //black
//        leftFoot.setPosition(0.99);
//        rightFoot.setPosition(0.99);
    }
    public void robotDefend() {
        //white
        leftFoot.setPosition(0.4);
        rightFoot.setPosition(0.6);
//         //black
//        leftFoot.setPosition(0.6);
//        rightFoot.setPosition(0.6);
    }
}



