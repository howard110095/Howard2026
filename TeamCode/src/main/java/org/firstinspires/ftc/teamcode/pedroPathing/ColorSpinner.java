package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class ColorSpinner {
    public double delta;
    public ColorSensor color1, color2, color3, color4, color5, color6;
    public CRServo spin;
    public AnalogInput spinAnalogInput;
    private PIDController ColorSpinnerPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
    public int place1 = 0, place2 = 0, place3 = 0; //none 0  green 4  purple 5
    public double Place1 = 305, Place2 = 185, Place3 = 65; //angle

    public ColorSpinner(HardwareMap hardwareMap, Telemetry telemetry) {
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");
        color4 = hardwareMap.get(ColorSensor.class, "color4");
        color5 = hardwareMap.get(ColorSensor.class, "color5");
        color6 = hardwareMap.get(ColorSensor.class, "color6");
        spin = hardwareMap.get(CRServo.class, "spin");
        spinAnalogInput = hardwareMap.get(AnalogInput.class, "spinAnalogInput");
        spin.setDirection(CRServo.Direction.REVERSE);
        spin.setPower(0);
    }

    public void on(double power) {
        spin.setPower(power);
    }

    public double getPose() {
        return (double) spinAnalogInput.getVoltage() / spinAnalogInput.getMaxVoltage() * 360.0;
    }

    public double getDegree() {
        double pose = (getPose() - 226.5) * 360.0 / 350.4;
//        = getPose();
//        pose -= 226.5;
//        pose *= 360 / 350.4;
        if (pose < 0) pose += 360;
        return pose;
    }

    public void colorRenew() {
        place1 = 0;
        place2 = 0;
        place3 = 0;
    }

    private int detectColor(ColorSensor s1, ColorSensor s2) {
        if (s1.blue() + s1.green() > 1000)
            return (s1.blue() > s1.green()) ? 5 : 4;
        else if (s2.blue() + s2.green() > 1000)
            return (s2.blue() > s2.green()) ? 5 : 4;
        return 0;
    }

    public void detect3posePRO() {
        turnToAngle(Place1);
        if (Math.abs(getDegree() - Place1) < 30) {
            //place 1
            if (place1 == 0) place1 = detectColor(color1, color2);
            //place 2
            if (place2 == 0) place2 = detectColor(color3, color4);
            //place 3
            if (place3 == 0) place3 = detectColor(color5, color6);
        }
    }

    public void logic(int input) {
        if (place1 + place2 + place3 != 14) {
            if (place1 + place2 + place3 == 9 || place1 + place2 + place3 == 10) {
                if (place1 == 0) place1 = 14 - place2 - place3;
                else if (place2 == 0) place2 = 14 - place1 - place3;
                else place3 = 14 - place1 - place2;
            } else {
                place1 = 5;
                place2 = 4;
                place3 = 5;
            }
        }
        if (input == 21) {
            if (place1 == 4) turnToAngle(Place1);
            else if (place2 == 4) turnToAngle(Place2);
            else turnToAngle(Place3);
        } else if (input == 22) {
            if (place1 == 4) turnToAngle(Place3);
            else if (place2 == 4) turnToAngle(Place1);
            else turnToAngle(Place2);
        } else {
            if (place1 == 4) turnToAngle(Place2);
            else if (place2 == 4) turnToAngle(Place3);
            else turnToAngle(Place1);
        }
    }

    public void turnToAngle(double targetAngle) {
        double currentAngle = getDegree(); // 0~360
        delta = targetAngle - currentAngle;
        delta = (delta + 540) % 360 - 180;
        ColorSpinnerPID.setPID(0.007, 0, 0);
        double power = ColorSpinnerPID.calculate(0, -delta);
        power = clamp(power, -0.3, 0.3);
        spin.setPower(power);
    }
}
