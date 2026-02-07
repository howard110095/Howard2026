package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class ColorSpinner {
    public double delta;
    public static double p = 0, i = 0, d = 0, limit = 0.8;
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

    public void on() {
        spin.setPower(1);
    }

    public void shooting() {
        spin.setPower(0.15);
    }
    public void classify() {
        spin.setPower(0.2);
    }

    public void slowMode() {
        spin.setPower(0.18);
    }

    public void out() {
        spin.setPower(-1);
    }

    public double getPose() {
        return (double) spinAnalogInput.getVoltage() / spinAnalogInput.getMaxVoltage() * 360.0;
    }

    public double getDegree() {
        double pose = (getPose() - 226.5) * 360.0 / 350.4;
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

    public void detect3posePRO(double close) {
        turnToAngle(close);
        if (close == Place1) {
            if (Math.abs(getDegree() - Place1) < 30) {
                //place 1
                if (place1 == 0) place1 = detectColor(color1, color2);
                //place 2
                if (place2 == 0) place2 = detectColor(color3, color4);
                //place 3
                if (place3 == 0) place3 = detectColor(color5, color6);
            }
        } else if (close == Place2) {
            if (Math.abs(getDegree() - Place2) < 30) {

                if (place2 == 0) place2 = detectColor(color1, color2);

                if (place3 == 0) place3 = detectColor(color3, color4);

                if (place1 == 0) place1 = detectColor(color5, color6);
            }
        } else {
            if (Math.abs(getDegree() - Place3) < 30) {

                if (place3 == 0) place3 = detectColor(color1, color2);

                if (place1 == 0) place1 = detectColor(color3, color4);

                if (place2 == 0) place2 = detectColor(color5, color6);
            }
        }

    }

    public double close(double target) {
        if (Place2 <= target && target <= Place1) return Place2;
        else if (Place3 <= target && target <= Place2) return Place3;
        else return Place1;
    }

    public void logic(int input) {
        if (place1 + place2 + place3 != 14) {
            if (place1 + place2 + place3 == 9 || place1 + place2 + place3 == 10) {
                if (place1 == 0) place1 = 14 - place2 - place3;
                else if (place2 == 0) place2 = 14 - place1 - place3;
                else place3 = 14 - place1 - place2;
            } else if (place1 == 4 || place2 == 4 || place3 == 4) {
                place1 = (place1 == 4) ? 4 : 5;
                place2 = (place2 == 4) ? 4 : 5;
                place3 = (place3 == 4) ? 4 : 5;
            } else if (place1 == 5) {
                place2 = 5;
                place3 = 4;
            } else if (place2 == 5) {
                place3 = 5;
                place1 = 4;
            } else if (place3 == 5) {
                place1 = 5;
                place2 = 4;
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

//        double currentAngle = getDegree(); // 0~360
//        delta = targetAngle - currentAngle;
//        if (delta > 0) delta -= 360;
        ColorSpinnerPID.setPID(0.5, 0, 0.3);
        double power = ColorSpinnerPID.calculate(0, -delta / 180.0);
        power = clamp(power, -1, 1);
        spin.setPower(power);
    }
}
