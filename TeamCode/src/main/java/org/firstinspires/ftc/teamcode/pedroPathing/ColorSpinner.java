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
        double pose = getPose();
        pose -= 226.5;
        pose *= 360 / 350.4;
        if (pose < 0) pose += 360;
        return pose;
    }

    public void colorRenew() {
        place1 = 0;
        place2 = 0;
        place3 = 0;
    }

    public int detectColor(int place) {
        boolean detect = false;
        if (place == 1 && Math.abs(getDegree() - Place1) < 30) detect = true;
        if (place == 2 && Math.abs(getDegree() - Place2) < 30) detect = true;
        if (place == 3 && Math.abs(getDegree() - Place3) < 30) detect = true;
        if (detect) {
            if (color1.blue() + color1.green() > 1000) {
                if (color1.blue() > color1.green()) return 5;//purple
                else return 4; //green
            } else if (color2.blue() + color2.green() > 1000) {
                if (color2.blue() > color2.green()) return 5;
                else return 4;
            } else return 0;
        } else return 0;
    }

    public void detect3pose() {
        if (detectColor(1) != 0 || place1 != 0) {
            if (place1 == 0) place1 = detectColor(1);
            if (detectColor(2) != 0 || place2 != 0) {
                if (place2 == 0) place2 = detectColor(2);
                if (detectColor(3) != 0 || place3 != 0) {
                    if (place3 == 0) place3 = detectColor(3);
                    spin.setPower(0);
                } else turnToAngle(Place3);
            } else turnToAngle(Place2);
        } else turnToAngle(Place1);
    }

    public void detect3posePRO() {
        turnToAngle(Place1);
        if (Math.abs(getDegree() - Place1) < 30) {
            //place 1
            if (color1.blue() + color1.green() > 1000) {
                if (color1.blue() > color1.green()) place1 = 5;//purple
                else place1 = 4; //green
            } else if (color2.blue() + color2.green() > 1000) {
                if (color2.blue() > color2.green()) place1 = 5;
                else place1 = 4;
            } else place1 = 0;
            //place 2
            if (color3.blue() + color3.green() > 1000) {
                if (color3.blue() > color3.green()) place2 = 5;//purple
                else place2 = 4; //green
            } else if (color4.blue() + color4.green() > 1000) {
                if (color4.blue() > color4.green()) place2 = 5;
                else place2 = 4;
            } else place2 = 0;
            //place 3
            if (color5.blue() + color5.green() > 1000) {
                if (color5.blue() > color5.green()) place3 = 5;//purple
                else place3 = 4; //green
            } else if (color6.blue() + color6.green() > 1000) {
                if (color6.blue() > color6.green()) place3 = 5;
                else place3 = 4;
            } else place3 = 0;
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
