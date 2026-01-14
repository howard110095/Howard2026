package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSpinner {
    public double delta, aim;
    public double lastPosition = 0;
    public boolean lastChange = false;
    public ColorSensor color1, color2;
    public CRServo spin;
    public AnalogInput spinAnalogInput;

    boolean p1 = false, p2 = false, p3 = false;
    int place1 = 0, place2 = 0, place3 = 0; //none 0  green 4  purple 5

    public ColorSpinner(HardwareMap hardwareMap, Telemetry telemetry) {
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        spin = hardwareMap.get(CRServo.class, "spin");
        spinAnalogInput = hardwareMap.get(AnalogInput.class, "spinAnalogInput");
        spin.setDirection(CRServo.Direction.REVERSE);
        spin.setPower(0);
    }

    // 你可以加一些方法來控制 spinner，例如：
    public void on(double power) {
        spin.setPower(power);
    }

    public double getPose() {
        return (double) spinAnalogInput.getVoltage() / spinAnalogInput.getMaxVoltage() * 360.0;
    }

    public double getCurrentPose() {
        double pose = getPose();
        pose -= 226.5;
        pose *= 360 / 350.4;
        if (pose < 0) pose += 360;
        return pose;
    }

    public void spinToPosition(double target) {
        delta = (target - getCurrentPose() + 720.0) % 360.0;
        if (delta < 15) spin.setPower(0);
        else if (delta < 30) spin.setPower(0.3);
        else spin.setPower(0.5);
    }

    public void scanning() {
        if (lastChange) {
            delta = aim - getCurrentPose();
            if (delta < 20) {
                spin.setPower(0);
                lastChange = false;
            } else spin.setPower(0.2);
        } else {
            lastChange = true;
            aim = getCurrentPose() + 360;
        }
    }

    public void onTo() {
        spin.setPower(0.18);
        if (305 < getCurrentPose() && getCurrentPose() < 335) { //320
            place1 = detectColor();
        }
        if (185 < getPose() && getPose() < 215 && !p2) { //200
            place2 = detectColor();
        }
        if (65 < getPose() && getPose() < 95 && !p3) { //80
            place3 = detectColor();
        }

    }

    public void colorRenew() {
        p1 = false;
        p2 = false;
        p3 = false;
        place1 = 0;
        place2 = 0;
        place3 = 0;
    }

    public void check() {
        if (!(p1 && p2 && p3)) spin.setPower(0.18);
        else spin.setPower(0);
    }

    public int detectColor() {
        if (color1.alpha() > 350) {
            if (color1.blue() > color1.green()) return 5;
            else return 4;
        } else if (color2.alpha() > 350) {
            if (color2.blue() > color2.green()) return 5;
            else return 4;
        } else return 0;
    }
}
/*
    public double Rpercent() {
        return (double) CS.red() / (CS.red() + CS.green() + CS.blue());
    }

    public double Gpercent() {
        return (double) CS.green() / (CS.red() + CS.green() + CS.blue());
    }

    public double Bpercent() {
        return (double) CS.blue() / (CS.red() + CS.green() + CS.blue());
    }

    public int color_detect() {
        if (Rpercent() < 0.45 && Bpercent() < 0.2) return 2; //yellow
        else if (Rpercent() > 0.35) return 1; //red
        else if (Bpercent() > 0.42) return 3; //blue
        else return 0;
    }
*/