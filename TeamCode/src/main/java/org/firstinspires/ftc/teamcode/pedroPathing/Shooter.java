package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotBase.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import java.util.List;

@Configurable
public class Shooter {
    public DcMotorEx shooterU, shooterD, elevator;
    public Servo arm, turretPitchL, turretPitchR;
    public CRServo shooterSpinner1, shooterSpinner2;
    public AnalogInput aimAnalogInput;
    public Limelight3A limelight;
    public PIDController SpinnerPID = new PIDController(0.02, 0, 0.02);
    public PIDController ShooterUPID = new PIDController(0, 0, 0);
    public PIDController ShooterDPID = new PIDController(0, 0, 0);
    public static boolean controlShooting = false;
    public double shooterVelocity = 2000, uVelocity, dVelocity, shooterU_power, shooterD_power;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shooterSpinner1 = hardwareMap.get(CRServo.class, "aimX1");
        shooterSpinner2 = hardwareMap.get(CRServo.class, "aimX2");
        turretPitchL = hardwareMap.get(Servo.class, "aimYL");
        turretPitchR = hardwareMap.get(Servo.class, "aimYR");
        arm = hardwareMap.get(Servo.class, "arm");
        aimAnalogInput = hardwareMap.get(AnalogInput.class, "aimAnalogInput");
        shooterU = hardwareMap.get(DcMotorEx.class, "shooterU");
        shooterD = hardwareMap.get(DcMotorEx.class, "shooterD");
        elevator = hardwareMap.get(DcMotorEx.class, "feed");

        shooterU.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterU.setPower(0);
        shooterD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterD.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setPower(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(0);

        shooterSpinner1.setPower(0);
        shooterSpinner2.setPower(0);
    }

    public double dx(int pipe) {
        if (pipe == 0) return 62.0 - follower.getPose().getX();
        else if (pipe == 1) return 0.0 - follower.getPose().getX();
        else return -62.0 - follower.getPose().getX();
    }

    public double dy(int pipe) {
        if (pipe == 1) return 72.0 - follower.getPose().getY();
        else return 62.0 - follower.getPose().getY();
    }

    public double distance(int pipe) {
        return Math.pow(dx(pipe) * dx(pipe) + dy(pipe) * dy(pipe), 0.5);
    }



//    public void setVelocity(double velocity, boolean on) {
////        velocity = clamp(velocity, 3000, 4500);
////        velocity /= 6000;
//        double velocity1 = velocity / 4600;
//        double velocity2 = velocity / 4300;
//        uVelocity = ((shooterU.getVelocity() / 28) * 60) / 4600;
//        dVelocity = ((shooterD.getVelocity() / 28) * 60) / 4300;
//        ShooterUPID.setPID(ukP, ukI, ukD); // 設置 PID
//        shooterU_power = ShooterUPID.calculate(uVelocity, velocity1); // 計算輸出
//        ShooterDPID.setPID(dkP, dkI, dkD); // 設置 PID
//        shooterD_power = ShooterDPID.calculate(dVelocity, velocity2); // 計算輸出
//        shooterU_power = clamp(shooterU_power + velocity1, -1, 1.0);
//        shooterD_power = clamp(shooterD_power + velocity2, -1, 1.0);
//        shooterU.setPower(shooterU_power);
//        shooterD.setPower(shooterD_power);
//        if (controlShooting && on) elevatorUp();
//        else if (on && Math.abs(uVelocity - velocity1) < 0.03 && Math.abs(dVelocity - velocity2) < 0.03) //正規後的範圍
//            controlShooting = true;
//        else {
//            elevatorOff();
//            controlShooting = false;
//        }
//    }

//    public void shooting(int pipe, boolean on) {
//        limelight.pipelineSwitch(pipe);
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            double y = limelight.getLatestResult().getTy();
//            if (y > -16.5) shooterVelocity = 4.5894 * Math.pow(y, 2) - 3.1401 * y + 2506.5;
//            else shooterVelocity = 4100;
//        }
////            double dx = dx(pipe), dy = dy(pipe);
////            shooterVelocity = 4100;
//        //set power
//        uVelocity = shooterU.getVelocity() / 28.0 * 60.0;
//        dVelocity = shooterD.getVelocity() / 28.0 * 60.0;
//        ShooterUPID.setPID(ukP, ukI, ukP); // 設置 PID
//        shooterU_power = ShooterUPID.calculate(uVelocity, shooterVelocity); // 計算輸出
//        ShooterDPID.setPID(dkP, dkI, dkP); // 設置 PID
//        shooterD_power = ShooterDPID.calculate(dVelocity, shooterVelocity); // 計算輸出
//        shooterU_power = clamp(shooterU_power, 0, 1);
//        shooterD_power = clamp(shooterD_power, 0, 1);
//        shooterU.setPower(shooterU_power);
//        shooterD.setPower(shooterD_power);
//        if (on && shooterVelocity - 350 < uVelocity && shooterVelocity - 350 < dVelocity)
//            elevatorUp();
//        else elevatorOff();
//    }

    public void shootingPRO(int pipeline, double targetVelocity, double turretTargetYaw, double turretTargetPitch, boolean openShooting) {
        // setting pipe line
        limelight.pipelineSwitch(pipeline);
        LLResult result = limelight.getLatestResult();

        // shooting Velocity
        toVelocity = targetVelocity;
        if (targetVelocity == 0) {  //tracking
            if (result != null && result.isValid()) { //have apriltag, velocity
                double y = limelight.getLatestResult().getTy();
                if (y > -16.5) toVelocity = 4.5894 * Math.pow(y, 2) - 3.1401 * y + 2506.5;
                else toVelocity = 4100;
            } else
                toVelocity = 30.211 * distance(pipeline) - 297.24; // no apriltag, velocity formula
        }
        double velocity1 = toVelocity / 4600;
        double velocity2 = toVelocity / 4300;
        uVelocity = ((shooterU.getVelocity() / 28) * 60) / 4600;
        dVelocity = ((shooterD.getVelocity() / 28) * 60) / 4300;
        ShooterUPID.setPID(ukP, ukI, ukD); // setting PID
        shooterU_power = ShooterUPID.calculate(uVelocity, velocity1); // calculate output power
        ShooterDPID.setPID(dkP, dkI, dkD); // setting PID
        shooterD_power = ShooterDPID.calculate(dVelocity, velocity2); // calculate output power
        shooterU_power = clamp(shooterU_power + velocity1, -1, 1.0); // setting in correct range
        shooterD_power = clamp(shooterD_power + velocity2, -1, 1.0);
        shooterU.setPower(shooterU_power);
        shooterD.setPower(shooterD_power);

        // turret target yaw degree
        toYawDegree = turretTargetYaw;
        if (turretTargetYaw == -500) {  //tracking
            if (result != null && result.isValid()) {
                //tracking formula
//                double p = (Math.abs(result.getTx()) > 10) ? -spinP : -0.015;
//                double outputPower = result.getTx() * p;
//                if (getDegree() > 90) outputPower = clamp(outputPower, -1, 0);
//                else if (getDegree() < -90) outputPower = clamp(outputPower, 0, 1);
//                shooterSpinnerPower(result.getTx() * p);
                toYawDegree = getDegree() - result.getTx();
            } else {
                double targetDegree;
                if (dy(pipeline) < 0)
                    targetDegree = (dx(pipeline) >= 0) ? 0 : 180;   // right → 0, left → 180
                else targetDegree = Math.toDegrees(Math.atan2(dy(pipeline), dx(pipeline)));
                double heading = Math.toDegrees(follower.getPose().getHeading());
                toYawDegree = ((targetDegree - heading + 540) % 360) - 180;
            }
        }
        toDegree(toYawDegree);

        // turret target pitch degree
        toPitchDegree = turretTargetPitch;
        if (turretTargetPitch == 0)   //tracking
            toPitchDegree = 0.2726 * distance(pipeline) + 18.63;
        pitchDegree(toPitchDegree);

        // shooting or not
        if (openShooting && isAuto) elevatorUp();
        else if (openShooting && !isAuto && Math.abs(uVelocity - velocity1) < 0.03 && Math.abs(dVelocity - velocity1) < 0.03)
            elevatorUp();
        else elevatorOff();
    }

    public int tagNumber() {
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            return tag.getFiducialId();
        }
        return 0;
    }

    public void shooterPower(double power) {
        shooterU.setPower(power);
        shooterD.setPower(power);
    }

    public void clean() {
        shooterU.setPower(0.1);
        shooterD.setPower(0.1);
    }

    public void shooterSpinnerPower(double power) {
        shooterSpinner1.setPower(power);
        shooterSpinner2.setPower(power);
    }

    public double getPose() {
        return (double) aimAnalogInput.getVoltage() / aimAnalogInput.getMaxVoltage() * 360.0;
    }

    public double getDegree() {
        double reset = 325;
        if (getPose() < 150) return (getPose() - reset + 350) * 90.0 / 88.0;
        else return (getPose() - reset) * 90.0 / 88.0;
//        double reset = 143;
//        if (getPose() > 318) return (getPose() - reset - 350) * 90.0 / 88.0;
//        else return (getPose() - reset) * 90.0 / 88.0;
    }

    public void toDegree(double target) {
        target = clamp(target, -90, 90);
        SpinnerPID.setPID(spinP, 0, 0);
        double power = SpinnerPID.calculate(getDegree(), target);
        power = clamp(power, -0.2, 0.2);
        shooterSpinner1.setPower(power);
        shooterSpinner2.setPower(power);
    }


    //elevator
    public void elevatorUp() {
        elevator.setPower(1);
        arm.setPosition(0.49);
    }

    public void elevatorOff() {
        elevator.setPower(0);
        arm.setPosition(0.7);
    }

    public void cleaning() {
        elevator.setPower(0.1);
        arm.setPosition(0.2);
    }

    public void servoPose(double pose) {
        pose = clamp(pose, 0, 1);
        arm.setPosition(pose);
    }

    public void pitchDegree(double degree) {
        degree = clamp(degree, 24, 49);
        turretPitchR.setPosition(0.03 * (degree) - 0.71);
    }
}
