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
    public static double poseTarget = 0.5, poseDelta = 0;
    public DcMotorEx shooterU, shooterD, elevator;
    public Servo arm, turretPitchL, turretPitchR, shooterSpinner1, shooterSpinner2, led;
    public AnalogInput aimAnalogInput;
    public Limelight3A limelight;
    public PIDController SpinnerPID = new PIDController(0.02, 0, 0.02);
    public PIDController ShooterUPID = new PIDController(0, 0, 0);
    public PIDController ShooterDPID = new PIDController(0, 0, 0);
    public static boolean controlShooting = false;
    public double shooterVelocity = 2000, uVelocity, dVelocity, shooterU_power, shooterD_power;
//    public static double cameraP = 0.02, cameraI = 0.015, cameraD = 0.002;

    public static double d;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shooterSpinner1 = hardwareMap.get(Servo.class, "aimX1");
        shooterSpinner2 = hardwareMap.get(Servo.class, "aimX2");
        turretPitchL = hardwareMap.get(Servo.class, "aimYL");
        turretPitchR = hardwareMap.get(Servo.class, "aimYR");
        arm = hardwareMap.get(Servo.class, "arm");
        aimAnalogInput = hardwareMap.get(AnalogInput.class, "aimAnalogInput");
        shooterU = hardwareMap.get(DcMotorEx.class, "shooterU");
        shooterD = hardwareMap.get(DcMotorEx.class, "shooterD");
        elevator = hardwareMap.get(DcMotorEx.class, "feed");
        led = hardwareMap.get(Servo.class, "LED");
        led.setPosition(1);

        shooterU.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterU.setPower(0);
        shooterD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterD.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setPower(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(0);

    }

    public double dx(int pipe) {
        if (pipe == 0) return 66.0 - follower.getPose().getX();
        else if (pipe == 1) return 0.0 - follower.getPose().getX();
        else return -66.0 - follower.getPose().getX();
    }

    public double dy(int pipe) {
        if (pipe == 1) return 72.0 - follower.getPose().getY();
        else return 69.0 - follower.getPose().getY();
    }

    public double distance(int pipe) {
        return Math.pow(dx(pipe) * dx(pipe) + dy(pipe) * dy(pipe), 0.5);
    }

    public void shootingPRO(int pipeline, double targetVelocity, double turretTargetYaw, double turretTargetPitch, boolean openShooting) {
        // setting pipe line
        limelight.pipelineSwitch(pipeline);
        LLResult result = limelight.getLatestResult();

        // shooting Velocity
        toVelocity = targetVelocity;
        if (targetVelocity == 0) {  //tracking
            if (result != null && result.isValid()) { //have apriltag, velocity
                d = (29.5 - 14) / Math.tan(Math.toRadians(19.41 + limelight.getLatestResult().getTy()));
                d += 20;
                toVelocity = 11.144 * d + 2376.3;
            } else
                toVelocity = 11.144 * distance(pipeline) + 2376.3; // no apriltag, velocity formula
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
//            if (result != null && result.isValid()) {
                //tracking formula
//                if (Math.abs(result.getTx()) > 2) toYawDegree = getDegree() - result.getTx();
//                else toYawDegree = getDegree();
//                double p = (Math.abs(result.getTx()) > 10) ? -spinP : -0.015;
//                double outputPower = result.getTx() * p;
//                if (getDegree() > 90) outputPower = clamp(outputPower, -1, 0);
//                else if (getDegree() < -90) outputPower = clamp(outputPower, 0, 1);
//                shooterSpinnerPower(result.getTx() * p);
//                toYawDegree = getDegree() - result.getTx();
//                SpinnerPID.setPID(cameraP, cameraI, cameraD);
//            } else {
                double targetDegree;
                if (dy(pipeline) < 0)
                    targetDegree = (dx(pipeline) >= 0) ? 0 : 180;   // right → 0, left → 180
                else targetDegree = Math.toDegrees(Math.atan2(dy(pipeline), dx(pipeline)));
                double heading = Math.toDegrees(follower.getPose().getHeading());
                toYawDegree = ((targetDegree - heading + 540) % 360) - 180;
//                SpinnerPID.setPID(spinP, spinI, spinD);
//            }
        } else {
//            SpinnerPID.setPID(spinP, spinI, spinD);
        }
        toDegree(toYawDegree);

        // turret target pitch degree
        toPitchDegree = turretTargetPitch;
        if (turretTargetPitch == 0)   //tracking
            toPitchDegree = 0.3472 * distance(pipeline) + 12.679;
        pitchDegree(toPitchDegree);

        // shooting or not
        if (openShooting && isAuto) elevatorUp();
        else if (openShooting && !isAuto && Math.abs(uVelocity - velocity1) < 0.1 && Math.abs(dVelocity - velocity1) < 0.1)
            elevatorUp();
        else elevatorOff();

        if (result != null && result.isValid()) led.setPosition(0.55);
        else led.setPosition(0.3);
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


    public double getPose() {
        return (double) aimAnalogInput.getVoltage() / aimAnalogInput.getMaxVoltage() * 360.0;
//        return shooterSpinner1.getPosition();
    }

    public double getDegree() {
        double reset = 177.5;
        return (getPose() - reset) * 180.0 / 177.0; //black
//        double reset = 143;
//        if (getPose() > 318) return (getPose() - reset - 350) * 90.0 / 88.0;
//        else return (getPose() - reset) * 90.0 / 88.0;

//        return (getPose() - 0.5) * 180.0 / 0.56;
    }

    public void toDegree(double target) {
        target = clamp(target, -90, 90);
        double pp = 0.5 + target * 0.56 / 180.0;
////        SpinnerPID.setPID(0.018, 0.08, 0.0008);
//        double power = SpinnerPID.calculate(getDegree(), target);
//        power = clamp(power, -0.2, 0.2);
//        shooterSpinner1.setPower(power);
//        shooterSpinner2.setPower(power);


        shooterSpinner1.setPosition(pp);
        shooterSpinner2.setPosition(pp);
    }


    //elevator
    public void elevatorUp() {
        elevator.setPower(1);
        arm.setPosition(0.39);//white
//        arm.setPosition(0.5);//black
    }

    public void elevatorOff() {
        elevator.setPower(0);
        arm.setPosition(0.7); //white
//        arm.setPosition(0.7); //black
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

    public void color(double pwm) {
        pwm = clamp(pwm, 0.0, 1.0);
//        turretPitchR.setPosition(0.03 * (degree) - 0.71);
        led.setPosition(pwm);
    }

}
