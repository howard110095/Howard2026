package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.*;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Configurable
public class Shooter {
    public boolean isLastShoot = false;
    public static double poseTarget = 0.5, poseDelta = 0, targetDistance = 0;
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
        double x;
        if (isAuto) {
            if (pipe == 0) x = AutoRedX;
            else if (pipe == 1) x = AutoMidX;
            else x = AutoBlueX;
        } else {
            if (pipe == 0) x = TeleRedX;
            else x = -TeleBlueX; // no head
        }
        return x - follower.getPose().getX();
    }

    public double dy(int pipe) {
        double y;
        if (isAuto) {
            if (pipe == 0) y = AutoRedY;
            else if (pipe == 1) y = AutoMidY;
            else y = AutoBlueY;
        } else {
            if (pipe == 0) y = TeleRedY;
            else y = -TeleBlueY;
        }
        return y - follower.getPose().getY();
    }

    public double distance(int pipe) {
        return Math.pow(dx(pipe) * dx(pipe) + dy(pipe) * dy(pipe), 0.5);
    }

    public void shooterPID(double targetVelocity, double uP, double uI, double uD, double dP, double dI, double dD) {
        double velocity1 = targetVelocity / MaxUpVelocity;
        double velocity2 = targetVelocity / MaxDownVelocity;
        uVelocity = ((shooterU.getVelocity() / 28) * 60) / MaxUpVelocity;
        dVelocity = ((shooterD.getVelocity() / 28) * 60) / MaxDownVelocity;
        ShooterUPID.setPID(uP, uI, uD); // setting PID
        shooterU_power = ShooterUPID.calculate(uVelocity, velocity1); // calculate output power
        ShooterDPID.setPID(dP, dI, dD); // setting PID
        shooterD_power = ShooterDPID.calculate(dVelocity, velocity2); // calculate output power
        shooterU_power = clamp(shooterU_power + velocity1, -1, 1.0); // setting in correct range
        shooterD_power = clamp(shooterD_power + velocity2, -1, 1.0);
        shooterU.setPower(shooterU_power);
        shooterD.setPower(shooterD_power);
    }

    public void shootingPRO(int pipeline, double targetVelocity, double turretTargetYaw, double turretTargetPitch, boolean openShooting) {
        // setting pipe line
        limelight.pipelineSwitch(Math.abs(pipeline));
        LLResult result = limelight.getLatestResult();

        // shooting Velocity
        toVelocity = targetVelocity;
        if (targetVelocity == 0) {  //tracking
            if (result != null && result.isValid())  //have apriltag, velocity
                targetDistance = 20.0 + ((29.5 - 14) / Math.tan(Math.toRadians(19.41 + limelight.getLatestResult().getTy())));
            else
                targetDistance = distance(pipeline);

            if (targetDistance > 120)
                toVelocity = 0.4228 * targetDistance * targetDistance - 102.92 * targetDistance + 10300;
            else toVelocity = 11.144 * targetDistance + 2376.3;

        }
        double velocity1 = toVelocity / MaxUpVelocity;
        double velocity2 = toVelocity / MaxDownVelocity;
        uVelocity = ((shooterU.getVelocity() / 28) * 60) / MaxUpVelocity;
        dVelocity = ((shooterD.getVelocity() / 28) * 60) / MaxDownVelocity;
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
            double targetDegree;
            targetDegree = Math.toDegrees(Math.atan2(dy(pipeline), dx(pipeline)));
            double heading = Math.toDegrees(follower.getPose().getHeading());
            toYawDegree = ((targetDegree - heading + 540) % 360) - 180;
            if (result != null && result.isValid()) {
                double deltaDegree = 0;
//                if (follower.getPose().getY() < -12 && pipeline == 0) deltaDegree = 0; //-3
//                else if (follower.getPose().getY() < -12 && pipeline == 2) deltaDegree = 0; //3
                toYawDegree -= (result.getTx() - deltaDegree) * 0.5;
            }
        }
        toDegree(toYawDegree);

        // turret target pitch degree
        toPitchDegree = turretTargetPitch;
        if (turretTargetPitch == 0) {
            if (targetDistance < 100) toPitchDegree = 0.3472 * targetDistance + 12.679;
            else if (targetDistance < 130) toPitchDegree = 45;
            else if (targetDistance >= 130) toPitchDegree = 47;
//            toPitchDegree = 0.1602 * targetDistance + 20.7;
        }
        pitchDegree(toPitchDegree);

        // shooting or not
        boolean speedReady = Math.abs(uVelocity - velocity1) < 0.1 && Math.abs(dVelocity - velocity1) < 0.1;
        if (!openShooting && !isAuto) {
            isLastShoot = false;
            shooterU.setPower(0);
            shooterD.setPower(0);
            elevatorOff();
        } else if (!openShooting && isAuto) {
            shooterU.setPower(shooterU_power);
            shooterD.setPower(shooterD_power);
            elevatorOff();
        } else if (openShooting && (isAuto || isLastShoot)) {
            shooterU.setPower(shooterU_power);
            shooterD.setPower(shooterD_power);
            elevatorUp();
        } else if (openShooting && !isAuto && Math.abs(uVelocity - velocity1) < 0.1 && Math.abs(dVelocity - velocity1) < 0.1) {
            isLastShoot = true;
            shooterU.setPower(shooterU_power);
            shooterD.setPower(shooterD_power);
            elevatorUp();
        } else {
            shooterU.setPower(shooterU_power);
            shooterD.setPower(shooterD_power);
            elevatorOff();
        }

        shooterU.setPower(shooterU_power);
        shooterD.setPower(shooterD_power);
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
        pp -= yawDegreeOffset * 0.01;
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
        degree = clamp(degree, 24, 48);
        turretPitchR.setPosition(0.03 * (degree) - 0.71);
    }

    public void color(double pwm) {
        pwm = clamp(pwm, 0.0, 1.0);
        led.setPosition(pwm);
    }

}
