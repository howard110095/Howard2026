package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotBase.follower;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Shooter {
    public DcMotorEx shooterU, shooterD, elevator;
    public Servo arm;
    public CRServo shooterSpinner1, shooterSpinner2;
    public AnalogInput aimAnalogInput;
    public Limelight3A limelight;

    private PIDController SpinnerPID = new PIDController(0.02, 0, 0.02); // 手臂 PID 控制器
    private PIDController ShooterUPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
    private PIDController ShooterDPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
    public static double ukP = 0.001, ukI = 0.001, ukD = 0, dkP = 0.001, dkI = 0.001, dkD = 0;
    public static double spinP = 0.015, spinD = 0;
    public double shooterVelocity = 2000, uVelocity, dVelocity, uError, dError, shooterU_power, shooterD_power;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shooterSpinner1 = hardwareMap.get(CRServo.class, "aimX1");
        shooterSpinner2 = hardwareMap.get(CRServo.class, "aimX2");
        aimAnalogInput = hardwareMap.get(AnalogInput.class, "aimAnalogInput");
        shooterU = hardwareMap.get(DcMotorEx.class, "shooterU");
        shooterD = hardwareMap.get(DcMotorEx.class, "shooterD");

        shooterU.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterU.setPower(0);
        shooterD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterD.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setPower(0);

        shooterSpinner1.setPower(0);
        shooterSpinner2.setPower(0);

        arm = hardwareMap.get(Servo.class, "arm");
        elevator = hardwareMap.get(DcMotorEx.class, "feed");
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(0);
    }

    public void noVisionTracking(int pipe) {
        double targetX = 0, targetY = 0, targetDegree = 0;
        if (pipe == 0) {  //red
            targetX = 66;
            targetY = 66;
        } else if (pipe == 1) {  //middle
            targetX = 0;
            targetY = 72;
        } else if (pipe == 2) {  //blue
            targetX = -66;
            targetY = 66;
        }

        if (false) {
            if (follower.getPose().getX() == targetX) targetDegree = 90;
            else if (targetY < follower.getPose().getY() && follower.getPose().getX() < targetX)
                targetDegree = 0;
            else if (targetY < follower.getPose().getY() && follower.getPose().getX() > targetX)
                targetDegree = 180;
            else {
                double slope = (follower.getPose().getY() - targetY) / (follower.getPose().getX() - targetX);
                double delta = Math.toDegrees(Math.atan(slope));
                if (delta > 0) targetDegree = delta;
                else targetDegree = 180 + delta;
            }

        } else {
            double dx = targetX - follower.getPose().getX();
            double dy = targetY - follower.getPose().getY();

            if (dy < 0) { // 目標在上方（依你的座標定義）
                targetDegree = (dx >= 0) ? 0 : 180;   // 右上→0，左上→180
            } else {
                targetDegree = Math.toDegrees(Math.atan2(dy, dx));
            }
        }


        double heading = Math.toDegrees(follower.getPose().getHeading());
        double degree = ((targetDegree - heading + 540) % 360) - 180;
        degree = clamp(degree, -90, 90);
        toDegree(degree);
    }

    public void visionTracking(int pipe) {
        limelight.pipelineSwitch(pipe);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            //tracking formula
            if (Math.abs(result.getTx()) > 10) {
                //ta補償偏移
                shooterSpinner1.setPower(-result.getTx() / 10000.0 * 100.0);
                shooterSpinner2.setPower(-result.getTx() / 10000.0 * 100.0);
            } else {
                shooterSpinner1.setPower(-result.getTx() / 5000.0 * 100.0);
                shooterSpinner2.setPower(-result.getTx() / 5000.0 * 100.0);
            }
        } else {
            noVisionTracking(pipe);
        }

    }

    public void shooting(int pipe, boolean on) {
        if (on) {
            limelight.pipelineSwitch(pipe);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double y = limelight.getLatestResult().getTy();
                if (y > -16.5) shooterVelocity = 4.5894 * Math.pow(y, 2) - 3.1401 * y + 2506.5;
                else shooterVelocity = 4100;
            } else {

            }
            //set power
            uVelocity = shooterU.getVelocity() / 28.0 * 60.0;
            dVelocity = shooterD.getVelocity() / 28.0 * 60.0;

            ShooterUPID.setPID(ukP, ukI, ukP); // 設置 PID
            shooterU_power = ShooterUPID.calculate(uVelocity, shooterVelocity); // 計算輸出
            ShooterDPID.setPID(dkP, dkI, dkP); // 設置 PID
            shooterD_power = ShooterDPID.calculate(dVelocity, shooterVelocity); // 計算輸出
            shooterU_power = 1;
            shooterD_power = 1;
            elevatorUp();
            //--
//            if (uVelocity > shooterVelocity - 300 &&
//                    dVelocity > shooterVelocity - 300 &&
//                    uVelocity < shooterVelocity &&
//                    dVelocity < shooterVelocity) {
//                elevatorUp();
//            }
        } else {
            shooterU_power = 1;
            shooterD_power = 1;
        }
        shooterU.setPower(shooterU_power);
        shooterD.setPower(shooterD_power);
    }

    public void onShoot() {
        shooterU.setPower(1);
        shooterD.setPower(1);
    }

//    public void slowMode() {
//        shooterU.setPower(0.7);
//        shooterD.setPower(0.7);
//    }

    public void shooterOff() {
        shooterU.setPower(0);
        shooterD.setPower(0);
    }

    public void clean() {
        shooterU.setPower(0.1);
        shooterD.setPower(0.1);
    }

    public void onTurn(double power) {
        shooterSpinner1.setPower(power);
        shooterSpinner2.setPower(power);
    }

    public double getPose() {
        return (double) aimAnalogInput.getVoltage() / aimAnalogInput.getMaxVoltage() * 360.0;
    }

    public double getDegree() {
        double reset = 325;
        if (getPose() < 100) return (getPose() - reset + 350) * 90.0 / 88.0;
        else return (getPose() - reset) * 90.0 / 88.0;
    }

    public void toDegree(double target) {
        target = clamp(target, -90, 90);
        SpinnerPID.setPID(spinP, 0, spinD);
        double power = SpinnerPID.calculate(getDegree(), target);
//        double power = (target - getDegree()) * 0.02;
        power = clamp(power, -1, 1);
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
}
