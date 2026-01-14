package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotBase.follower;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
public class Shooter {
    public DcMotorEx shooterU, shooterD, elevator;
    public Servo arm;
    public CRServo shooterSpinner1, shooterSpinner2;
    public AnalogInput aimAnalogInput;
    public Limelight3A limelight;

    private PIDController ShooterUPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
    private PIDController ShooterDPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
    public static double ukP = 0.001, ukI = 0.001, ukD = 0, dkP = 0.001, dkI = 0.001, dkD = 0;
    public double shooterVelocity, uVelocity, dVelocity,uError,dError,shooterU_power,shooterD_power;

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
        double heading = Math.toDegrees(follower.getPose().getHeading());
        if (pipe == 0) { //red
            if (heading <= 135) toDegree(45 - heading);
            else if (315 <= heading) toDegree(405 - heading);
            else if (135 < heading && heading <= 225) toDegree(-90);
            else toDegree(90);
        } else if (pipe == 2) { //blue
            if (45 <= heading && heading <= 225) toDegree(135 - heading);
            else if (225 < heading && heading <= 315) toDegree(-90);
            else toDegree(90);
        } else if (pipe == 1) {  //middle
            if (0 <= heading && heading <= 180) toDegree(90 - heading);
            else if (180 < heading && heading <= 270) toDegree(-90);
            else toDegree(90);
        }
    }

    public void visionTracking(int pipe) {
        limelight.pipelineSwitch(pipe);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            //tracking formula


        } else {
            noVisionTracking(pipe);
        }

    }

    public void shooting(int pipe,boolean on) {
        if(on){
            limelight.pipelineSwitch(pipe);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double y = limelight.getLatestResult().getTy();
                if (y > -16.5) shooterVelocity = 4.5894 * Math.pow(y, 2) - 3.1401 * y + 2506.5;
                else shooterVelocity = 4100;

            } else {

            }
            //set power
            uVelocity = shooterU.getVelocity() / 28 * 60;
            dVelocity = shooterD.getVelocity() / 28 * 60;
            uError = shooterVelocity - uVelocity;
            dError = shooterVelocity - dVelocity;

            ShooterUPID.setPID(ukP, ukI, ukP); // 設置 PID
            shooterU_power = ShooterUPID.calculate(uVelocity, shooterVelocity); // 計算輸出
            ShooterDPID.setPID(dkP, dkI, dkP); // 設置 PID
            shooterD_power = ShooterDPID.calculate(dVelocity, shooterVelocity); // 計算輸出
            //--
            if (uVelocity > shooterVelocity - 300 &&
                    dVelocity > shooterVelocity - 300 &&
                    uVelocity < shooterVelocity &&
                    dVelocity < shooterVelocity) {
                elevatorUp();
            }
        }
        else{
            shooterU_power = 0.3;
            shooterD_power = 0.3;
        }
        shooterU.setPower(shooterU_power);
        shooterD.setPower(shooterD_power);
    }

    public void onShoot() {
        shooterU.setPower(1);
        shooterD.setPower(1);
    }

    public void slowMode() {
        shooterU.setPower(0.3);
        shooterD.setPower(0.3);
    }

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

    public void toDegree(double target) {
        target = clamp(target, -90, 90);
        double nowDegree = (getPose() - 0) / 360.0 * 180;//
        // catching
        double power = (nowDegree - target) * 0.02;
        power = clamp(power, -1, 1);
        shooterSpinner1.setPower(power);
        shooterSpinner2.setPower(power);
    }


    //elevator
    public void elevatorUp() {
        elevator.setPower(1);
        arm.setPosition(0.45);
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
