package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSpinner {
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
    }

    // 你可以加一些方法來控制 spinner，例如：
    public void on(double power) {
        spin.setPower(power);
    }

    public double getPose() {
        return (double) spinAnalogInput.getVoltage() / spinAnalogInput.getMaxVoltage() * 360.0;
    }

    public void onTo() {
        if (((345 < getPose() && getPose() < 350) || (0 < getPose() && getPose() < 5)) && !p1) {
            place1 = detectColor();
            p1 = true;
        }
        if (230 < getPose() && getPose() < 240 && !p2) {
            place2 = detectColor();
            p2 = true;
        }
        if (115 < getPose() && getPose() < 125 && !p3) {
            place3 = detectColor();
            p3 = true;
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
        if (color1.alpha() > 300) {
            if (color1.blue() > color1.green()) return 5;
            else return 4;
        } else {
            if (color2.blue() > color2.green()) return 5;
            else return 4;
        }
    }
}
//236 117 15
//-------------------------------------------------------------------------
/*

package indubitables.config.subsystem;

import static indubitables.config.util.RobotConstants.*;

        import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import indubitables.config.util.RobotConstants;
import indubitables.config.util.action.RunAction;

public class LiftSubsystem {
    private Telemetry telemetry;

    public DcMotor rightLift, leftLift;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public RunAction toZero, toHighBucket, toHighChamber, toHumanPlayer, toTransfer, toPark;
    public PIDController liftPID;
    public static int target;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0.005;


    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        liftPID = new PIDController(p, i, d);

        toZero = new RunAction(this::toZero);
        toHighBucket = new RunAction(this::toHighBucket);
        toHighChamber = new RunAction(this::toHighChamber);
        toHumanPlayer = new RunAction(this::toHumanPlayer);
        toTransfer = new RunAction(this::toTransfer);
        toPark = new RunAction(this::toPark);
    }

    public void updatePIDF(){
        if (!manual) {
            liftPID.setPID(p,i,d);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(getPos(), target);
            double ticks_in_degrees = 537.7 / 360.0;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            rightLift.setPower(power);
            leftLift.setPower(power);

            telemetry.addData("lift pos", getPos());
            telemetry.addData("lift target", target);
        }
    }

    public void manual(double n){
        manual = true;

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(hang) {
            n = -0.75;
        }

        rightLift.setPower(n);
        leftLift.setPower(n);
    }

    //Util
    public void targetCurrent() {
        setTarget(getPos());
        manual = false;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(int b) {
        target = b;
    }

    public void addToTarget(int b) {
        target += b;
    }

    public int getPos() {
        pos = rightLift.getCurrentPosition() - bottom;
        return rightLift.getCurrentPosition() - bottom;
    }

    // OpMode
    public void init() {
        liftPID.setPID(p,i,d);
        bottom = getPos();
    }

    public void start() {
        target = 0;
    }

    //Presets

    public void toZero() {
        manual = false;
        setTarget(liftToZero);
    }

    public void toHighBucket() {
        manual = false;
        setTarget(liftToHighBucket);
    }

    public void toHighChamber2() {
        setTarget(liftToHighChamber2);
    }

    public void toHighChamber() {
        setTarget(liftToHighChamber);
    }

    public void toHumanPlayer() {
        setTarget(liftToHumanPlayer);
    }

    public void toTransfer() {
        setTarget(liftToTransfer);
    }

    public void toPark() {
        setTarget(liftToPark);
    }

}










mport com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware{
    public DcMotorEx lf, lb, rf, rb;
    public DcMotorEx shooterU, shooterD;
    public DcMotorEx intake;
    public DcMotorEx feed;
    public CRServo aimX1, aimX2;
    public Servo aimYL, aimYR;
    public CRServo spin;
    public Servo arm;
    public Limelight3A limelight3A;
    public AnalogInput spinAnalog;
    public AnalogInput aimAnalog;
    public ColorSensor color1,color2;
    public IMU imu;
    public RobotHardware(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterU = hardwareMap.get(DcMotorEx.class,"shooterU");
        shooterD = hardwareMap.get(DcMotorEx.class,"shooterD");
        shooterU.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterD.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterD.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        aimX1 = hardwareMap.get(CRServo.class, "aimX1");
        aimX2 = hardwareMap.get(CRServo.class, "aimX2");
        aimYL = hardwareMap.get(Servo.class, "aimYL");
        aimYR = hardwareMap.get(Servo.class, "aimYR");

        spin = hardwareMap.get(CRServo.class, "spin");
        spin.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        aimAnalog = hardwareMap.get(AnalogInput.class,"aimAnalog");
        spinAnalog = hardwareMap.get(AnalogInput.class,"spinAnalog");

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        feed = hardwareMap.get(DcMotorEx.class,"feed");

        color1 = hardwareMap.get(ColorSensor.class,"color1");
        color2 = hardwareMap.get(ColorSensor.class,"color2");

        arm = hardwareMap.get(Servo.class,"arm");

        imu = hardwareMap.get(IMU.class,"imu");
    }
}
*/