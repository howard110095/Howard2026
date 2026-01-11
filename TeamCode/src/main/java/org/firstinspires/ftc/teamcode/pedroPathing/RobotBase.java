package org.firstinspires.ftc.teamcode.pedroPathing;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import androidx.core.content.pm.PermissionInfoCompat;
import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import java.util.List;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;


@Configurable
@TeleOp
public abstract class RobotBase extends OpMode {
    public Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    //-----------------------------------------
    public ColorSensor CS;
    public final Pose startPose = new Pose(0, 0, 0);
    //    protected Limelight3A limelight; // Limelight 相機，用於目標追蹤
    protected DcMotorEx SlideFront, SlideBack, ArmUp, ArmDown; //定義直流馬達
    protected CRServo Left, Right; // 手腕控制伺服馬達 (左/右)
    protected Servo Claw, Lwrist, Rwrist; // 爪子控制伺服馬達
//    private PIDController ArmPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
//    private PIDController SlidePID = new PIDController(0, 0, 0); // 升降滑軌 PID 控制器

    public Limelight3A limelight3A;

    //
    protected ColorSpinner colorSpinner;  // 這樣 Tele/Auto 都可以使用;\
    protected Elevator elevator;  // 這樣 Tele/Auto 都可以使用
    protected Shooter shooter;  // 這樣 Tele/Auto 都可以使用
    protected Intake intake;  // 這樣 Tele/Auto 都可以使用

    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 1000) // 右馬達全速震動 1000 毫秒
            .addStep(0.0, 0.0, 1000) // 暫停 1000 毫秒
            .build();

    public void init() {
        //follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();


//        limelight = new LimelightSystem(hardwareMap,telemetry);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");


        //setting
        colorSpinner = new ColorSpinner(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        robotInit(); // 執行自定義初始化邏輯

        telemetry.addData("init", "done");
        telemetry.update();
    }

    public void init_loop() {
        // 等待比賽開始
        robotInitLoop();
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    public void start() {
        robotStart();
    }

    public void loop() {
        robotLoop(); //比賽進行中，自定義執行邏輯
    }

    public void stop() {
        robotStop();
        //limelight.stop(); // 停止 Limelight
    }

    //
    protected abstract void robotInit();

    protected abstract void robotInitLoop();

    protected abstract void robotStart();

    protected abstract void robotLoop();

    protected abstract void robotStop();

    // 手臂轉動到指定角度
    public void armTurn2angle(double target) {
//        armPosNow = ArmUp.getCurrentPosition() / armEnc2deg + armOffset; // 計算當前角度
//        if (slidePosNow >= 50 && target <= 15 && armPosNow > 15) target = armPosNow;
//        target = clamp(target, armBottomLimit, armUpLimit); // 限制目標角度範圍
//        //28 -> 0 ; 80 -> 0.16
//        armF = (slidePosNow - smin) / (smax - smin) * 0.16 + 0;// 計算前饋
//        ArmPID.setPID(armP, armI, armD); // 設置 PID
//        ArmPID.setTolerance(10); // 設置誤差容忍度
//        armOutput = ArmPID.calculate(armPosNow, target) + armF * Math.cos(Math.toRadians(armPosNow))/* */; // 計算輸出
//        armOutput = clamp(armOutput, armPowerMin, armPowerMax); // 限制輸出範圍
//        ArmUp.setPower(armOutput);
//        ArmDown.setPower(armOutput);
    }



    // 滑軌移動到指定位置
    public void slideToPosition(double slidePos) {
//        slidePosNow = (SlideBack.getCurrentPosition() / slide2length) * 2 + smin + slideOffset; // 計算當前位置
//        slidePos = clamp(slidePos, smin, smax); // 限制目標範圍
//        SlidePID.setPID(slideP, slideI, slideD); // 設置 PID
//        slidePower = SlidePID.calculate(slidePosNow, slidePos) + Math.sin(Math.toRadians(slidePosNow)) * slideF; // 計算輸出
//        SlideFront.setPower(slidePower);
//        SlideBack.setPower(slidePower);
    }









    // 計算角度誤差
//    private double calculateAngleError(double target, double current) {
//        double error = target - current;
//        if (error > 180) error -= 360; // 誤差大於 180 時，取反方向
//        if (error < -180) error += 360; // 誤差小於 -180 時，取反方向
//        return error;
//    }
    // 將 Limelight 輸出轉換為 Pose2d
//    public void convertToPose2d(String telemetryOutput) {
//        limelight.stop();
//        String positionData = telemetryOutput.split("position=\\(")[1].split("\\)")[0];
//        String orientationData = telemetryOutput.split("yaw=")[1].split(",")[0];
//        String[] positionParts = positionData.split(" ");
//        double xMeters = Double.parseDouble(positionParts[0]);
//        double yMeters = Double.parseDouble(positionParts[1]);
//        double yawDegrees = Double.parseDouble(orientationData);
//        x_target = xMeters * 39.3701;
//        y_target = yMeters * 39.3701;
//        heading_target = yawDegrees;
//    }

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
}
