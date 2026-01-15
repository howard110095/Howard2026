package org.firstinspires.ftc.teamcode.pedroPathing;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import androidx.core.content.pm.PermissionInfoCompat;
import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
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
    public static Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    //-----------------------------------------
//    public final Pose startPose = new Pose(0, 0, 0);
//    private PIDController ArmPID = new PIDController(0, 0, 0); // 手臂 PID 控制器
//    private PIDController SlidePID = new PIDController(0, 0, 0); // 升降滑軌 PID 控制器

    //
    protected ColorSpinner colorSpinner;  // 這樣 Tele/Auto 都可以使用;
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

        robotInit(); // 執行自定義初始化邏輯
        //setting
        colorSpinner = new ColorSpinner(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);


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
        shooter.limelight.start();
        robotStart();
    }

    public void loop() {
        robotLoop(); //比賽進行中，自定義執行邏輯
    }

    public void stop() {
        robotStop();
        shooter.limelight.stop(); // 停止 Limelight
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

}
