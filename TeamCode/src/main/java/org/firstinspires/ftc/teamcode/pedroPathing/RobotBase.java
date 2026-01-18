package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import androidx.core.content.pm.PermissionInfoCompat;

import com.bylazar.configurables.annotations.Configurable;
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;


@Configurable
@TeleOp
public abstract class RobotBase extends OpMode {
    public static Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private Supplier<PathChain> pathChain;
    public TelemetryManager telemetryM;

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

    protected abstract void robotInit();

    protected abstract void robotInitLoop();

    protected abstract void robotStart();

    protected abstract void robotLoop();

    protected abstract void robotStop();

}
