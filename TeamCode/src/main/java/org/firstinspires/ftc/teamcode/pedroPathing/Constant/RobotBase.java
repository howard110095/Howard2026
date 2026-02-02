package org.firstinspires.ftc.teamcode.pedroPathing.Constant;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ColorSpinner;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Foot;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Shooter;


@Configurable
@TeleOp
public abstract class RobotBase extends OpMode {
    public static Follower follower;
    public static Pose startingPose = new Pose(0, 0, Math.toRadians(0)); //See ExampleAuto to understand how to use this
    public static Pose savedPose = null;
    private Supplier<PathChain> pathChain;
    public TelemetryManager telemetryM;
    protected ColorSpinner colorSpinner;
    protected Shooter shooter;
    protected Intake intake;
    protected Foot foot;

    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 1000) // work for 1000 microsecond
            .addStep(0.0, 0.0, 1000) // stop for 1000 microsecond
            .build();

    public void init() {
        //follower
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robotInit();
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        //setting
        colorSpinner = new ColorSpinner(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        foot = new Foot(hardwareMap, telemetry);

        //april tag
        AprilTagNumber = 0;

        telemetry.addData("init", "done");
        telemetry.update();
    }

    public void init_loop() {
        robotInitLoop();
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    public void start() {
        shooter.limelight.start();
        robotStart();
    }

    public void loop() {
        robotLoop();
        savedPose = follower.getPose();
    }

    public void stop() {
        robotStop();
        shooter.limelight.stop();
    }

    protected abstract void robotInit();

    protected abstract void robotInitLoop();

    protected abstract void robotStart();

    protected abstract void robotLoop();

    protected abstract void robotStop();

}
