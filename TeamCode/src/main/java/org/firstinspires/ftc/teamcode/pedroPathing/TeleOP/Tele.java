package org.firstinspires.ftc.teamcode.pedroPathing.TeleOP;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;


@Configurable
//@TeleOp(name = "Tele", group = "Linear OpMode")
public abstract class Tele extends RobotBase {
    protected abstract int targetAprilTag();

    public static double v = 3500, angle = 48;
    boolean teleopStarted = false;
    private Pose parkingB1 = new Pose(30, 18, Math.toRadians(270));
    private Pose parkingB2 = new Pose(36, 36, Math.toRadians(90));
    private Pose parkingR1 = new Pose(36, -12, Math.toRadians(90));
    private Pose parkingR2 = new Pose(36, -36, Math.toRadians(270));
    private PathChain tracking;
    private Timer TeleTimer;
    public double hangYawDegree = 0, hangPitch = 48;
    int EndGameMode = 0;
    boolean last2B = false, NormalMode = true;

    @Override
    public void robotInit() {
        TeleTimer = new Timer();
        autoYawOffset = 0;
        yawDegreeOffset = 0;
        isAuto = false;
        if (savedPose != null) startingPose = savedPose;
        follower.setStartingPose(startingPose);
    }

    @Override
    protected void robotInitLoop() {
    }

    @Override
    public void robotStart() {
        TeleTimer.resetTimer();
        follower.startTeleopDrive();
        if (targetAprilTag() == 2 && !FieldReverse) {
            follower.setX(-follower.getPose().getX());
            follower.setY(-follower.getPose().getY());
            follower.setHeading(Math.toRadians(Math.toDegrees(follower.getPose().getHeading()) + 180.0));
            FieldReverse = true;
        }
    }

    @Override
    public void robotLoop() {
        InitPose();

        if (gamepad1.y) EndGameMode = 3;
        if (gamepad1.b) EndGameMode = 2;
        if (gamepad1.x) EndGameMode = 1;
        if (gamepad1.a) EndGameMode = 0;
        if (gamepad2.y) yawDegreeOffset = 0;

        if (EndGameMode != 0) {
            if (EndGameMode == 3) foot.robotUp();
            else if (EndGameMode == 2) foot.robotDefend();
            else if (EndGameMode == 1) foot.robotDown();

            shooter.toDegree(90);
            shooter.shooterU.setPower(0);
            shooter.shooterD.setPower(0);
            colorSpinner.spin.setPower(0);
            intake.off();
        } else {
            if (gamepad1.right_trigger > 0.3) foot.robotDefend();
            else foot.robotDown();

            yawDegreeOffset += gamepad2.right_stick_x;

            ShooterIntakeControl();
            if (NormalMode) AutoTrackingMode();
            else HandMode(); // vision, pinpoint are not work (fool mode)

            //set shooting constant
            shooter.shootingPRO(targetAprilTag(), setVelocity, setYawDegree, setPitchDegree, setShooting);
        }

        if (!last2B && gamepad2.b) NormalMode = !NormalMode;
        last2B = gamepad2.b;

        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_x;

        follower.update();
        follower.setTeleOpDrive(axial, lateral, yaw * 0.8, !NormalMode);

//        if (gamepad1.left_trigger > 0.3) {
//            teleopStarted = false;
//            tracking = follower.pathBuilder()
//                    .addPath(new BezierLine(follower.getPose(), parkingB1))
//                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkingB1.getHeading())
//                    .build();
//            follower.followPath(tracking, true);
//            follower.update();
//        } else {
//            if (!teleopStarted) {
//                follower.breakFollowing();
//                follower.startTeleopDrive();
//                teleopStarted = true;
//            }
//            follower.update();
//            follower.setTeleOpDrive(axial, lateral, yaw * 0.8, false);
//        }


        telemetry.addData("now yaw degree", shooter.getDegree());
        telemetry.addData("distance", shooter.targetDistance);
        telemetry.addData("tx", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("target velocity", toVelocity);
        telemetry.addData("shooter Up velocity", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetry.addData("shooter Down velocity", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }

    public void InitPose() {
        if (gamepad1.dpad_up)
            follower.setHeading(Math.toRadians(0)); //front
        if (gamepad1.dpad_right)
            follower.setHeading(Math.toRadians(270)); //right
        if (gamepad1.dpad_left)
            follower.setHeading(Math.toRadians(90)); //left
        if (gamepad1.dpad_down)
            follower.setHeading(Math.toRadians(180)); //back

        if (targetAprilTag() == 2) { //blue
            if (gamepad2.dpad_up)
                follower.setPose(new Pose(InitUpX, -InitUpY, Math.toRadians(270))); //blue area
            if (gamepad2.dpad_right)
                follower.setPose(new Pose(-InitUpX, -InitUpY, Math.toRadians(270))); //red area
            if (gamepad2.dpad_left)
                follower.setPose(new Pose(InitCornerX, InitCornerY, Math.toRadians(0))); //red human
            if (gamepad2.dpad_down)
                follower.setPose(new Pose(-InitCornerX, InitCornerY, Math.toRadians(180))); //blue human
        } else {
            if (gamepad2.dpad_up)
                follower.setPose(new Pose(InitUpX, InitUpY, Math.toRadians(90))); //red area
            if (gamepad2.dpad_right)
                follower.setPose(new Pose(InitCornerX, -InitCornerY, Math.toRadians(0))); //blue human
            if (gamepad2.dpad_left)
                follower.setPose(new Pose(-InitUpX, InitUpY, Math.toRadians(90))); //blue area
            if (gamepad2.dpad_down)
                follower.setPose(new Pose(-InitCornerX, -InitCornerY, Math.toRadians(180))); //red human
        }
    }

    public void ShooterIntakeControl() {
        if (gamepad1.right_trigger > 0.3) { //shooting
            setShooting = true;
            colorSpinner.on();
            intake.on();
        } else if (gamepad1.left_bumper) {  //out ball
            setShooting = false;
            colorSpinner.out();
            intake.out();
        } else {
            setShooting = false;
            colorSpinner.slowMode();
            intake.on();
        }
    }

    public void AutoTrackingMode() {
        shooter.led.setPosition(0.6);
        setVelocity = 0;
        setYawDegree = -500;
        setPitchDegree = 0;
        if (shooter.limelight.getLatestResult() != null && shooter.limelight.getLatestResult().isValid())
            telemetry.addData("Mode", "Auto Tracking");
        else telemetry.addData("Mode", "Field Tracking");
    }

    public void HandMode() {
        shooter.led.setPosition(0.35); // set LED color
        hangYawDegree -= gamepad2.right_stick_x * 0.8; // adjust turret yaw degree
        hangPitch -= gamepad2.left_stick_y * 0.8; // adjust pitch degree

        setVelocity = 3200;
        setYawDegree =  clamp(hangYawDegree, -90.0, 90.0);;
        setPitchDegree = clamp(hangPitch, 24.0, 49.0);
        telemetry.addData("Mode", "Hand Mode");
    }
}




