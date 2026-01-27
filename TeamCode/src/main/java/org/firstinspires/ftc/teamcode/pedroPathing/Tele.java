package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;


@Configurable
//@TeleOp(name = "Tele", group = "Linear OpMode")
public abstract class Tele extends RobotBase {
    protected abstract int targetAprilTag();

    public boolean driveMode = false, lastDriveMode = true;
    public double hangYawDegree = 0, hangPitch = 45;
    int EndGameMode = 0, isChangeDriveMode = 1;
    boolean last1RB = false, NormalMode = true;

    @Override
    public void robotInit() {
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
        follower.startTeleopDrive();
    }

    @Override
    public void robotLoop() {
        //InitPose();
        if (gamepad1.dpad_up) {
//            follower.update();
            follower.setPose(InitCenter);
        }
        if (gamepad1.dpad_left) {
//            follower.update();
            follower.setPose(InitBlueCorner);
        }
        if (gamepad1.dpad_right) {
//            follower.update();
            follower.setPose(InitRedCorner);
        }


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
            foot.robotDown();
            yawDegreeOffset += gamepad2.right_stick_x;

            ShooterIntakeControl();

            if (NormalMode) AutoTrackingMode();
            else HandMode(); // vision, pinpoint are not work (fool mode)


            //set shooting constant
            shooter.shootingPRO(targetAprilTag() * isChangeDriveMode, setVelocity, setYawDegree, setPitchDegree, setShooting);
        }


        if (!last1RB && gamepad1.right_bumper) NormalMode = !NormalMode;
        last1RB = gamepad1.right_bumper;

        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_x;

        follower.update();
        if (gamepad1.left_stick_button) driveMode = true;
        if (gamepad1.right_stick_button) driveMode = false;

        if (lastDriveMode && !driveMode && targetAprilTag() == 2) {
            isChangeDriveMode = -1;
            follower.setX(-follower.getPose().getX());
            follower.setY(-follower.getPose().getY());
            follower.setHeading(Math.toRadians(Math.toDegrees(follower.getPose().getHeading()) + 180.0));
        } else if (!lastDriveMode && driveMode && targetAprilTag() == 2) {
            isChangeDriveMode = 1;
            follower.setX(-follower.getPose().getX());
            follower.setY(-follower.getPose().getY());
            follower.setHeading(Math.toRadians(Math.toDegrees(follower.getPose().getHeading()) + 180.0));
        }

        lastDriveMode = driveMode;
        follower.setTeleOpDrive(axial, lateral, yaw * 0.8, driveMode);

        telemetry.addData("tx", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("target velocity", toVelocity);
        telemetry.addData("hangYawDegree", hangYawDegree);
        telemetry.addData("hangPitch", hangPitch);
        telemetry.addData("dx", shooter.dx(targetAprilTag() * isChangeDriveMode));
        telemetry.addData("dy", shooter.dy(targetAprilTag() * isChangeDriveMode));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }

    public void InitPose() {
        if (targetAprilTag() == 2 && !driveMode) {
            if (gamepad2.dpad_up)
                follower.setPose(new Pose(39, -63, Math.toRadians(270))); //blue area
            if (gamepad2.dpad_right)
                follower.setPose(new Pose(-39, -63, Math.toRadians(270))); //red area
            if (gamepad2.dpad_left)
                follower.setPose(new Pose(63, 63, Math.toRadians(0))); //red human
            if (gamepad2.dpad_down)
                follower.setPose(new Pose(-63, 63, Math.toRadians(180))); //blue human
        } else {
            if (gamepad2.dpad_up)
                follower.setPose(new Pose(39, 63, Math.toRadians(90))); //red area
            if (gamepad2.dpad_right)
                follower.setPose(new Pose(-39, 63, Math.toRadians(90))); //blue area
            if (gamepad2.dpad_left)
                follower.setPose(new Pose(63, -63, Math.toRadians(0))); //blue human
            if (gamepad2.dpad_down)
                follower.setPose(new Pose(-63, -63, Math.toRadians(180))); //red human
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
        } else if (gamepad1.left_trigger > 0.3) { //intake ball
            setShooting = false;
            colorSpinner.slowMode();
            intake.on();
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

        hangYawDegree = clamp(hangYawDegree, -90.0, 90.0);
        hangPitch = clamp(hangPitch, 24.0, 49.0);

        setVelocity = 3500;
        setYawDegree = hangYawDegree;
        setPitchDegree = hangPitch;
        telemetry.addData("Mode", "Hand Mode");
    }
}




