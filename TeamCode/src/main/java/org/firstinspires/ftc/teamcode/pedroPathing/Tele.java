package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;


@Configurable
//@TeleOp(name = "Tele", group = "Linear OpMode")
public abstract class Tele extends RobotBase {
    protected abstract int targetAprilTag();

    public boolean driveMode = false;
    public double hangVelocity = 3500, hangYawDegree = 0, hangPitch = 45;
    int modeTemp = 0, endTemp = 0;
    boolean last1Y = false, last1A = false, last1RB;

    @Override
    public void robotInit() {
        velocityOffset = 0;
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
        if (gamepad1.dpad_up) {
            follower.update();
            follower.setPose(InitCenter);
        }
        if (gamepad1.dpad_left) {
            follower.update();
            follower.setPose(InitBlueCorner);
        }
        if (gamepad1.dpad_right) {
            follower.update();
            follower.setPose(InitRedCorner);
        }

        if (endTemp % 3 == 0) { //normal
            foot.robotDown();
            yawDegreeOffset += gamepad2.right_stick_x;
            if(gamepad2.y) yawDegreeOffset = 0;

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

            if (modeTemp % 2 == 0) {
                shooter.led.setPosition(0.6);
                setVelocity = 0;
                setYawDegree = -500;
                setPitchDegree = 0;
                if (shooter.limelight.getLatestResult() != null && shooter.limelight.getLatestResult().isValid())
                    telemetry.addData("Mode", "Auto Tracking");
                else telemetry.addData("Mode", "Field Tracking");
            }
            // vision, pinpoint are not work
            else {
                shooter.led.setPosition(0.35);
                hangYawDegree -= gamepad2.right_stick_x * 0.8;
                hangPitch -= gamepad2.left_stick_y * 0.8;

                hangYawDegree = clamp(hangYawDegree, -90.0, 90.0);
                hangPitch = clamp(hangPitch, 24.0, 49.0);

                setYawDegree = hangYawDegree;
                setPitchDegree = hangPitch;
                telemetry.addData("Mode", "Hand Mode");
            }
            //set shooting constant
            shooter.shootingPRO(targetAprilTag(), setVelocity, setYawDegree, setPitchDegree, setShooting);
        }
        else {
            modeTemp = 0;
            shooter.toDegree(90);
            colorSpinner.on();
            intake.off();
            if (endTemp % 3 == 1) foot.robotDefend();
            else foot.robotUp();
        }

        if (!last1Y && gamepad1.y) endTemp++;
        else if (!last1RB && gamepad1.right_bumper) endTemp = 0;
        last1Y = gamepad1.y;
        last1RB = gamepad1.left_bumper;

        if (!last1A && gamepad1.a) modeTemp++;
        last1A = gamepad1.a;


        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = -gamepad1.right_stick_x;

        follower.update();
        if (gamepad1.left_stick_button) driveMode = true;
        if (gamepad1.right_stick_button) driveMode = false;
        follower.setTeleOpDrive(axial, lateral, yaw * 0.8, driveMode);

        telemetry.addData("tx", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("savedPose", savedPose);
        telemetry.addData("hangVelocity", hangVelocity);
        telemetry.addData("hangYawDegree", hangYawDegree);
        telemetry.addData("hangPitch", hangPitch);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }
}




