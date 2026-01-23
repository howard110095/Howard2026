package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
//@TeleOp(name = "Tele", group = "Linear OpMode")
public abstract class Tele extends RobotBase {
    protected abstract int targetAprilTag();

    int temp = 0;
    boolean lastState = false;

    @Override
    public void robotInit() {

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
        if (gamepad1.right_bumper) foot.robotUp();
        else foot.robotDown();

        if (temp % 2 == 0) {
            setVelocity = 0;
            setYawDegree = -500;
            setPitchDegree = 0;
            if (gamepad1.right_trigger > 0.3) { //shooting
                setShooting = true;
                colorSpinner.on(1);
                intake.slowMode();
            } else if (gamepad1.left_bumper) {  //out ball
                setShooting = false;
                colorSpinner.on(-1);
                intake.out();
            } else if (gamepad1.left_trigger > 0.3) { //intake ball
                setShooting = false;
                colorSpinner.on(0.18);
                intake.on();
            } else {
                setShooting = false;
                colorSpinner.on(0.18);
                intake.off();
            }
            if (shooter.limelight.getLatestResult() != null && shooter.limelight.getLatestResult().isValid())
                telemetry.addData("Mode", "Auto Tracking");
            else telemetry.addData("Mode", "Field Tracking");
        } else {
            setVelocity = 3600;
            setYawDegree = 0;
            setPitchDegree = 45;
            if (gamepad1.right_trigger > 0.3) { //shooting
                setShooting = true;
                colorSpinner.on(1);
                intake.off();
            } else if (gamepad1.left_bumper) {  //out ball
                setShooting = false;
                colorSpinner.on(-1);
                intake.off();
            } else if (gamepad1.left_trigger > 0.3) { //intake ball
                setShooting = false;
                colorSpinner.on(0.18);
                intake.on();
            } else {
                setShooting = false;
                colorSpinner.on(0.18);
                intake.off();
            }
            telemetry.addData("Mode", "Hand Mode");
        }

        if (!lastState && gamepad2.left_bumper) temp++;
        lastState = gamepad2.left_bumper;
        shooter.shootingPRO(targetAprilTag(), setVelocity, setYawDegree, setPitchDegree, setShooting);

        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);

        telemetry.addData("ty", shooter.limelight.getLatestResult().getTy());
//        telemetry.addData("distance pinpoint", shooter.distance(0));
//        telemetry.addData("distance vision", shooter.d);
//        telemetry.addData("toVelocity", toVelocity);
        telemetry.addData("target degree", toYawDegree);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }
}




