package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "TeleTest", group = "Linear OpMode")
public abstract class TeleTest extends RobotBase {


    public static double p = 0, d = 0, t = 0;


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


        if (shooter.limelight.getLatestResult() != null && shooter.limelight.getLatestResult().isValid()) {
            toYawDegree = shooter.getDegree() - shooter.limelight.getLatestResult().getTx();
            telemetry.addData("Auto", "Mode");
            telemetryM.addData("Auto", "Mode");
        } else {
            toYawDegree = t;
            telemetry.addData("Field", "Mode");
            telemetryM.addData("Field", "Mode");
        }
        toYawDegree = clamp(toYawDegree, -90, 90);
        shooter.SpinnerPID.setPID(p, 0, d);
        double power = shooter.SpinnerPID.calculate(shooter.getDegree(), toYawDegree);
        power = clamp(power, -0.2, 0.2);
        shooter.shooterSpinner1.setPower(power);
        shooter.shooterSpinner2.setPower(power);


        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);


        telemetry.addData("target", t);
        telemetry.addData("get degree", shooter.getDegree());
        telemetry.addData("getLatestResult", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();

        telemetryM.addData("target", t);
        telemetryM.addData("get degree", shooter.getDegree());
        telemetryM.addData("getLatestResult", shooter.limelight.getLatestResult().getTx());
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetryM.update();
    }

    public void robotStop() {
    }
}




