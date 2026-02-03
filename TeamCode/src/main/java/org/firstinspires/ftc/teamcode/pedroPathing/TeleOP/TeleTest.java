package org.firstinspires.ftc.teamcode.pedroPathing.TeleOP;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.setPitchDegree;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.setShooting;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.setVelocity;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.setYawDegree;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;

@Configurable
@TeleOp(name = "TeleTest", group = "Linear OpMode")
public class TeleTest extends RobotBase {
    public static double target = 120, angle = 40;

    @Override
    public void robotInit() {
        follower.setStartingPose(startingPose);
    }

    @Override
    protected void robotInitLoop() {
//        telemetry.addData("SavedPose", (savedPose != null) ? "YES" : "NO");
    }

    @Override
    public void robotStart() {
        follower.startTeleopDrive();
    }

    @Override
    public void robotLoop() {
        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);


//        telemetry.addData("getPose", shooter.getPose());
//        telemetry.addData("getDegree", shooter.getDegree());
        telemetry.addData("up velocity rpm", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetry.addData("down velocity rpm", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetry.addData("distance", shooter.distance(0));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("up velocity ", shooter.shooterU.getVelocity() / 28 * 60);
        telemetry.addData("down velocity ", shooter.shooterD.getVelocity() / 28 * 60);
        telemetry.update();

//        telemetryM.addData("setColor", setColor);
//        telemetryM.addData("get now", shooter.getDegree());
//        telemetryM.addData("distance", shooter.distance(0));
//        telemetryM.addData("Up Power", shooter.shooterU.getPower());
//        telemetryM.addData("Down Power", shooter.shooterD.getPower());
        telemetryM.addData("target", target);
        telemetryM.addData("up velocity rpm", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetryM.addData("down velocity rpm", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetryM.addData("target",target);
        telemetryM.addData("now degree",colorSpinner.getDegree());
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetryM.update();
    }

    public void robotStop() {
    }
}




