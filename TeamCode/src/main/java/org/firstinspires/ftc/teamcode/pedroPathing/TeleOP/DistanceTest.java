package org.firstinspires.ftc.teamcode.pedroPathing.TeleOP;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;

@Configurable
@TeleOp(name = "DistanceTest", group = "Linear OpMode")
public class DistanceTest extends RobotBase {

    @Override
    public void robotInit() {
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
        shooter.shootingPRO(0, 300, -500, 0, false);

        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);


        telemetry.addData("pinpoint distance", shooter.distance(0));
        telemetry.addData("limelight distance", 20.0 + ((29.5 - 14) / Math.tan(Math.toRadians(19.41 + shooter.limelight.getLatestResult().getTy()))));
        telemetry.addData("Error distance", shooter.distance(0) - 20.0 - ((29.5 - 14) / Math.tan(Math.toRadians(19.41 + shooter.limelight.getLatestResult().getTy()))));
        telemetry.addData("limelight ty", shooter.limelight.getLatestResult().getTy());
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
//        telemetryM.addData("target", target);
//        telemetryM.addData("up velocity rpm", shooter.shooterU.getVelocity() / 28.0 * 60.0);
//        telemetryM.addData("down velocity rpm", shooter.shooterD.getVelocity() / 28.0 * 60.0);
//        telemetryM.addData("target",target);
//        telemetryM.addData("now degree",colorSpinner.getDegree());
//        telemetryM.addData("X", follower.getPose().getX());
//        telemetryM.addData("Y", follower.getPose().getY());
//        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
//        telemetryM.update();
    }

    public void robotStop() {
    }
}




