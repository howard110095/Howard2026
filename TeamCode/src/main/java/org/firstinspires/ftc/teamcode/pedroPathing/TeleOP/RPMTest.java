package org.firstinspires.ftc.teamcode.pedroPathing.TeleOP;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.isAuto;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.setPitchDegree;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.setYawDegree;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;

@Configurable
@TeleOp(name = "RPMTest", group = "Linear OpMode")
public class RPMTest extends RobotBase {
    public static double targetVelocity = 3000;
    public static double uP = 0;
    public static double uI = 0;
    public static double uD = 0;
    public static double dP = 0;
    public static double dI = 0;
    public static double dD = 0;

    @Override
    public void robotInit() {
        isAuto = true;
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
        shooter.shooterPID(targetVelocity, uP, uI, uD, dP, dI, dD);

        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);

        telemetry.addData("distance", shooter.distance(0));
        telemetry.addData("Target velocity rpm", targetVelocity);
        telemetry.addData("up velocity rpm", shooter.shooterU.getVelocity() / 28 * 60);
        telemetry.addData("down velocity rpm", shooter.shooterD.getVelocity() / 28 * 60);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();

        telemetryM.addData("Target velocity rpm", targetVelocity);
        telemetryM.addData("up velocity rpm", shooter.shooterU.getVelocity() / 28 * 60);
        telemetryM.addData("down velocity rpm", shooter.shooterD.getVelocity() / 28 * 60);
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetryM.update();

    }

    public void robotStop() {
    }
}




