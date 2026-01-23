package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "TeleTest", group = "Linear OpMode")
public class TeleTest extends RobotBase {
    public static double position = 0.5, rpm = 2500, setColor = 0;


    @Override
    public void robotInit() {
        if (savedPose != null) startingPose = savedPose;
        else startingPose = startBluePose; // 或 new Pose()
        follower.setStartingPose(startingPose);
    }

    @Override
    protected void robotInitLoop() {
        telemetry.addData("SavedPose", (savedPose != null) ? "YES" : "NO");
//        shooter.cameraP = 0.02;
//        shooter.cameraI = 0.015;
//        shooter.cameraD = 0.002;
    }

    @Override
    public void robotStart() {
        follower.startTeleopDrive();
    }

    @Override
    public void robotLoop() {



        intake.slowMode();
        colorSpinner.on(0.3);
        shooter.elevator.setPower(1);
        shooter.shooterU.setPower(0.5);
        shooter.shooterD.setPower(0.5);
        shooter.arm.setPosition(position);


        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);


//        telemetry.addData("target", t);
//        telemetry.addData("get degree", shooter.getDegree());
        telemetry.addData("getPose", shooter.getPose());
        telemetry.addData("getDegree", shooter.getDegree());
//        telemetry.addData("ty", shooter.limelight.getLatestResult().getTy());
        telemetry.addData("distance", shooter.distance(0));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("up velocity ", shooter.shooterU.getVelocity() / 28 * 60);
        telemetry.addData("down velocity ", shooter.shooterD.getVelocity() / 28 * 60);
        telemetry.update();

        telemetryM.addData("setColor", setColor);
        if (Math.abs(shooter.limelight.getLatestResult().getTx()) > 2) {
            telemetryM.addData("target vision", shooter.getDegree() - shooter.limelight.getLatestResult().getTx());
        } else {
            telemetryM.addData("target vision", shooter.getDegree());
        }
        telemetryM.addData("get now", shooter.getDegree());
        telemetryM.addData("target", rpm);
        telemetryM.addData("up velocity rpm", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetryM.addData("down velocity rpm", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetryM.addData("p", shooter.SpinnerPID.getP());
        telemetryM.addData("i", shooter.SpinnerPID.getI());
        telemetryM.addData("d", shooter.SpinnerPID.getD());
//        telemetryM.addData("target", t);
//        telemetryM.addData("get degree", shooter.getDegree());
//        telemetryM.addData("getLatestResult", shooter.limelight.getLatestResult().getTx());
//        telemetryM.addData("X", follower.getPose().getX());
//        telemetryM.addData("Y", follower.getPose().getY());
//        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetryM.update();
    }

    public void robotStop() {
    }
}




