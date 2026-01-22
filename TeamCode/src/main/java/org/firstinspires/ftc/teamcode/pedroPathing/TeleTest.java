package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "TeleTest", group = "Linear OpMode")
public class TeleTest extends RobotBase {


    public static double angle = 30, rpm = 2500;


    @Override
    public void robotInit() {

    }

    @Override
    protected void robotInitLoop() {
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


//        if (shooter.limelight.getLatestResult() != null && shooter.limelight.getLatestResult().isValid()) {
//            toYawDegree = shooter.getDegree() - shooter.limelight.getLatestResult().getTx();
//            t = toYawDegree;
//            telemetry.addData("Auto", "Mode");
//            telemetryM.addData("Auto", "Mode");
//        } else {
//        }


//        shooter.shooterD.setPower(1);
//
        setVelocity = rpm;
        setPitchDegree = angle;
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
        shooter.shootingPRO(0, setVelocity, setYawDegree, setPitchDegree, setShooting);

        // drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);


//        telemetry.addData("target", t);
//        telemetry.addData("get degree", shooter.getDegree());
        telemetry.addData("ty", shooter.limelight.getLatestResult().getTy());
        telemetry.addData("distance", shooter.distance(0));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("up velocity ", shooter.shooterU.getVelocity() / 28 * 60);
        telemetry.addData("down velocity ", shooter.shooterD.getVelocity() / 28 * 60);
        telemetry.update();


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




