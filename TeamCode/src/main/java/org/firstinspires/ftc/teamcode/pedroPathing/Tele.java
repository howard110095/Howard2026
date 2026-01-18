package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

@Configurable
@TeleOp(name = "Tele", group = "Linear OpMode")
public class Tele extends RobotBase {
    public static double targetVelocity = 3800, test2 = 3800, test3 = 30;
    int temp = 1;
    boolean last = false;

//    public static double velocityU,

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
//        intake.on();
//        colorSpinner.on(1);
        shooter.setVelocity(targetVelocity, true);
//        //teleop
//        if (gamepad1.left_trigger > 0.3) {
//            temp = 1;
//            intake.out();
//            colorSpinner.on(-1);
//        } else {
//            //shooting
//            shooter.shooting(2, gamepad1.right_bumper);
//            //spinner
//            if (gamepad1.right_bumper) colorSpinner.spin.setPower(1);
//            else colorSpinner.spin.setPower(0.18);
//            //intake state
//            if (!last && gamepad1.left_bumper) temp++;
//            last = gamepad1.left_bumper;
//        }
//        shooter.visionTracking(2);
//        colorSpinner.detect3posePRO();
//
//        test3 = clamp(test3, 24, 49);
//        shooter.pitchDegree(test3);


//        shooter.pitchDegree(test1);
//        shooter.setVelocity(test2, true);
//        intake.on();
//        colorSpinner.on(1);
//        shooter.setVelocity(test2, true);
//        shooter.visionTracking((int)test1);
        // 底盤遙控
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.8, true);

        // 顯示數據
        telemetryM.addData("target velocity", targetVelocity);
        telemetryM.addData("up velocity", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetryM.addData("down velocity", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetryM.update();

//        telemetry.addData("3 blue", colorSpinner.color3.blue());
//        telemetry.addData("3 green", colorSpinner.color3.green());
//        telemetry.addData("4 blue", colorSpinner.color4.blue());
//        telemetry.addData("4 green", colorSpinner.color4.green());
//        telemetry.addData("5 blue", colorSpinner.color5.blue());
//        telemetry.addData("5 green", colorSpinner.color5.green());
//        telemetry.addData("6 blue", colorSpinner.color6.blue());
//        telemetry.addData("6 green", colorSpinner.color6.green());
//        telemetry.addData("tagNumber", shooter.tagNumber());
//        telemetry.addData("getLatestResult", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("pid up calculate",shooter.ShooterUPID.calculate(shooter.shooterU.getVelocity() / 2800, targetVelocity / 4600));
        telemetry.addData("pid down calculate",shooter.ShooterDPID.calculate(shooter.shooterD.getVelocity() / 2800, targetVelocity / 4300));
        telemetry.addData("shooterU.getPower", shooter.shooterU.getPower());
        telemetry.addData("shooterD.getPower", shooter.shooterD.getPower());
//        telemetry.addData("1", colorSpinner.place1);
//        telemetry.addData("2", colorSpinner.place2);
//        telemetry.addData("3", colorSpinner.place3);
//        telemetry.addData("shooter.getPose()", shooter.getPose());
        telemetry.addData("Up Velocity ", shooter.uVelocity);
        telemetry.addData("Down Velocity ", shooter.dVelocity);
        telemetry.addData("Up Velocity rpm", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetry.addData("Down Velocity rpm", shooter.shooterD.getVelocity() / 28.0 * 60.0);
//        telemetry.addData("shooterU.getVelocity()", shooter.shooterU.getVelocity());
//        telemetry.addData("shooterD.getVelocity()", shooter.shooterD.getVelocity());
//        telemetry.addData("delta", colorSpinner.delta);
//        telemetry.addData("test1", test1);
//        telemetry.addData("color spin pose", colorSpinner.getPose());
//        telemetry.addData("getCurrentPose", colorSpinner.getDegree());
//        telemetry.addData("sh pose", shooter.getPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }
}




