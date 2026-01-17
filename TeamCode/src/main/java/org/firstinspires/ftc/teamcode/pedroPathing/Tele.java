package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

@Configurable
@TeleOp(name = "Tele", group = "Linear OpMode")
public class Tele extends RobotBase {
    public static double test1 = 36, test2 = 3800, test3 = 30;

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
        if (gamepad1.dpad_up) test1 = 1;
        else if (gamepad1.dpad_left) test1 = 2;
        else if (gamepad1.dpad_right) test1 = 0;
        shooter.visionTracking((int) test1);

        if (gamepad1.y) test3 += 0.5;
        else if (gamepad1.a) test3 -= 0.5;

        //shooting
        shooter.shooting((int) test1, gamepad1.right_bumper);
        //spinner
        if (gamepad1.right_bumper) colorSpinner.spin.setPower(1);
        else colorSpinner.spin.setPower(0.18);
        //intake
        if (gamepad1.left_bumper) intake.on();
        else if (gamepad1.left_trigger > 0.5) intake.out();
        else intake.off();

        test3 = clamp(test3, 24, 49);
        shooter.pitchDegree(test3);


//        shooter.pitchDegree(test1);
//        shooter.setVelocity(test2, true);
//        intake.on();
//        colorSpinner.on(1);
//
//        shooter.setVelocity(test2, true);


//        shooter.visionTracking((int)test1);
        // 底盤遙控
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.5, true);


        // 顯示數據
        telemetry.addData("getLatestResult", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("shooterU.getPower", shooter.shooterU.getPower());
        telemetry.addData("shooterD.getPower", shooter.shooterD.getPower());
//        telemetry.addData("1", colorSpinner.place1);
//        telemetry.addData("2", colorSpinner.place2);
//        telemetry.addData("3", colorSpinner.place3);
        telemetry.addData("shooter.getPose()", shooter.getPose());
        telemetry.addData("uVelocity", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetry.addData("dVelocity", shooter.shooterD.getVelocity() / 28.0 * 60.0);
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




