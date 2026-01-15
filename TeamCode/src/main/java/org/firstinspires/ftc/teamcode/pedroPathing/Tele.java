package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Configurable
@TeleOp(name = "Tele", group = "Linear OpMode")
public class Tele extends RobotBase {
    public double test1 = 0.4;
    public boolean was1B = false, stop = false;
    public int temp1B = 0;

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
//        if (!stop) {
//            if (colorSpinner.detectColor(1) != 0 || colorSpinner.place1 != 0) {
//                if (colorSpinner.place1 == 0) colorSpinner.place1 = colorSpinner.detectColor(1);
//                if (colorSpinner.detectColor(2) != 0 || colorSpinner.place2 != 0) {
//                    if (colorSpinner.place2 == 0) colorSpinner.place2 = colorSpinner.detectColor(2);
//                    if (colorSpinner.detectColor(3) != 0 || colorSpinner.place3 != 0) {
//                        if (colorSpinner.place3 == 0) {
//                            colorSpinner.place3 = colorSpinner.detectColor(3);
//                            stop = true;
//                        }
//                    } else colorSpinner.turnToAngle(colorSpinner.Place3);
//                } else colorSpinner.turnToAngle(colorSpinner.Place2);
//            } else
//                colorSpinner.turnToAngle(colorSpinner.Place1);
//        } else colorSpinner.spin.setPower(0);


        if (gamepad1.dpad_up) test1 += 0.01;
        if (gamepad1.dpad_down) test1 -= 0.01;

        shooter.elevator.setPower(1);
        shooter.arm.setPosition(test1);
        shooter.shooting(2, false);
        intake.on();
        colorSpinner.on(0.4);
//Z
//        shooter.shooting(2,true);

//
//        shooter.shooterU.setPower(gamepad1.right_trigger);
//        shooter.shooterD.setPower(gamepad1.right_trigger);
////
//        if (gamepad1.dpad_up) test1 = 65;  //3
//        else if (gamepad1.dpad_left) test1 = 185;//1
//        else if (gamepad1.dpad_right) test1 = 305;//2
//        colorSpinner.turnToAngle(test1);


//        intake.on();
//        shooter.clean();
//        elevator.clean();

//        colorSpinner.onTo();
//        if (!was1B && gamepad1.b) colorSpinner.scanning();
//        was1B = gamepad1.b;

//        check1B();
//        if (!wasB && gamepad1.b) {
//            telemetry.addData("tx", limelight.getTxError());
//            telemetry.addData("ty", limelight.getTyError());
//            telemetry.addData("ta", limelight.getTaError());
//            telemetry.update();
//        }
//        wasB = gamepad1.b;
//        if(gamepad1.left_bumper) for_testing_1 += 0.01;
//        else if(gamepad1.right_bumper) for_testing_1 -= 0.01;
//        elevator.servoPose(for_testing_1);

//
//        if(gamepad1.left_bumper) shooter.onShoot();
//        else shooter.off();
//
//        if(gamepad1.right_bumper) intake.on();
//        else intake.off();
//
//        shooter.onTurn(gamepad1.left_stick_x*0.3);
//
//        if(gamepad1.left_trigger>0.3) elevator.up();
//        else  elevator.off();

        // 底盤遙控
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x * 0.6;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.5, true);


        // 顯示數據
//        telemetry.addData("1.red", colorSpinner.color1.red());
//        telemetry.addData("1.blue", colorSpinner.color1.blue());
//        telemetry.addData("1.green", colorSpinner.color1.green());
//        telemetry.addData("1.alpha", colorSpinner.color1.alpha());
//        telemetry.addData("2.red", colorSpinner.color2.red());
//        telemetry.addData("2.blue", colorSpinner.color2.blue());
//        telemetry.addData("2.green", colorSpinner.color2.green());
//        telemetry.addData("2.alpha", colorSpinner.color2.alpha());
        telemetry.addData("1", colorSpinner.place1);
        telemetry.addData("2", colorSpinner.place2);
        telemetry.addData("3", colorSpinner.place3);
//        telemetry.addData("delta", colorSpinner.delta);
        telemetry.addData("uVelocity", shooter.shooterU.getVelocity() / 28.0 * 60.0);
        telemetry.addData("dVelocity", shooter.shooterD.getVelocity() / 28.0 * 60.0);
        telemetry.addData("shooterU.getVelocity()", shooter.shooterU.getVelocity());
        telemetry.addData("shooterD.getVelocity()", shooter.shooterD.getVelocity());
        telemetry.addData("delta", colorSpinner.delta);
        telemetry.addData("test1", test1);
//        telemetry.addData("color spin pose", colorSpinner.getPose());
        telemetry.addData("getCurrentPose", colorSpinner.getDegree());
//        telemetry.addData("sh pose", shooter.getPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();
    }

    public void robotStop() {
    }

    public void check1B() {
        if (!was1B && gamepad1.b) temp1B++;

        if (temp1B % 4 == 0) { //take ball
            intake.off();
            colorSpinner.on(0);
            shooter.slowMode();
        }
        if (temp1B % 4 == 1) { //take ball
            intake.on();
            colorSpinner.on(0.18);
            shooter.slowMode();
        } else if (temp1B % 4 == 2) { // ready shooting
            intake.off();
            colorSpinner.on(0.18);
            shooter.onShoot();
        } else if (temp1B % 4 == 3) { //shooting
            intake.off();
            colorSpinner.on(0.4);
            shooter.onShoot();
        }

        was1B = gamepad1.b;
    }

}




