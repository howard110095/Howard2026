package org.firstinspires.ftc.teamcode.pedroPathing;

//import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;


@Configurable
@TeleOp(name = "Tele", group = "Linear OpMode")
public class Tele extends RobotBase {
    public double for_testing_1 = 0, for_testing_2 = 0, for_testing_3 = 0;   //0.53 0.94
    public boolean wasB = false;
    @Override
    public void robotInit() {
        colorSpinner.colorRenew();
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

        colorSpinner.check();
        colorSpinner.onTo();
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


        if(gamepad1.left_bumper) shooter.onShoot();
        else shooter.off();

        if(gamepad1.right_bumper) intake.on();
        else intake.off();

        shooter.onTurn(gamepad1.left_stick_x*0.3);

        if(gamepad1.left_trigger>0.3) elevator.up();
        else  elevator.off();

        // 底盤遙控
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x * 0.6;
        follower.update();
        //follower.setTeleOpDrive(axial, lateral, -yaw * 0.5, true);


        // 顯示數據
        telemetry.addData("1.red",colorSpinner.color1.red());
        telemetry.addData("1.blue",colorSpinner.color1.blue());
        telemetry.addData("1.green",colorSpinner.color1.green());
        telemetry.addData("1.alpha",colorSpinner.color1.alpha());
        telemetry.addData("2.red",colorSpinner.color2.red());
        telemetry.addData("2.blue",colorSpinner.color2.blue());
        telemetry.addData("2.green",colorSpinner.color2.green());
        telemetry.addData("2.alpha",colorSpinner.color2.alpha());
        telemetry.addData("1",colorSpinner.place1);
        telemetry.addData("2",colorSpinner.place2);
        telemetry.addData("3",colorSpinner.place3);
        telemetry.addData("spin pose",colorSpinner.getPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void robotStop() {
    }
}




