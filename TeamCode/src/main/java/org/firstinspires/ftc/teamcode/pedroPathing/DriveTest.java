package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public abstract class DriveTest extends OpMode {
    public Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    //-----------------------------------------
    public final Pose startPose = new Pose(0, 0, 0);

    public void init(){
        //follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    public void init_loop() {
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        // 底盤遙控
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x * 0.6;
        follower.update();
        follower.setTeleOpDrive(axial, lateral, -yaw * 0.5, true);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {

    }
}
