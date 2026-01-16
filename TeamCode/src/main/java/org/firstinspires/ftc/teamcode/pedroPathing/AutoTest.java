package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

@Configurable
@Autonomous(name = "AutoTest", group = "Examples")
public class AutoTest extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path park;
    private PathChain path0,path00, path1, path2, path3;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(startBlueLongPose, shootingBluePose))
                .setLinearHeadingInterpolation(startBlueLongPose.getHeading(), shootingBluePose.getHeading())
                .build();

        path00= follower.pathBuilder()
                .addPath(new BezierLine(startBlueLongPose, shootingBluePose))
                .setLinearHeadingInterpolation(startBlueLongPose.getHeading(), shootingBluePose.getHeading())
                .build();

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(shootingBluePose, blueRoll1))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueRoll1.getHeading())
                .addPath(new BezierLine(blueRoll1, shootingBluePose))
                .setLinearHeadingInterpolation(blueRoll1.getHeading(), shootingBluePose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootingBluePose, blueControl2, blueRoll2))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueRoll2.getHeading())
                .addPath(new BezierCurve(blueRoll2, blueControl2, shootingBluePose))
                .setLinearHeadingInterpolation(blueRoll2.getHeading(), shootingBluePose.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootingBluePose, blueControl3, blueRoll3))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueRoll3.getHeading())
                .addPath(new BezierCurve(blueRoll3, blueControl3, shootingBluePose))
                .setLinearHeadingInterpolation(blueRoll3.getHeading(), shootingBluePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(path0, true);
            setPathState(1);
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void robotLoop() {
        follower.update();
        autonomousPathUpdate();
        //vision
        if (pathState == 12 && follower.getPose().getX() < -30) shooter.visionTracking(1);
        else shooter.visionTracking(2);
        // These loop the movements of the robot

        telemetry.addData("uVelocity", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("uVelocity", shooter.uVelocity);
        telemetry.addData("dVelocity", shooter.dVelocity);
        telemetry.addData("dVelocity", shooter.shooterVelocity);
        telemetry.addData("shooterU_power", shooter.shooterU_power);
        telemetry.addData("shooterD_power", shooter.shooterD_power);
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void robotInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
//        startingPose = startBluePose;
        follower.setStartingPose(startBlueLongPose);
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void robotInitLoop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void robotStart() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void robotStop() {
    }
}

