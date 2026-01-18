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
    private PathChain path0, path00;

    public void buildPaths() {
//        path0 = follower.pathBuilder()
//                .addPath(new BezierLine(s0_0, s0_1))
//                .setLinearHeadingInterpolation(s0_0.getHeading(), s0_1.getHeading())
//                .build();
//
//        path00 = follower.pathBuilder()
//                .addPath(new BezierLine(follower.getPose(), s0_0))
//                .setLinearHeadingInterpolation(follower.getPose().getHeading(), s0_0.getHeading())
//                .build();


    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(path0, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.25) {
                if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                    follower.breakFollowing();
                }
                setPathState(2);
            }
        } else if (pathState == 2) {
            if (!follower.isBusy()) {
                follower.followPath(path00, true);
                setPathState(3);
            }
        } else if (pathState == 3) {
            if (!follower.isBusy()) {
                follower.breakFollowing();
                setPathState(-1);
            }
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
//        follower.setStartingPose(s0_0);
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

