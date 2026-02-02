package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;

@Configurable
@Autonomous(name = "AutoTest", group = "Examples")
public class AutoTest extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path park;
    private PathChain front, back;
    private Pose start = new Pose(0, 0, Math.toRadians(90));
    private Pose end = new Pose(0, 24, Math.toRadians(90));

    public void buildPaths() {
        front = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();

        back = follower.pathBuilder()
                .addPath(new BezierLine(end, start))
                .setLinearHeadingInterpolation(end.getHeading(), start.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(front, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (!follower.isBusy()) {
                follower.startTeleopDrive();     // ← 進推牆模式：manualDrive=true
                pathTimer.resetTimer();          // ← 開始計時推牆
                setPathState(2);
            }
        } else if (pathState == 2) {
            follower.setTeleOpDrive(0.35, 0, 0, true);
            if (pathTimer.getElapsedTimeSeconds() > 2.0) {  // ← 你要推幾秒就填幾秒
                follower.breakFollowing();                  // 停止推牆
                setPathState(3);
            }
        }else if (pathState == 3) {
            follower.followPath(back, true);
            setPathState(4);
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

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);

//        draw();
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
        follower.setStartingPose(start);
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

