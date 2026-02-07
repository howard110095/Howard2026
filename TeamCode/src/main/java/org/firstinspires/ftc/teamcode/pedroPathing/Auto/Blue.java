package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;

@Configurable
@Autonomous(name = "Blue", group = "Examples")
public class Blue extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, time = 0;
    private Path park;
    private PathChain path0, path1, pathToTakeBall;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(B_P2_start, B_P2_shoot))
                .setLinearHeadingInterpolation(B_P2_start.getHeading(), B_P2_shoot.getHeading())
                .build();

//        path1 = follower.pathBuilder()
//                .addPath(new BezierCurve(B_P2_shoot, B_P2_R3_control, B_P2_R3_end))
//                .setLinearHeadingInterpolation(B_P2_shoot.getHeading(), B_P2_R3_end.getHeading())
//                .addPath(new BezierLine(B_P2_R3_end, B_P2_shoot))
//                .setLinearHeadingInterpolation(B_P2_R3_end.getHeading(), B_P2_shoot.getHeading())
//                .build();

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(B_P2_shoot, B_P2_stop))
                .setLinearHeadingInterpolation(B_P2_shoot.getHeading(), B_P2_stop.getHeading())
                .build();


//
//        .
//        addPath(new BezierCurve(new Point(preloadPose), new Point(15, 36, Point.CARTESIAN), new Point(61, 36.25, Point.CARTESIAN), new Point(59, 26.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(preloadPose.getHeading(), Math.toRadians(180))
//                .addPath(new BezierLine(new Point(59.000, 26.000, Point.CARTESIAN), new Point(28, 26.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))


//        park = new Path(new BezierCurve(scorePose, parkControlPose, parkPose));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(path0, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (!follower.isBusy()) setPathState(2);
        } else if (pathState == 2) {
            if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                setShooting = true;
                colorSpinner.slowMode();
                setPathState(3);
            }
        } else if (pathState == 3) {
            if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(11);
            }
        }
        //stop
        else if (pathState == 11) {
            if (!follower.isBusy()) {
                follower.followPath(path1, true);
                setPathState(12);
            }
        }
        //to 3 roll
//        else if (pathState == 31) {
//            if (!follower.isBusy()) {
//                setShooting = false;
//                colorSpinner.slowMode();
//                follower.followPath(path3, true);
//                setPathState(32);
//            }
//        } else if (pathState == 32) {
//            if (!follower.isBusy()) {
//                intake.off();
//                colorSpinner.slowMode();
//                setShooting = true;
//                setPathState(33);
//            }
//
//        } else if (pathState == 33) {
//            if (pathTimer.getElapsedTimeSeconds() >= 1.5) setPathState(34);
//        }
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
//        intake.on();
        shooter.shootingPRO(2, setVelocity, setYawDegree, setPitchDegree, setShooting);
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
        follower.setStartingPose(B_P2_start);
        buildPaths();
    }

    @Override
    public void robotInitLoop() {
    }

    @Override
    public void robotStart() {
        isAuto = true;
        foot.robotDown();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void robotStop() {
    }
}

