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
@Autonomous(name = "Auto1", group = "Examples")
public class Auto1 extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path park;
    private PathChain path0, path1, path2, path3;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(startBluePose, shootingBluePose))
                .setLinearHeadingInterpolation(startBluePose.getHeading(), shootingBluePose.getHeading())
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
            intake.off();
            colorSpinner.on(0.4);
            shooter.shooting(2, false);
            follower.followPath(path0, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (follower.getPose().getY() < 35) {
                shooter.shooting(2, true);
                setPathState(2);
            }
        } else if (pathState == 2) {
            if (!follower.isBusy()) {
                setPathState(3);
            }
        } else if (pathState == 3) {
            if (pathTimer.getElapsedTimeSeconds() >= 1) setPathState(11);
        }
        //to 1 roll
        else if (pathState == 11) {
            if (!follower.isBusy()) {
                intake.on();
                shooter.shooting(2, false);
                colorSpinner.on(0.18);
                follower.followPath(path1, true);
                setPathState(12);
            }
        } else if (pathState == 12) {
            if (!follower.isBusy()) {
                intake.off();
                colorSpinner.on(0.4);
                shooter.shooting(2, true);
                setPathState(13);
            }
        } else if (pathState == 13) {
            if (pathTimer.getElapsedTimeSeconds() >= 1.5) setPathState(21);
        }
        //to 2 roll
        else if (pathState == 21) {
            if (!follower.isBusy()) {
                intake.on();
                shooter.shooting(2, false);
                colorSpinner.on(0.18);
                follower.followPath(path2, true);
                setPathState(22);
            }
        } else if (pathState == 22) {
            if (!follower.isBusy()) {
                intake.off();
                colorSpinner.on(0.4);
                shooter.shooting(2, true);
                setPathState(23);
            }

        } else if (pathState == 23) {
            if (pathTimer.getElapsedTimeSeconds() >= 1.5) setPathState(31);
        }
//        //to 3 roll
        else if (pathState == 31) {
            if (!follower.isBusy()) {
                intake.on();
                shooter.shooting(2, false);
                colorSpinner.on(0.18);
                follower.followPath(path3, true);
                setPathState(32);
            }
        } else if (pathState == 32) {
            if (!follower.isBusy()) {
                intake.off();
                colorSpinner.on(0.4);
                shooter.shooting(2, true);
                setPathState(33);
            }

        } else if (pathState == 33) {
            if (pathTimer.getElapsedTimeSeconds() >= 1.5) setPathState(34);
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
        shooter.visionTracking(2);
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
        follower.setStartingPose(startBluePose);
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

