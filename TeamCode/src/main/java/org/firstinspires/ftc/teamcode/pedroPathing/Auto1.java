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
    private int pathState, time = 0;
    private Path park;
    private PathChain path0, path1, pathToGate, pathBackGate;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(startBluePose, shootingBlueSeePose))
                .setLinearHeadingInterpolation(startBluePose.getHeading(), shootingBlueSeePose.getHeading())
                .build();

        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootingBlueSeePose, blueControl2, blueRoll2))
                .setLinearHeadingInterpolation(shootingBlueSeePose.getHeading(), blueRoll2.getHeading())
                .addPath(new BezierCurve(blueRoll2, blueControl2, shootingBluePose))
                .setLinearHeadingInterpolation(blueRoll2.getHeading(), shootingBluePose.getHeading())
                .build();

        pathToGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootingBluePose, blueToGateControl2, blueOpenGate))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueOpenGate.getHeading())
                .addPath(new BezierLine(blueOpenGate, blueCatchFromGate))
                .setLinearHeadingInterpolation(blueOpenGate.getHeading(), blueCatchFromGate.getHeading())
                .build();

        pathBackGate = follower.pathBuilder()
                .addPath(new BezierCurve(blueCatchFromGate, blueBackFromGateControl, shootingBluePose))
                .setLinearHeadingInterpolation(blueCatchFromGate.getHeading(), shootingBluePose.getHeading())
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
            colorSpinner.slowMode();
            setShooting = false;
            follower.followPath(path0, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (follower.getPose().getY() < 23) {
                setShooting = true;
                colorSpinner.on();
                setPathState(2);
            }
        } else if (pathState == 2) {
            if (!follower.isBusy()) setPathState(3);
        } else if (pathState == 3) {
            if (pathTimer.getElapsedTimeSeconds() >= 1.2) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(11);
            }
        }
        //to 1 roll
        else if (pathState == 11) {
            if (!follower.isBusy()) {
                follower.followPath(path1, true);
                setPathState(12);
            }
        } else if (pathState == 12) {
            if (follower.getPose().getX() < -40) setPathState(13);
        } else if (pathState == 13) {
            if (follower.getPose().getX() > -30) setPathState(14);
        } else if (pathState == 14) {
            if (follower.getPose().getY() > 0) {
                setShooting = true;
                colorSpinner.on();
                setPathState(15);
            }
        } else if (pathState == 15) {
            if (!follower.isBusy()) setPathState(16);
        } else if (pathState == 16) {
            if (pathTimer.getElapsedTimeSeconds() >= quickShootTime) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(21);
            }
        }
        //to 2 roll
        else if (pathState == 21 && time <= 2) {
            if (!follower.isBusy()) {
                follower.followPath(pathToGate, true);
                setPathState(22);
            }
        } else if (pathState == 22) {
            if (!follower.isBusy()) setPathState(23);
        } else if (pathState == 23) {
            if (pathTimer.getElapsedTimeSeconds() >= 0.5) setPathState(24);
        } else if (pathState == 24) {
            if (!follower.isBusy()) {
                follower.followPath(pathBackGate, true);
                setPathState(25);
            }
        } else if (pathState == 25) {
            if (follower.getPose().getY() > 0) {
                setShooting = true;
                colorSpinner.on();
                setPathState(26);
            }
        } else if (pathState == 26) {
            if (!follower.isBusy()) setPathState(27);
        } else if (pathState == 27) {
            if (pathTimer.getElapsedTimeSeconds() >= quickShootTime) {  //shooting time
                setShooting = false;
                colorSpinner.slowMode();
                time++;
                setPathState(21);
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
        intake.on();
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
        follower.setStartingPose(startBluePose);
        buildPaths();
    }

    @Override
    public void robotInitLoop() {
    }

    @Override
    public void robotStart() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void robotStop() {
    }
}

