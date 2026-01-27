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
@Autonomous(name = "BlueP1", group = "Examples")
public class BlueP1 extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, time = 0;
    private Path park;
    private PathChain path0, path1roll, path2roll, pathToTakeBall, pathEnd;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(startBluePose, shootingBluePose))
                .setLinearHeadingInterpolation(startBluePose.getHeading(), shootingBluePose.getHeading())
                .build();

        path1roll = follower.pathBuilder()
                .addPath(new BezierLine(shootingBluePose, blueRoll1))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueRoll1.getHeading())
                .addPath(new BezierCurve(blueRoll1, blueOpenControl1, blueOpenGate1))
                .setLinearHeadingInterpolation(blueRoll1.getHeading(), blueOpenGate1.getHeading())
                .addPath(new BezierLine(blueOpenGate1, shootingBluePose))
                .setLinearHeadingInterpolation(blueOpenGate1.getHeading(), shootingBluePose.getHeading())
                .build();

        path2roll = follower.pathBuilder()
                .addPath(new BezierCurve(shootingBluePose, blueControl2, blueRoll2))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueRoll2.getHeading())
                .addPath(new BezierCurve(blueRoll2, blueOpenControl2, blueOpenGate1))
                .setLinearHeadingInterpolation(blueRoll2.getHeading(), blueOpenGate1.getHeading())
                .addPath(new BezierLine(blueOpenGate1, shootingBluePose))
                .setLinearHeadingInterpolation(blueOpenGate1.getHeading(), shootingBluePose.getHeading())
                .build();

        pathToTakeBall = follower.pathBuilder()
                .addPath(new BezierLine(shootingBluePose, blueOpenGate))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueOpenGate.getHeading())
                .addPath(new BezierLine(blueOpenGate, blueCatchFromGate))
                .setLinearHeadingInterpolation(blueOpenGate.getHeading(), blueCatchFromGate.getHeading())
                .addPath(new BezierLine(blueCatchFromGate, shootingBluePose))
                .setLinearHeadingInterpolation(blueCatchFromGate.getHeading(), shootingBluePose.getHeading())
                .build();

        pathEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootingBluePose, blueCatchFromGate))
                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueCatchFromGate.getHeading())
                .addPath(new BezierLine(blueCatchFromGate, shootingBluePose))
                .setLinearHeadingInterpolation(blueCatchFromGate.getHeading(), shootingBluePose.getHeading())
                .build();

//        pathToGate = follower.pathBuilder()
//                .addPath(new BezierLine(shootingBluePose, blueOpenGate))
//                .setLinearHeadingInterpolation(shootingBluePose.getHeading(), blueOpenGate.getHeading())
//                .addPath(new BezierLine(blueOpenGate, blueCatchFromGate))
//                .setLinearHeadingInterpolation(blueOpenGate.getHeading(), blueCatchFromGate.getHeading())
//                .build();
//
//        pathBackGate = follower.pathBuilder()
//                .addPath(new BezierLine(blueCatchFromGate, shootingBluePose))
//                .setLinearHeadingInterpolation(blueCatchFromGate.getHeading(), shootingBluePose.getHeading())
//                .build();


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
            if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(11);
            }
        }
        //to 1 roll
        else if (pathState == 11) {
            if (!follower.isBusy()) {
                follower.followPath(path1roll, true);
                setPathState(12);
            }
        } else if (pathState == 12) {
            if (follower.getPose().getX() < -40) setPathState(13);
        } else if (pathState == 13) {
            if (follower.getPose().getX() > -24) {
                setShooting = true;
                colorSpinner.on();
                setPathState(14);
            }
        } else if (pathState == 14) {
            if (!follower.isBusy()) setPathState(15);
        } else if (pathState == 15) {
            if (pathTimer.getElapsedTimeSeconds() >= quickShootTime) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(21);
            }
        }
        //to 2 roll
        else if (pathState == 21) {
            if (!follower.isBusy()) {
                follower.followPath(path2roll, true);
                setPathState(22);
            }
        } else if (pathState == 22) {
            if (follower.getPose().getX() < -40) setPathState(23);
        } else if (pathState == 23) {
            if (follower.getPose().getX() > -24) {
                setShooting = true;
                colorSpinner.on();
                setPathState(24);
            }
        } else if (pathState == 24) {
            if (!follower.isBusy()) setPathState(25);
        } else if (pathState == 25) {
            if (pathTimer.getElapsedTimeSeconds() >= quickShootTime) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(31);
            }
        }
        //to open gate
        else if (pathState == 31 && time <= 1) {
            if (!follower.isBusy()) {
                follower.followPath(pathToTakeBall, true);
                setPathState(32);
            }
        } else if (pathState == 32) {
            if (follower.getPose().getX() < -40) setPathState(33);
        } else if (pathState == 33) {
            if (follower.getPose().getX() > -24) {
                setShooting = true;
                colorSpinner.on();
                setPathState(34);
            }
        } else if (pathState == 34) {
            if (!follower.isBusy()) setPathState(35);
        } else if (pathState == 35) {
            if (pathTimer.getElapsedTimeSeconds() >= quickShootTime) {  //shooting time
                setShooting = false;
                colorSpinner.slowMode();
                time++;
                setPathState(31);
            }
        }
        //end
        else if (pathState == 31 && time == 2) {
            if (!follower.isBusy()) {
                follower.followPath(pathEnd, true);
                setPathState(41);
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

