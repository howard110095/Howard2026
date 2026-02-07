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
    private PathChain path0, pathR1, pathR2, pathToTakeBall, pathEnd;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_start, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_start.getHeading(), B_P1_shoot.getHeading())
                .build();

        pathR1 = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_shoot, B_P1_R1_end))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_R1_end.getHeading())
                .addPath(new BezierCurve(B_P1_R1_end, B_P1_Open1_control, B_P1_Open1_end))
                .setLinearHeadingInterpolation(B_P1_R1_end.getHeading(), B_P1_Open1_end.getHeading())
                .addPath(new BezierLine(B_P1_Open1_end, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_Open1_end.getHeading(), B_P1_shoot.getHeading())
                .build();

        pathR2 = follower.pathBuilder()
                .addPath(new BezierCurve(B_P1_shoot, B_P1_R2_control, B_P1_R2_end))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_R2_end.getHeading())
                .addPath(new BezierCurve(B_P1_R2_end, B_P1_R1_back_control, B_P1_Open2_back_end))
                .setLinearHeadingInterpolation(B_P1_R2_end.getHeading(), B_P1_Open2_back_end.getHeading())
                .addPath(new BezierLine(B_P1_Open2_back_end, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_Open2_back_end.getHeading(), B_P1_shoot.getHeading())
                .build();

        pathToTakeBall = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_shoot, B_P1_Open2_end))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_Open2_end.getHeading())
                .addPath(new BezierCurve(B_P1_Open2_end, B_P1_catch_control,B_P1_catch_end))
                .setLinearHeadingInterpolation(B_P1_Open2_end.getHeading(), B_P1_catch_end.getHeading())
                .addPath(new BezierLine(B_P1_catch_end, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_catch_end.getHeading(), B_P1_shoot.getHeading())
                .build();

        pathEnd = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_shoot, B_P1_catch_end))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_catch_end.getHeading())
                .addPath(new BezierLine(B_P1_catch_end, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_catch_end.getHeading(), B_P1_shoot.getHeading())
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
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            velocityOffset = 150;
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
            velocityOffset = 0;
            if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(11);
            }
        }
        //to 1 roll
        else if (pathState == 11) {
            if (!follower.isBusy()) {
                follower.followPath(pathR1, true);
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
                follower.followPath(pathR2, true);
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
        }else if (pathState == 41) {
            if (follower.getPose().getX() < -40) setPathState(42);
        } else if (pathState == 42) {
            if (follower.getPose().getX() > -24) {
                setShooting = true;
                colorSpinner.on();
                setPathState(43);
            }
        } else if (pathState == 43) {
            if (!follower.isBusy()) setPathState(44);
        } else if (pathState == 44) {
            if (pathTimer.getElapsedTimeSeconds() >= quickShootTime) {
                setShooting = false;
                colorSpinner.slowMode();
                setPathState(45);
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
        shooter.shootingPRO(2, setVelocity, -60, setPitchDegree, setShooting);
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
        follower.setStartingPose(B_P1_start);
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

