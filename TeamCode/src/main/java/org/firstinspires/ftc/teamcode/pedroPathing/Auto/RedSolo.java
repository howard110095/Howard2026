package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotConstants.R_P1_shoot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constant.RobotBase;

@Configurable
@Autonomous(name = "RedSolo", group = "Examples")
public class RedSolo extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path park;
    private PathChain path0, path1_go, path1_back, path2, path3, path4;
    public static boolean spinDETECT = false;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(R_P1_start, R_P1_seeAprilTag))
                .setLinearHeadingInterpolation(R_P1_start.getHeading(), R_P1_seeAprilTag.getHeading())
                .build();

        path1_go = follower.pathBuilder()
                .addPath(new BezierLine(R_P1_seeAprilTag, R_P1_R1_end))
                .setLinearHeadingInterpolation(R_P1_seeAprilTag.getHeading(), R_P1_R1_end.getHeading())
                .build();

        path1_back = follower.pathBuilder()
                .addPath(new BezierLine(R_P1_R1_end,R_P1_shoot))
                .setLinearHeadingInterpolation(R_P1_R1_end.getHeading(), R_P1_shoot.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(R_P1_shoot, R_P1_R2_control, R_P1_R2_end))
                .setLinearHeadingInterpolation(R_P1_shoot.getHeading(),R_P1_R2_end.getHeading())
                .addPath(new BezierCurve(R_P1_R2_end, R_P1_R2_control, R_P1_shoot))
                .setLinearHeadingInterpolation(R_P1_R2_end.getHeading(), R_P1_shoot.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(R_P1_shoot, R_P1_R3_control, R_P1_R3_end))
                .setLinearHeadingInterpolation(R_P1_shoot.getHeading(),R_P1_R3_end.getHeading())
                .addPath(new BezierLine(R_P1_R3_end, R_P1_shoot))
                .setLinearHeadingInterpolation(R_P1_R3_end.getHeading(),R_P1_shoot.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(R_P1_shoot,R_P1_stop))
                .setLinearHeadingInterpolation(R_P1_shoot.getHeading(), R_P1_stop.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            setShooting = false;
            follower.followPath(path0, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (!follower.isBusy()) {
                setPathState(2);
            }
        } else if (pathState == 2) {
            colorSpinner.on();
            setShooting = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(10);
        }
        //to 1 roll
        else if (pathState == 10) {
            if (pathTimer.getElapsedTimeSeconds() > 0.3) setPathState(11);
        } else if (pathState == 11) {
            if (!follower.isBusy()) {
//                intake.on();
                setShooting = false;
                colorSpinner.slowMode();
                follower.followPath(path1_go, true);
                colorSpinner.colorRenew();
                setPathState(12);
            }
        } else if (pathState == 12) {
            if (!follower.isBusy())  setPathState(13);
        } else if (pathState == 13) {
            if (pathTimer.getElapsedTimeSeconds() > 1.2) setPathState(14);
        } else if (pathState == 14) {
            spinDETECT = true;
            if (!(21 <= AprilTagNumber && AprilTagNumber <= 23)) AprilTagNumber = 21;
//            intake.on();
            follower.followPath(path1_back, true);
            setPathState(15);
        } else if (pathState == 15) {
            if (follower.getPose().getX() < 40) {
                setPathState(16);
            }
        } else if (pathState == 16) {
            if (!follower.isBusy()) {
//                intake.off();
                setPathState(17);
            }
        } else if (pathState == 17) {
            if (pathTimer.getElapsedTimeSeconds() >= 0.1) setPathState(18);
        } else if (pathState == 18) {
//            intake.off();
            spinDETECT = false;
            colorSpinner.logic(AprilTagNumber);
            if (pathTimer.getElapsedTimeSeconds() >= toReadyShootingTime) setPathState(19);
        } else if (pathState == 19) {
            colorSpinner.on();
            setShooting = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(21);
        }
        //to 2 roll
        else if (pathState == 21) {
            if (!follower.isBusy()) {
//                intake.on();
                setShooting = false;
                colorSpinner.slowMode();
                follower.followPath(path2, true);
                colorSpinner.colorRenew();
                setPathState(22);
            }
        } else if (pathState == 22) {
            if (follower.getPose().getX() > 53) setPathState(23);
        } else if (pathState == 23) {
            if (follower.getPose().getX() < 50) {
                spinDETECT = true;
                setPathState(24);
            }
        } else if (pathState == 24) {
            if (!follower.isBusy()) {
//                intake.off();
                setPathState(26);
            }
        } else if (pathState == 26) {
            if (pathTimer.getElapsedTimeSeconds() >= 0.1) setPathState(27);
        } else if (pathState == 27) {
//            intake.off();
            spinDETECT = false;
            colorSpinner.logic(AprilTagNumber);
            if (pathTimer.getElapsedTimeSeconds() >= toReadyShootingTime) setPathState(28);
        } else if (pathState == 28) {
            colorSpinner.on();
            setShooting = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(31);
        }
        //to 3 roll
        else if (pathState == 31) {
            if (!follower.isBusy()) {
//                intake.on();
                setShooting = false;
                colorSpinner.slowMode();
                follower.followPath(path3, true);
                colorSpinner.colorRenew();
                setPathState(32);
            }
        } else if (pathState == 32) {
            if (follower.getPose().getX() > 53) setPathState(33);
        } else if (pathState == 33) {
            if (follower.getPose().getX() < 50) {
                spinDETECT = true;
                setPathState(34);
            }
        } else if (pathState == 34) {
            if (!follower.isBusy()) {
                setPathState(36);
            }
        } else if (pathState == 36) {
            if (pathTimer.getElapsedTimeSeconds() >= 0.1) setPathState(37);
        } else if (pathState == 37) {
//            intake.off();
            spinDETECT = false;
            colorSpinner.logic(AprilTagNumber);
            if (pathTimer.getElapsedTimeSeconds() >= toReadyShootingTime) setPathState(38);
        } else if (pathState == 38) {
            colorSpinner.on();
            setShooting = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(41);
        } else if (pathState == 41) {
            if (!follower.isBusy()) {
                setShooting = false;
                follower.followPath(path4, true);
                setPathState(-1);
            }
        } else {
            setShooting = false;
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
        if (10 <= pathState && pathState <= 13 && !(21 <= AprilTagNumber && AprilTagNumber <= 23)) {
            setAprilTagMode = 1;
            AprilTagNumber = shooter.tagNumber();
        } else setAprilTagMode = 2;

        shooter.shootingPRO(setAprilTagMode, setVelocity, setYawDegree, setPitchDegree, setShooting);

        if (spinDETECT) colorSpinner.detect3posePRO();


        telemetry.addData("number", AprilTagNumber);
        telemetry.addData("1", colorSpinner.place1);
        telemetry.addData("2", colorSpinner.place2);
        telemetry.addData("3", colorSpinner.place3);
        telemetry.addData("getTx", shooter.limelight.getLatestResult().getTx());
        telemetry.addData("detect color", spinDETECT);
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
        follower.setStartingPose(R_P1_start);
        buildPaths();

        setAprilTagMode = 2;
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

