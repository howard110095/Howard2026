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
@Autonomous(name = "BlueSolo", group = "Examples")
public class BlueSolo extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path park;
    private PathChain path0, path1_go, path1_back, path2, path3, path4;
    public static boolean spinDETECT = false;
    public double close = 0;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_start, B_P1_seeAprilTag))
                .setLinearHeadingInterpolation(B_P1_start.getHeading(), B_P1_seeAprilTag.getHeading())
                .build();

        path1_go = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_seeAprilTag, B_P1_R1_end))
                .setLinearHeadingInterpolation(B_P1_seeAprilTag.getHeading(), B_P1_R1_end.getHeading())
                .addPath(new BezierLine(B_P1_R1_end, B_P1_Open1_end))
                .setLinearHeadingInterpolation(B_P1_R1_end.getHeading(), B_P1_Open1_end.getHeading())
                .build();

        path1_back = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), B_P1_shoot))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), B_P1_shoot.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(B_P1_shoot, B_P1_R2_control, B_P1_R2_end))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_R2_end.getHeading())
                .addPath(new BezierCurve(B_P1_R2_end, B_P1_R2_control, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_R2_end.getHeading(), B_P1_shoot.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(B_P1_shoot, B_P1_R3_control, B_P1_R3_end))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_R3_end.getHeading())
                .addPath(new BezierLine(B_P1_R3_end, B_P1_shoot))
                .setLinearHeadingInterpolation(B_P1_R3_end.getHeading(), B_P1_shoot.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(B_P1_shoot, B_P1_stop))
                .setLinearHeadingInterpolation(B_P1_shoot.getHeading(), B_P1_stop.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
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
                setPathState(10);
            }
        }
        //to 1 roll
        else if (pathState == 10) { // see tag
            if (pathTimer.getElapsedTimeSeconds() > 0.3) setPathState(11);
        } else if (pathState == 11) {
            if (!follower.isBusy()) {
                setShooting = false;
                follower.followPath(path1_go, 0.7, true);
                colorSpinner.colorRenew();
                setPathState(12);
            }
        } else if (pathState == 12) {
            if (!follower.isBusy()) {
                follower.startTeleopDrive();
                setPathState(13);
            }
        } else if (pathState == 13) {
            follower.setTeleOpDrive(0.2, 0, 0, true);
            if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                follower.breakFollowing();
                setPathState(14);
            }
        } else if (pathState == 14) {
            colorSpinner.slowMode();
            if (!(21 <= AprilTagNumber && AprilTagNumber <= 23)) AprilTagNumber = 21;
            follower.followPath(path1_back, true);
            setPathState(15);
        } else if (pathState == 15) {
            if (follower.getPose().getX() > -28) {
                close = colorSpinner.close(colorSpinner.getDegree());
                spinDETECT = true;
                setPathState(16);
            }
        } else if (pathState == 16) {
            if (!follower.isBusy()) setPathState(17);
        } else if (pathState == 17) {
            if (pathTimer.getElapsedTimeSeconds() >= waitingTime) setPathState(18);
        } else if (pathState == 18) {
            spinDETECT = false;
            colorSpinner.logic(AprilTagNumber);
            if (pathTimer.getElapsedTimeSeconds() >= toReadyShootingTime) setPathState(19);
        } else if (pathState == 19) {
            colorSpinner.classify();
            setShooting = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(21);
        }
        //to 2 roll
        else if (pathState == 21) {
            if (!follower.isBusy()) {
                setShooting = false;
                colorSpinner.slowMode();
                follower.followPath(path2, 0.7, true);
                colorSpinner.colorRenew();
                setPathState(22);
            }
        } else if (pathState == 22) {
            if (follower.getPose().getX() < -50) setPathState(23);
        } else if (pathState == 23) {
            if (follower.getPose().getX() > -22.5) {
                close = colorSpinner.close(colorSpinner.getDegree());
                spinDETECT = true;
                setPathState(24);
            }
        } else if (pathState == 24) {
            if (!follower.isBusy()) setPathState(26);
        } else if (pathState == 26) {
            if (pathTimer.getElapsedTimeSeconds() >= waitingTime + 0.2) setPathState(27);
        } else if (pathState == 27) {
            spinDETECT = false;
            colorSpinner.logic(AprilTagNumber);
            if (pathTimer.getElapsedTimeSeconds() >= toReadyShootingTime) setPathState(28);
        } else if (pathState == 28) {
            colorSpinner.classify();
            setShooting = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(31);
        }
        //to 3 roll
        else if (pathState == 31) {
            if (!follower.isBusy()) {
                setShooting = false;
                colorSpinner.slowMode();
                follower.followPath(path3, 0.8, true);
                colorSpinner.colorRenew();
                setPathState(32);
            }
        } else if (pathState == 32) {
            if (follower.getPose().getX() < -50) setPathState(33);
        } else if (pathState == 33) {
            if (follower.getPose().getX() > -28) {
                close = colorSpinner.close(colorSpinner.getDegree());
                spinDETECT = true;
                setPathState(34);
            }
        } else if (pathState == 34) {
            if (!follower.isBusy()) setPathState(36);
        } else if (pathState == 36) {
            if (pathTimer.getElapsedTimeSeconds() >= waitingTime) setPathState(37);
        } else if (pathState == 37) {
            spinDETECT = false;
            colorSpinner.logic(AprilTagNumber);
            if (pathTimer.getElapsedTimeSeconds() >= toReadyShootingTime) setPathState(38);
        } else if (pathState == 38) {
            colorSpinner.classify();
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

        if (spinDETECT) colorSpinner.detect3posePRO(close);


        telemetry.addData("number", AprilTagNumber);
        telemetry.addData("close pose", close);
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
        follower.setStartingPose(B_P1_start);
        buildPaths();
        autoYawOffset = -6;
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
        isAuto = true;
        foot.robotDown();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void robotStop() {

    }
}

