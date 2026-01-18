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
@Autonomous(name = "Auto", group = "Examples")
public class Auto extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path park;
    private PathChain path0, path1_go, path1_back, path2, path3;
    public static boolean spinDETECT = false, shoot = false;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(startBluePose, shootingBlueSeePose))
                .setLinearHeadingInterpolation(startBluePose.getHeading(), shootingBlueSeePose.getHeading())
                .build();

        path1_go = follower.pathBuilder()
                .addPath(new BezierLine(shootingBlueSeePose, blueRoll1))
                .setLinearHeadingInterpolation(shootingBlueSeePose.getHeading(), blueRoll1.getHeading())
                .build();

        path1_back = follower.pathBuilder()
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
//                .addPath(new BezierCurve(blueRoll3, blueControl3, shootingBluePose))
//                .setLinearHeadingInterpolation(blueRoll3.getHeading(), shootingBluePose.getHeading())
                .addPath(new BezierLine(blueRoll3, shootingBluePose))
                .setLinearHeadingInterpolation(blueRoll3.getHeading(), shootingBluePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
//            intake.off();
            shooter.toDegree(-55);
            shoot = false;
            follower.followPath(path0, true);
            setPathState(1);
        } else if (pathState == 1) {
            if (!follower.isBusy()) {
                setPathState(2);
            }
        } else if (pathState == 2) {
            colorSpinner.on(1);
            shoot = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(11);
        }
        //to 1 roll
        else if (pathState == 11) {
            if (!follower.isBusy()) {
//                intake.on();
                shoot = false;
                colorSpinner.on(0.18);
                follower.followPath(path1_go, true);
                colorSpinner.colorRenew();
                setPathState(12);
            }
        } else if (pathState == 12) {
            if (!follower.isBusy()) {
                setPathState(13);
            }
        } else if (pathState == 13) {
            if (pathTimer.getElapsedTimeSeconds() > 1.7) setPathState(14);
        } else if (pathState == 14) {
            shooter.toDegree(-60);
            if (AprilTagNumber == 0) AprilTagNumber = 21;
//            intake.on();
            follower.followPath(path1_back, true);
            setPathState(15);
        } else if (pathState == 15) {
            if (follower.getPose().getX() > -40) {
                spinDETECT = true;
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
            colorSpinner.on(1);
            shoot = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(21);
        }
        //to 2 roll
        else if (pathState == 21) {
            if (!follower.isBusy()) {
//                intake.on();
                shoot = false;
                colorSpinner.on(0.18);
                follower.followPath(path2, true);
                colorSpinner.colorRenew();
                setPathState(22);
            }
        } else if (pathState == 22) {
            if (follower.getPose().getX() < -53) setPathState(23);
        } else if (pathState == 23) {
            if (follower.getPose().getX() > -50) {
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
            colorSpinner.on(1);
            shoot = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(31);
        }
        //to 3 roll
        else if (pathState == 31) {
            if (!follower.isBusy()) {
//                intake.on();
                shoot = false;
                colorSpinner.on(0.18);
                follower.followPath(path3, true);
                colorSpinner.colorRenew();
                setPathState(32);
            }
        } else if (pathState == 32) {
            if (follower.getPose().getX() < -53) setPathState(33);
        } else if (pathState == 33) {
            if (follower.getPose().getX() > -50) {
                spinDETECT = true;
                setPathState(34);
            }
        } else if (pathState == 34) {
            if (!follower.isBusy()) {
//                intake.off();
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
            colorSpinner.on(1);
            shoot = true;
            if (pathTimer.getElapsedTimeSeconds() >= shootingTime) setPathState(-1);
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
        shooter.pitchDegree(43);
        if (pathState <= 11) shooter.toDegree(-55);
        else if ((pathState == 12 || pathState == 13) && AprilTagNumber == 0) {
//            shooter.visionTracking(1);
            shooter.toDegree(-90);
            AprilTagNumber = shooter.tagNumber();
        } else if (pathState >= 14) shooter.toDegree(-60);

        shooter.setVelocity(AutoVelocity, shoot);
//        else shooter.visionTracking(2);
//        else shooter.toDegree(-55);
        // These loop the movements of the robot

        if (spinDETECT) {
            colorSpinner.detect3posePRO();
        }


        telemetry.addData("number", AprilTagNumber);
        telemetry.addData("1", colorSpinner.place1);
        telemetry.addData("2", colorSpinner.place2);
        telemetry.addData("3", colorSpinner.place3);
        telemetry.addData("getTx", shooter.limelight.getLatestResult().getTx());
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

