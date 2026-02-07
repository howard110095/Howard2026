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
@Autonomous(name = "BlueStrafePark", group = "Examples")
public class BlueStrafePark extends RobotBase {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, time = 0;
    private PathChain path0, park;

    public void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(B_P2_start, B_P2_stop))
                .setLinearHeadingInterpolation(B_P2_start.getHeading(), B_P2_stop.getHeading())
                .build();

//        park = follower.pathBuilder()
//                .addPath(new BezierLine(B_P2_shoot, B_P2_stop))
//                .setLinearHeadingInterpolation(B_P2_shoot.getHeading(), B_P2_stop.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(path0, true);
            setPathState(1);
        }
//        else if (pathState == 1) {
//            if (!follower.isBusy()) setPathState(2);
//        } else if (pathState == 2) {
//            setShooting = true;
//            colorSpinner.on();
//            if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
//                setShooting = false;
//                colorSpinner.slowMode();
//                setPathState(3);
//            }
//        } else if (pathState == 3) {
//            if (!follower.isBusy()) {
//                follower.followPath(park, true);
//                setPathState(-1);
//            }
//        }else  if (!follower.isBusy()) {
//            follower.followPath(park, true);
//            setPathState(-1);
//        }
//            colorSpinner.spin.setPower(0);

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
//        shooter.shootingPRO(2, setVelocity, setYawDegree, setPitchDegree, setShooting);
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

