package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {
    //mode
    public static boolean isAuto = true;
    // shooting pro constant
    public static int setAprilTagMode = 0;
    public static double toVelocity = 0, toYawDegree = -500, toPitchDegree = 0;
    public static double setVelocity = 0, setYawDegree = -500, setPitchDegree = 0;
    public static boolean setShooting = false;

    public static double ukP = 3.2, ukI = 0.13, ukD = 0.003;
    public static double dkP = 3.2, dkI = 0.13, dkD = 0.003;
    public static double spinP = 0.018, spinI = 0.08, spinD = 0.0008;
    public static double cameraP = 0.02, cameraI = 0.015, cameraD = 0.002;

    //tele op
    public static Pose InitRedCorner = new Pose(59.5, -63, Math.toRadians(0));
    public static Pose InitCenter = new Pose(0, 0, Math.toRadians(90));
    public static Pose InitBlueCorner = new Pose(-59.5, -63, Math.toRadians(180));

    //field
    public static double AutoBlueX = -66, AutoBlueY = 69, AutoRedX = 66, AutoRedY = 69, AutoMidX = 0, AutoMidY = 72;
    public static double TeleBlueX = -66, TeleBlueY = 66, TeleRedX = 66, TeleRedY = 66;

    //Auto
    public static double shootingTime = 2.5;
    public static int AprilTagNumber = 0;
    public static double AutoVelocity = 3650, toReadyShootingTime = 0.5;


    //blue pose
    public static Pose shootingBlueSeePose = new Pose(-12, 12, Math.toRadians(180));
    public static Pose shootingBluePose = new Pose(-18, 12, Math.toRadians(180));
    public static Pose startBluePose = new Pose(-50.85, 51.33, Math.toRadians(135));
    public static Pose blueRoll1 = new Pose(-54, 7.5, Math.toRadians(180));
    public static Pose blueControl2 = new Pose(-18, -12, Math.toRadians(180));
    public static Pose blueRoll2 = new Pose(-56, -12, Math.toRadians(180));
    public static Pose blueControl3 = new Pose(-12, -36, Math.toRadians(180));
    public static Pose blueRoll3 = new Pose(-56, -36, Math.toRadians(180));


    public static Pose blueLongShootingPose = new Pose(-15, -57.5, Math.toRadians(180));
    public static Pose startBlueLongPose = new Pose(-15, -57.5, Math.toRadians(90));
    public static Pose blueRoll3_2 = new Pose(-55, -36, Math.toRadians(180));
    public static Pose blueCorner = new Pose(-58, -58.5, Math.toRadians(180));


    //red pose

}
