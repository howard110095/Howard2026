package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class RobotConstants {
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
