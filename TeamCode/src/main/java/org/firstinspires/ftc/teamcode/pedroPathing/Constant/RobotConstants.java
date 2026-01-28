package org.firstinspires.ftc.teamcode.pedroPathing.Constant;

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

    //shooter pid
    public static double ukP = 3.2, ukI = 0.13, ukD = 0.003, dkP = 3.2, dkI = 0.13, dkD = 0.003;

    //Offset
    public static double yawDegreeOffset = 0;

    //tele op
    public static Pose InitRedCorner = new Pose(59.5, -63, Math.toRadians(0));
    public static Pose InitCenter = new Pose(0, 0, Math.toRadians(90));
    public static Pose InitBlueCorner = new Pose(-59.5, -63, Math.toRadians(180));

    //field
    public static double AutoBlueX = -66, AutoBlueY = 69, AutoRedX = 66, AutoRedY = 69, AutoMidX = 0, AutoMidY = 72;
    public static double TeleBlueX = -66, TeleBlueY = 66, TeleRedX = 66, TeleRedY = 66;
    public static double InitUpX = 48, InitUpY = 63, InitCornerX = 59.5, InitCornerY = 63;

    //Auto
    public static double shootingTime = 2.2, toReadyShootingTime = 0.5, quickShootTime = 0.9;
    public static int AprilTagNumber = 0;

    //blue pose
    public static Pose B_P1_seeAprilTag = new Pose(-12, 12, Math.toRadians(180));
    public static Pose B_P1_shoot = new Pose(-20, 12, Math.toRadians(180));
    public static Pose B_P1_start = new Pose(-50.85, 51.33, Math.toRadians(135));
    public static Pose B_P1_R1_end = new Pose(-54, 12, Math.toRadians(180));
    public static Pose B_P1_Open1_control = new Pose(-48, 9, Math.toRadians(180));
    public static Pose B_P1_Open1_end = new Pose(-54, 4, Math.toRadians(180));
    public static Pose B_P1_R2_control = new Pose(-18, -12, Math.toRadians(180));
    public static Pose B_P1_R2_end = new Pose(-55, -12, Math.toRadians(180));
    public static Pose B_P1_R3_control = new Pose(-12, -36, Math.toRadians(180));
    public static Pose B_P1_R3_end = new Pose(-55, -36, Math.toRadians(180));

    //short shoot
    public static Pose B_P1_R1_back_control = new Pose(-48, -8, Math.toRadians(180));
    public static Pose B_P1_Open2_end = new Pose(-53.5, -6, Math.toRadians(180));
    public static Pose B_P1_catch_control = new Pose(-44,-18, Math.toRadians(180));
    public static Pose B_P1_catch_end = new Pose(-59, -18, Math.toRadians(180));
    public static Pose B_P1_stop = new Pose(-36, 0, Math.toRadians(180));

    //long shoot
    public static Pose B_P2_start = new Pose(-15, -61, Math.toRadians(180));
    public static Pose B_P2_shoot = new Pose(-15, -48, Math.toRadians(180));
    public static Pose B_P2_R3_control = new Pose(-15, -36, Math.toRadians(180));
    public static Pose B_P2_R3_end = new Pose(-55, -36, Math.toRadians(180));
    public static Pose B_P2_corner = new Pose(-58, -60, Math.toRadians(180));
    public static Pose B_P2_stop = new Pose(-58, -60, Math.toRadians(180));

    //red pose
    public static Pose R_P1_seeAprilTag = new Pose(12, 12, Math.toRadians(0));
    public static Pose R_P1_shoot = new Pose(20, 12, Math.toRadians(0));
    public static Pose R_P1_start = new Pose(50.85, 51.33, Math.toRadians(45));
    public static Pose R_P1_R1_end = new Pose(54, 12, Math.toRadians(0));
    public static Pose R_P1_Open1_control = new Pose(48, 9, Math.toRadians(0));
    public static Pose R_P1_Open1_end = new Pose(54, 4, Math.toRadians(0));
    public static Pose R_P1_R2_control = new Pose(18, -12, Math.toRadians(0));
    public static Pose R_P1_R2_end = new Pose(55, -12, Math.toRadians(0));
    public static Pose R_P1_R3_control = new Pose(12, -36, Math.toRadians(0));
    public static Pose R_P1_R3_end = new Pose(55, -36, Math.toRadians(0));

    //short shoot
    public static Pose R_P1_R1_back_control = new Pose(48, -8, Math.toRadians(0));
    public static Pose R_P1_Open2_end = new Pose(53.5, -6, Math.toRadians(0));
    public static Pose R_P1_catch_control = new Pose(44,-18, Math.toRadians(0));
    public static Pose R_P1_catch_end = new Pose(59, -18, Math.toRadians(0));
    public static Pose R_P1_stop = new Pose(36, 0, Math.toRadians(0));

    //long shoot
    public static Pose R_P2_start = new Pose(15, -61, Math.toRadians(0));
    public static Pose R_P2_shoot = new Pose(15, -48, Math.toRadians(0));
    public static Pose R_P2_R3_control = new Pose(15, -36, Math.toRadians(0));
    public static Pose R_P2_R3_end = new Pose(55, -36, Math.toRadians(0));
    public static Pose R_P2_corner = new Pose(58, -60, Math.toRadians(0));
    public static Pose R_P2_stop = new Pose(58, -60, Math.toRadians(0));

}
