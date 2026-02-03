package org.firstinspires.ftc.teamcode.pedroPathing.Constant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {
    //mode
    public static boolean isAuto = true, FieldReverse = false;

    // shooting pro constant
    public static int setAprilTagMode = 0;
    public static double toVelocity = 0, toYawDegree = -500, toPitchDegree = 0;
    public static double setVelocity = 0, setYawDegree = -500, setPitchDegree = 0;
    public static double MaxUpVelocity = 4800, MaxDownVelocity = 4757;
    public static boolean setShooting = false;

    //shooter pid
    public static double ukP = 3.2, ukI = 0.1, ukD = 0.01, dkP = 3.2, dkI = 0.1, dkD = 0.01;

    //Offset
    public static double yawDegreeOffset = 0;

    //field
    public static double AutoBlueX = -66, AutoBlueY = 66, AutoRedX = 66, AutoRedY = 66, AutoMidX = 0, AutoMidY = 72;
    public static double TeleBlueX = -66, TeleBlueY = 66, TeleRedX = 66, TeleRedY = 66;
    public static double InitUpX = 31.73, InitUpY = 61.47, InitCornerX = 59.5, InitCornerY = 63;

    //Auto
    public static double shootingTime = 2.2, waitingTime = 0.2, toReadyShootingTime = 0.5, quickShootTime = 1.2;
    public static int AprilTagNumber = 0;

    //blue pose
    public static Pose B_P1_seeAprilTag = new Pose(-12, 12, Math.toRadians(180));
    public static Pose B_P1_shoot = new Pose(-20, 12, Math.toRadians(180));
    public static Pose B_P1_start = new Pose(-52.3, 50.7, Math.toRadians(140));
    public static Pose B_P1_R1_end = new Pose(-50, 12, Math.toRadians(180));
    public static Pose B_P1_Open1_control = new Pose(-48, 9, Math.toRadians(180));
    public static Pose B_P1_Open1_end = new Pose(-50, 6, Math.toRadians(180));
    public static Pose B_P1_R2_control = new Pose(-18, -12, Math.toRadians(180));
    public static Pose B_P1_R2_end = new Pose(-55, -12, Math.toRadians(180));
    public static Pose B_P1_R3_control = new Pose(-12, -36, Math.toRadians(180));
    public static Pose B_P1_R3_end = new Pose(-55, -36, Math.toRadians(180));

    //short shoot
    public static Pose B_P1_R1_back_control = new Pose(-48, -8, Math.toRadians(180));
    public static Pose B_P1_Open2_end = new Pose(-53.5, -6, Math.toRadians(180));
    public static Pose B_P1_catch_control = new Pose(-44, -18, Math.toRadians(180));
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
    public static Pose R_P1_start = new Pose(52.3, 50.7, Math.toRadians(40));
    public static Pose R_P1_R1_end = new Pose(50, 12, Math.toRadians(0));
    public static Pose R_P1_Open1_control = new Pose(48, 9, Math.toRadians(0));
    public static Pose R_P1_Open1_end = new Pose(50, 6, Math.toRadians(0));
    public static Pose R_P1_R2_control = new Pose(18, -12, Math.toRadians(0));
    public static Pose R_P1_R2_end = new Pose(55, -12, Math.toRadians(0));
    public static Pose R_P1_R3_control = new Pose(12, -36, Math.toRadians(0));
    public static Pose R_P1_R3_end = new Pose(55, -36, Math.toRadians(0));

    //short shoot
    public static Pose R_P1_R1_back_control = new Pose(48, -8, Math.toRadians(0));
    public static Pose R_P1_Open2_end = new Pose(53.5, -6, Math.toRadians(0));
    public static Pose R_P1_catch_control = new Pose(44, -18, Math.toRadians(0));
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
