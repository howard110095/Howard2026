package org.firstinspires.ftc.teamcode.pedroPathing.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueTeleOP", group = "Linear OpMode")
public class BlueTeleOP extends Tele {
    @Override
    protected int targetAprilTag() {
        return 2;
    }
}
