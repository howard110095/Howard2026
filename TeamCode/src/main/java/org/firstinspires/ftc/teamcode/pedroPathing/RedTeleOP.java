package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RedTeleOP", group = "Linear OpMode")
public class RedTeleOP extends Tele {
    @Override
    protected int targetAprilTag() {
        return 0;
    }
}
