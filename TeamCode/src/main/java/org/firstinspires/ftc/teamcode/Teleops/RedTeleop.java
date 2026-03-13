package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RedTeleop.j")
public class RedTeleop extends org.firstinspires.ftc.teamcode.Teleops.TeleOp{
    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
}