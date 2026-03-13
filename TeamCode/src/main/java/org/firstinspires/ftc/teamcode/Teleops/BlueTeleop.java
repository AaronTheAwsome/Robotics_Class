package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueTeleop.j")
public class BlueTeleop extends org.firstinspires.ftc.teamcode.Teleops.TeleOp {

    @Override
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
}
