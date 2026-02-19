package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {

    private final DcMotor shooterLeft;   // myMotorE
    private final DcMotor shooterRight;  // myMotorE2

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotor.class, "myMotorE");
        shooterRight = hardwareMap.get(DcMotor.class, "myMotorE2");

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // right bumper pressed
    public void shoot() {
        shooterLeft.setPower(1);
        shooterRight.setPower(-1);
    }

    // right bumper released
    public void stop() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}
