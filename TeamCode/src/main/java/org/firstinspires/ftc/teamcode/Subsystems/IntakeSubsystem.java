package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private final DcMotor pickUp;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        pickUp = hardwareMap.get(DcMotor.class, "pickUp");
        pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeIn() {
        pickUp.setPower(-1);
    }

    public void stop() {
        pickUp.setPower(0);
    }

    public int getTicks() {
        return pickUp.getCurrentPosition();
    }
}
