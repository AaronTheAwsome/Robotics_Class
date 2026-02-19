package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    private final DcMotor liftMotor;
    private final DcMotor pickUp;
    private final Servo liftServo;
    private final Servo liftServo2;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        liftMotor  = hardwareMap.get(DcMotor.class, "liftMotor");
        pickUp     = hardwareMap.get(DcMotor.class, "pickUp");
        liftServo  = hardwareMap.get(Servo.class, "liftServo");
        liftServo2 = hardwareMap.get(Servo.class, "liftServo2");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pickUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setStartPositions() {
        liftServo.setPosition(0.5);
        liftServo2.setPosition(0.5);
    }

    // gamepad1.x
    public void intakeAndLift() {
        pickUp.setPower(-1);
        liftMotor.setPower(0.2);
    }

    // gamepad1.circle
    public void stopIntakeAndLift() {
        pickUp.setPower(0);
        liftMotor.setPower(0);
    }

    // gamepad1.triangle pressed: open / dump
    public void dumpPosition() {
        liftServo.setPosition(0);
        liftServo2.setPosition(1);
    }

    // triangle not pressed: default
    public void neutralPosition() {
        liftServo.setPosition(0.5);
        liftServo2.setPosition(0.5);
    }

    // left bumper press / release
    public void liftUp() {
        liftMotor.setPower(1);
    }

    public void liftStop() {
        liftMotor.setPower(0);
    }

    public int getLiftTicks() {
        return liftMotor.getCurrentPosition();
    }

    public void stop() {
        pickUp.setPower(0);
        liftMotor.setPower(0);
    }
}
