package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {
    private final DcMotor launchMotor;
    private final DcMotor elevator1;   // myMotorE
    private final DcMotor elevator2;   // myMotorE2
    private final Servo smaker;
    private final Servo otherSmaker;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        elevator1   = hardwareMap.get(DcMotor.class, "myMotorE");
        elevator2   = hardwareMap.get(DcMotor.class, "myMotorE2");
        smaker      = hardwareMap.get(Servo.class, "Smaker");
        otherSmaker = hardwareMap.get(Servo.class, "otherSmaker");

        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        smaker.setPosition(0.5);
        otherSmaker.setPosition(0.5);
    }

    // flywheel
    public void spinUp(double power) {
        launchMotor.setPower(power);
    }

    public void spinOff() {
        launchMotor.setPower(0);
    }

    // feeder servos
    public void feed() {
        smaker.setPosition(0);
        otherSmaker.setPosition(1);
    }

    public void neutral() {
        smaker.setPosition(0.5);
        otherSmaker.setPosition(0.5);
    }

    // elevator
    public void elevatorOn() {
        elevator1.setPower(1);
        elevator2.setPower(-1);
    }

    public void elevatorOff() {
        elevator1.setPower(0);
        elevator2.setPower(0);
    }

    public int getFlywheelTicks() {
        return launchMotor.getCurrentPosition();
    }
}
