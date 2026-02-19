package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {

    private final DcMotor leftMotor;   // myMotor3
    private final DcMotor rightMotor;  // myMotor2
    private final IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftMotor  = hardwareMap.get(DcMotor.class, "myMotor3");
        rightMotor = hardwareMap.get(DcMotor.class, "myMotor2");
        imu        = hardwareMap.get(IMU.class, "imu");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void driveTank(double drive, double turn) {
        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void headingHold(double turnDeadband, double kP) {
        double turnInput = 0; // you pass in gamepad turn, see OpMode below

        // If you want the subsystem to fully handle it, give headingHold the turn:
        // and save it in a field instead, but to keep this simple we will call this
        // only when |turn| < deadband in the OpMode.

        double yaw = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

        if (Math.abs(yaw) > 2) {
            double correction = -yaw * kP;
            correction = Range.clip(correction, -0.3, 0.3);
            leftMotor.setPower(-correction);
            rightMotor.setPower(correction);
        }
    }

    public double getYaw()   { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }
    public double getPitch() { return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES); }
    public double getRoll()  { return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES); }

    public int getLeftTicks()  { return leftMotor.getCurrentPosition(); }
    public int getRightTicks() { return rightMotor.getCurrentPosition(); }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
