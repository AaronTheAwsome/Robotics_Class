package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    private final DcMotor leftMotor;   // myMotor3
    private final DcMotor rightMotor;  // myMotor2
    private final IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        rightMotor = hardwareMap.get(DcMotor.class, "myMotor2");
        leftMotor  = hardwareMap.get(DcMotor.class, "myMotor3");

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void drive(double drive, double turn) {
        double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        double yaw = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

        if (Math.abs(turn) < 0.07) {
            if (yaw < -2) {
                rightMotor.setPower(-yaw / 50.0);
                leftMotor.setPower(yaw / 50.0);
            } else if (yaw > 2) {
                rightMotor.setPower(-yaw / 50.0);
                leftMotor.setPower(yaw / 50.0);
            }
        } else {
            imu.resetYaw();
        }
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public int getLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightTicks() {
        return rightMotor.getCurrentPosition();
    }
}

public class DriveSubsystem {
}
