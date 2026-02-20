package org.firstinspires.ftc.teamcode.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDriveTrain{
    private IMU imu;
    private DcMotor dcMotor;
    private DcMotor dcMotor2;
    private DcMotor dcMotor3;
    private DcMotor dcMotor4;
    double holdYaw = 0;

    public MecanumDriveTrain(HardwareMap hardwareMap){

        dcMotor = hardwareMap.get(DcMotor.class, "dcMotor");
        dcMotor2 = hardwareMap.get(DcMotor.class, "dcMotor2");
        dcMotor3 = hardwareMap.get(DcMotor.class, "dcMotor3");
        dcMotor4 = hardwareMap.get(DcMotor.class, "dcMotor4");
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot.LogoFacingDirection logodirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logodirection,usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientation));

        dcMotor.setDirection(DcMotor.Direction.FORWARD);
        dcMotor2.setDirection(DcMotor.Direction.REVERSE);
        dcMotor3.setDirection(DcMotor.Direction.FORWARD);
        dcMotor4.setDirection(DcMotor.Direction.REVERSE);

        dcMotor.setZeroPowerBehavior(BRAKE);
        dcMotor2.setZeroPowerBehavior(BRAKE);
        dcMotor3.setZeroPowerBehavior(BRAKE);
        dcMotor4.setZeroPowerBehavior(BRAKE);

    }



    // Rotate the movement direction counter to the bot's rotation



    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public void mecanumDrive(double drive, double turn){

        double leftFrontPower = Range.clip(drive + turn, -1.0, 1.0);
        double leftBackPower = Range.clip(drive - turn, -1.0, 1.0);
        double rightFrontPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightBackPower = Range.clip(drive - turn, -1.0, 1.0);

        dcMotor.setPower(leftFrontPower);
        dcMotor2.setPower(rightFrontPower);
        dcMotor3.setPower(leftBackPower);
        dcMotor4.setPower(rightBackPower);




    }

}