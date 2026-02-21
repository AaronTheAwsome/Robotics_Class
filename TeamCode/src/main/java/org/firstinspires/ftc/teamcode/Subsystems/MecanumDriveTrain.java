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
    public void mecanumDrive(double yy,double xx, double rxrx){


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(yy) + Math.abs(xx) + Math.abs(rxrx), 1);
        double frontLeftPower = (yy + xx + rxrx) / denominator;
        double backLeftPower = (yy - xx + rxrx) / denominator;
        double frontRightPower = (yy - xx - rxrx) / denominator;
        double backRightPower = (yy + xx - rxrx) / denominator;

        dcMotor.setPower(frontLeftPower);
        dcMotor2.setPower(backLeftPower);
        dcMotor3.setPower(frontRightPower);
        dcMotor4.setPower(backRightPower);


    }

}