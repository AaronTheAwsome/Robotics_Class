package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import javax.annotation.Nonnull;

@Config
public class DriveSubsystem {
    public static double correctionMultiplier    = 1.0;
    public static double ANGULAR_TOLERANCE_DEGREES = 2.0;
    public static double rotMulti                = 1.1;

    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public GoBildaPinpointDriver pinpoint;

    private double headingToMaintain = 0.0;

    public DriveSubsystem(@Nonnull HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
    }


    public double getCurrentHeadingDeg() {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        double headingDeg = Math.toDegrees(pinpoint.getHeading());
        return Math.round(headingDeg * 10.0) / 10.0;
    }

    public void setHeadingToMaintain(double headingDeg) {
        this.headingToMaintain = headingDeg;
    }

    public double getHeadingToMaintain() {
        return headingToMaintain;
    }

    private double getHeadingError(double currentDeg) {
        double error = headingToMaintain - currentDeg;
        error = ((error + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return error;
    }
    private double limiter(double input, double lim) {
        if (input > lim) return lim;
        if (input < -lim) return -lim;
        return input;
    }

    public void drive2(double x, double y, double rx) {
        x = -x;
        y = -y;

        double robotHeadingRad = pinpoint.getHeading();
        double robotHeadingDeg = getCurrentHeadingDeg();

        if (Math.abs(rx) < 1e-3) {
            double errorDeg = getHeadingError(robotHeadingDeg);
            boolean withinTolerance = Math.abs(errorDeg) < ANGULAR_TOLERANCE_DEGREES;

            if (!withinTolerance) {
                double absError = Math.abs(errorDeg);
                double rotSpeed;
                if (absError > 20) {
                    rotSpeed = 1.0;
                } else {
                    rotSpeed = correctionMultiplier * absError * absError / 800.0;
                }
                rx = limiter(Math.signum(errorDeg) * rotSpeed, rotSpeed);
            }
        } else {
            headingToMaintain = robotHeadingDeg;
        }

        double rotX = x * Math.cos(-robotHeadingRad) - y * Math.sin(-robotHeadingRad);
        double rotY = x * Math.sin(-robotHeadingRad) + y * Math.cos(-robotHeadingRad);
        rotX = rotX * rotMulti;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1.0);
        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
    }
    public void slowDown(double SPEED){
        leftFront.setPower(SPEED);
        leftBack.setPower(SPEED);
        rightFront.setPower(SPEED);
        rightBack.setPower(SPEED);
    }
    public void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
