package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Shooter {

    public static double TARGET_RPM = 500; // change this live from the dashboard
    double gearRatio = 15.0/16.0;
    public static double MOTOR_SPEED = 1;
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    public static double forward = 0.5;
    public static double back = 0.5;

    Servo myServo;
    Servo myServo2;

    boolean running = false;

    public Shooter(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        myServo =  hardwareMap.get(Servo.class, "myServo");
        myServo2 =  hardwareMap.get(Servo.class, "myServo2");

    }

    public void toggleMotor() {
        if (running) {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            //shooter1.setPower(MOTOR_SPEED);
            //shooter2.setPower(MOTOR_SPEED);
            running = false;
        } else {
            double targetDegPerSec = TARGET_RPM * 6.0 * gearRatio; // reads latest value from dashboard
            shooter1.setVelocity(targetDegPerSec, AngleUnit.DEGREES);
            shooter2.setVelocity(targetDegPerSec, AngleUnit.DEGREES);
            //shooter1.setPower(MOTOR_SPEED);
            //shooter2.setPower(MOTOR_SPEED);
            running = true;
        }
    }
    public void servopos2() {
        myServo.setPosition(0.5 + back);
        myServo2.setPosition(0.5 - forward);
    }
    public void servopos1() {
        myServo.setPosition(0.5);
        myServo2.setPosition(0.5);
    }

}