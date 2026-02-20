package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Shooter {

    public static double TARGET_RPM = 50; // change this live from the dashboard
    double gearRatio = 15.0/(16.0 * 5);
    DcMotorEx shooter1;
    DcMotorEx shooter2;

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
        shooter1.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        myServo =  hardwareMap.get(Servo.class, "myServo");
        myServo2 =  hardwareMap.get(Servo.class, "myServo2");

    }

    public void toggleMotor() {
        if (running) {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            running = false;
        } else {
            double targetDegPerSec = TARGET_RPM * 6.0 * gearRatio; // reads latest value from dashboard
            shooter1.setVelocity(targetDegPerSec, AngleUnit.DEGREES);
            shooter2.setVelocity(targetDegPerSec, AngleUnit.DEGREES);
            running = true;
        }
    }
    public void servopos1() {
        myServo.setPosition(0.0);
        myServo2.setPosition(0.0);
    }
    public void servopos2() {
        myServo.setPosition(0.5);
        myServo2.setPosition(1.5);
    }
}