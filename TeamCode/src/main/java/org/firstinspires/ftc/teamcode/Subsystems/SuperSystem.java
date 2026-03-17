package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SuperSystem {
    //declare objects
    Telemetry dashboardTelemetry;
    ElapsedTime clawTimer = new ElapsedTime();
    public static int clawPause = 150;
    ElapsedTime armTimer = new ElapsedTime();
    boolean armPauseTriggered = false;
    ElapsedTime transferTimer = new ElapsedTime();
    boolean transferTriggered = false;
    public static int transferPause = 800;
    public static int pickUpPause = 350;
    boolean isPickupPause = false;
    ElapsedTime pickUpPauseTimer = new ElapsedTime();
    ElapsedTime scanPause = new ElapsedTime();
    boolean isScanning = false;
    public static double scanPowerFast = 1; // 2/17 changed from .6
    public static double scanPowerSlow = .4; // 2/17 changed from .3
    public static double upperLimit = -.15; //-0.05 for clip
    public static double lowerLimit = -0.22; // -0.15 for clip
    public static double autoUpperLimit = -.13; //-0.05 for clip
    public static double autoLowerLimit = -0.25; // -0.15 for clip

    public static int xScanDirection = 0;
    public boolean xReady = false;
    public boolean yReady = false;
    public static double xLeftLimit = -0.15;
    public static double xRightLimit = 0.05;
    public static int scanToggle;
    public static boolean isLowScanning = false;
    public boolean isLowPickupPause = false;
    public boolean prepTransfer = false;
    public boolean lowTransfer = false;

    //set up vision
    RevBlinkinLedDriver blinkin;
    Servo rgbLED;
    public static MyLimeLight myLimeLight;
    DcMotorSimple headlights;
    boolean isBlue = false;
    int toggleState = 0; //0 is yellow, 1 is color, 2 is lift
    boolean isAutoScan = false;
    boolean isAutoTransferDone = false;
    boolean isAutoClip = false;
    boolean isAutoTransferPause = false;
    boolean isAuto;
    boolean isAutoBasket = false;

    public SuperSystem(@NonNull HardwareMap hardwareMap, @NonNull Telemetry db, int program){
        dashboardTelemetry = db;
        headlights = hardwareMap.get(DcMotorSimple.class, "led");
        myLimeLight = new MyLimeLight(hardwareMap);
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinken");
        rgbLED = hardwareMap.get(Servo.class,"rgbLight");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        rgbLED.setPosition(.333);
//        isAuto = b;
        if (program != 0) {
            isAuto = true;
            if (program == 1) {
                upperLimit = autoUpperLimit; //-0.05 for clip
                lowerLimit = autoLowerLimit; // -0.15 for clip
                isAutoClip = true;
            } else {
                upperLimit = -.15;
                lowerLimit = -.22;
                isAutoBasket = true;
            }
        }
    }

    public void start(){
        headlights.setPower(0);
    }
}