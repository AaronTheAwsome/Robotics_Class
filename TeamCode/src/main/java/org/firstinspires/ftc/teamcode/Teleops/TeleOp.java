package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.SuperSystem;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "POOP")
//@Disabled
public abstract class TeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    DriveSubsystem myDriveTrain;
    private Shooter myShooter;
    GamepadEx g1;
    ElapsedTime feederTimer = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    protected AllianceColor allianceColor;
    double headingOffset;
    double left_y, right_y, left_x, right_x, left_t, right_t;
    int directionToGo = 0;
    SuperSystem superSystem;
    Odometry odometry;
    public static double xSpeedFactor = 0.2;
    int prescan = 0;


    @Override
    public void init() {
        this.myDriveTrain = new DriveSubsystem(hardwareMap);
        this.myShooter = new Shooter(hardwareMap);
        g1 = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        superSystem = new SuperSystem(hardwareMap,dashboardTelemetry, 0);
        g1 = new GamepadEx(gamepad1);
        allianceColor = getAllianceColor();
        odometry = new Odometry(hardwareMap);
        odometry.odoUp();
        superSystem.setIsAutoScan(false);
        if (allianceColor.equals(AllianceColor.BLUE)) {
            superSystem.setAlliance(true);
        } else {
            superSystem.setAlliance(false);
        }

            dashboardTelemetry.addData("Status", "Initialized");
            dashboardTelemetry.update();
        }

        /*
         * Code to run ONCE when the driver hits START
         */
        @Override
        public void start() {
            superSystem.start();
        }

        public void loop() {
            //update gamepad values
            g1.readButtons();
            left_y = zeroAnalogInput(g1.getLeftY());
            right_y = zeroAnalogInput(g1.getRightY());
            left_x = zeroAnalogInput(g1.getLeftX());
            right_x = zeroAnalogInput(g1.getRightX());
            left_t = -zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            right_t = zeroAnalogInput(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));


            dashboardTelemetry.addData("autoTransfer Done",superSystem.getIsAutoTransferDone());
            dashboardTelemetry.addData("direction facing",drive.getHeadingToMaintain());

//        driver 1
            if (!superSystem.getIsScanning() && !superSystem.getIsLowScanning()){
                if (allianceColor.equals(AllianceColor.BLUE)) {
                    drive.drive2(digitalTransmission(-left_x), digitalTransmission(-left_y), right_x);
                } else {
                    drive.drive2(digitalTransmission(-left_x), digitalTransmission(-left_y), right_x);
                }
            }else if(superSystem.getIsLowScanning()){
                drive.drive2(digitalTransmission((double) -superSystem.getXScanDirection() * xSpeedFactor),digitalTransmission(0),0);
            }else if(superSystem.getIsScanning()){
                double directionToGO = superSystem.getXScanDirection();
                dashboardTelemetry.addData("xOffset", directionToGo);

                if(drive.getHeadingToMaintain() == 0){
                    drive.drive2(digitalTransmission((double) superSystem.getXScanDirection() *xSpeedFactor),digitalTransmission(0),0);
                }else if(drive.getHeadingToMaintain() == 90){
                    drive.drive2(digitalTransmission(0),digitalTransmission((double) superSystem.getXScanDirection() *xSpeedFactor),0);
                }else if(drive.getHeadingToMaintain() == 180){
                    drive.drive2(digitalTransmission((double) -superSystem.getXScanDirection() * xSpeedFactor),digitalTransmission(0),0);
                }else if(drive.getHeadingToMaintain() == -90){
                    drive.drive2(digitalTransmission(0),digitalTransmission((double) -superSystem.getXScanDirection() * xSpeedFactor),0);
                }
            }

            if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                superSystem.toggle();
            }


            //this will run all of our supersystem commands
            if (g1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (this.g1.isDown(GamepadKeys.Button.A)) { //really X
                    superSystem.reset();
                    if(drive.getHeadingToMaintain() == 180){
                        prescan = 1;
                    }else{
                        prescan = 0;
                    }
                } else if (this.g1.wasJustPressed(GamepadKeys.Button.B)) { //really O
                    if(prescan == 1){
                        superSystem.toPreScan();
                        prescan = 2;
                    } else if (prescan == 2){
                        if (superSystem.getToggleState() == 0) {
                            superSystem.lowScan(1);
                        } else if (superSystem.getToggleState() == 1) {
                            if (allianceColor.equals(AllianceColor.BLUE)) {
                                superSystem.lowScan(2);
                            } else if (allianceColor.equals(AllianceColor.RED)) {
                                superSystem.lowScan(0);
                            }
                        }
                        prescan = 3;
                    } else if(prescan == 3){
                        superSystem.lowTransfer();
                        prescan = 1;
                    }else{
                        if (superSystem.getToggleState() == 0) {
                            superSystem.scan(1);
                        } else if (superSystem.getToggleState() == 1) {
                            if (allianceColor.equals(AllianceColor.BLUE)) {
                                superSystem.scan(2);
                            } else if (allianceColor.equals(AllianceColor.RED)) {
                                superSystem.scan(0);
                            }
                        }
                    }
                } else if (this.g1.isDown(GamepadKeys.Button.Y)) { //really ^
                    superSystem.prepToDropOff();
                    if(superSystem.getToggleState() == 0){
                        drive.setHeadingToMaintain(135);
                    }
//                else if(superSystem.getToggleState() == 1){
//                    drive.setHeadingToMaintain(0);
//                }

                } else if (this.g1.isDown(GamepadKeys.Button.X)) { ////really []
                    superSystem.dropOff();
                }
            /*
            else if (g1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                superSystem.liftPhase1();
            } else if (g1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                superSystem.liftPhase2();
            } else if (g1.isDown(GamepadKeys.Button.DPAD_UP)) {
                superSystem.liftPhase3();
            } else if (g1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                superSystem.liftPhase4();
            }
             */


            }else{
                if (this.g1.wasJustPressed(GamepadKeys.Button.A)) { //really X
                    drive.setHeadingToMaintain(0); // 180 degrees???
                    prescan = 0;
                } else if (this.g1.wasJustPressed(GamepadKeys.Button.B)) { //really O
                    drive.setHeadingToMaintain(90);
                    prescan = 0;
                } else if (this.g1.wasJustPressed(GamepadKeys.Button.Y)) { //really ^
                    drive.setHeadingToMaintain(180);
                    prescan = 1;
                } else if (this.g1.wasJustPressed(GamepadKeys.Button.X)) { ////really []
                    drive.setHeadingToMaintain(-90);
                    prescan = 0;
                }
            }
            superSystem.update();
            dashboardTelemetry.update();
        }

        /**
         * removes the analog drift
         */
        private double zeroAnalogInput(double input){
            if (Math.abs(input) < 0.05){
                input = 0;
            }
            return input;
        }

        private double digitalTransmission(double input) {
            if (input < -0.8){
                return 3*input+2;
            } else if (input > 0.8){
                return 3*input-2;
            }
            return .5*input;
        }

        protected abstract AllianceColor getAllianceColor();
    }
