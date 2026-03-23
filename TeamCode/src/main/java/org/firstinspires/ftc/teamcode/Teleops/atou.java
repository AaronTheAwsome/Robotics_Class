package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
@Autonomous(name = "auto")
public class atou extends LinearOpMode{


    public void runOpMode(){
        waitForStart();
        int COUNT = 3;
        DriveSubsystem myDriveTrain = new DriveSubsystem(hardwareMap);
        myDriveTrain.setHeadingToMaintain(myDriveTrain.getCurrentHeadingDeg());
        sleep(1000);
        Shooter SUPA_GUN = new Shooter(hardwareMap);
        myDriveTrain.drive2(-0.5,-1,0);
        sleep(1000);
        myDriveTrain.drive2(0,0,0);
        sleep(500);
        for (int i = 1; i < 10; i++){
            SUPA_GUN.toggleMotor();
        }
        for (int i = 0; i < COUNT; i++) {
            for (int v = 1; v < 10; v++){
                SUPA_GUN.toggleMotor();
            }
            sleep(1000);
            SUPA_GUN.servopos2();
            sleep(500);
            SUPA_GUN.servopos1();
        }
        SUPA_GUN.toggleMotor();
        sleep(1000);
    }
}