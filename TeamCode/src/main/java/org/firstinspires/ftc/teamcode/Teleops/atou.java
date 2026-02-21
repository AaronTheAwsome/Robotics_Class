package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveTrain;
@Autonomous(name = "atuo")
public class atou extends LinearOpMode{
    public void runOpMode(){
        waitForStart();
        MecanumDriveTrain myDriveTrain = new MecanumDriveTrain(hardwareMap);
        myDriveTrain.mecanumDrive(1,0,0);
        sleep(1000);
    }
}
