package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@Autonomous (name="SlipTest2", group="Autonomous")
public class SlipTest2 extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        //initialize robot hardware
        robot = new Robot(hardwareMap, telemetry);

        //wait for start button to be pressed
        waitForStart();

        //start autonomous
        AutonomousSequence();
    }

    //autonomous sequence
    private void AutonomousSequence(){
        robot.DriveForward(60, 1);
        robot.DriveRight(60, 1);
        robot.DriveBackward(60, 1);
        robot.DriveLeft(60, 1);
    }
}