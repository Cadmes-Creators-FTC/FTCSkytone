package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused", "FieldCanBeLocal"})
@Autonomous (name="RedLine", group="Autonomous")
public class RedLine extends LinearOpMode {

    private Robot robot;


    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.addData("State", "initializing");
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry);

        robot.WaitForGyroCalibration();


        telemetry.addData("State", "initialized");
        telemetry.update();

        //wait for pressing play
        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        //if on start autonomous
        AutonomousSequence();

        telemetry.addData("State", "Done");
        telemetry.update();
    }

    //autonomous sequence
    private void AutonomousSequence(){
        robot.DriveForward(10, 0.5);
        robot.Turn(-90, 0.5);
        robot.DriveForward(70, 0.5);
        robot.DriveLeft(35,0.3);
    }
}