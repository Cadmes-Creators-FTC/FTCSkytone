package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused", "FieldCanBeLocal"})
@Autonomous (name="RedBuildPlateLine", group="Autonomous")
public class RedBuildPlateLine extends LinearOpMode {

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
        robot.DriveForward(40, 0.4);
        robot.DriveRight(40, 0.4);
        robot.DriveForward(40, 0.4);
        robot.DriveForward(40, 0.2);
        robot.BuildPlateHooksDown();
        robot.DriveBackward(80, 0.4);
        robot.Turn(90, 0.5);
        robot.BuildPlateHooksUp();
        robot.DriveBackward(40, 0.8);
        robot.Turn(-180, 0.5);
        robot.DriveForward(30, 0.8);
        robot.DriveLeft(20, 0.5);
    }
}