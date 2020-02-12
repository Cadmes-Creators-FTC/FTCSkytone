package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused", "FieldCanBeLocal"})
@Autonomous (name="RedBuildPlateLine", group="Autonomous")
public class RedBuildPlateLine extends LinearOpMode {

    private Robot robot;
    private RobotAutonomous autonomous;


    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.addData("State", "initializing");
        telemetry.update();

        robot = new Robot();

        //wait for gyro calibration
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        autonomous = new RobotAutonomous(robot);


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
        autonomous.DriveForward(40, 0.4);
        autonomous.DriveRight(40, 0.4);
        autonomous.DriveForward(40, 0.4);
        autonomous.DriveForward(40, 0.2);
        autonomous.MoveBuildPlate(true);
        autonomous.DriveBackward(80, 0.4);
        autonomous.Turn(90, 0.5);
        autonomous.MoveBuildPlate(false);
        autonomous.DriveBackward(40, 0.8);
        autonomous.Turn(-180, 0.5);
        autonomous.DriveForward(30, 0.8);
        autonomous.DriveLeft(20, 0.5);
    }
}