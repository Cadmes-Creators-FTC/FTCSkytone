package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;
import org.firstinspires.ftc.teamcode.misc.MathFunctions;


@SuppressWarnings({"FieldCanBeLocal"})
@TeleOp(name = "Main_Robot", group = "TeleOp")
public class MainTeleop extends LinearOpMode {

    private Robot robot;
    private boolean headlessDrive;

    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "initializing");
        telemetry.update();

        //initialize robot hardware
        robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("State", "initialized");
        telemetry.addData("imu calibration state: ", "calibrating");
        telemetry.update();

        //wait for imu to calibrate
        robot.WaitForGyroCalibration();

        telemetry.addData("State", "initialized");
        telemetry.addData("imu calibration state: ", "calibrated");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            //drive and turn
            DriveWithController();


            //turn intake wheels
            if(gamepad2.dpad_up)
                robot.IntakeWheelsOut();

            if (gamepad2.dpad_down)
                robot.IntakeWheelsIn();

            if(gamepad2.dpad_left || gamepad2.dpad_right)
                robot.IntakeWheelsOff();


            //move build plate
            if(gamepad2.right_trigger > .1)
                robot.BuildPlateHooksDown();

            if (gamepad2.left_trigger > .1)
                robot.BuildPlateHooksUp();


            //drop capstone
            if(gamepad2.a)
                robot.DropCapstone();


            //fold arm out
            if(gamepad2.right_bumper)
                robot.ZoneReachArmOut();

            if(gamepad2.left_bumper)
                robot.ZoneReachArmIn();

        }
    }


    private void DriveWithController(){
        //get joystick input
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        //reverse y joystick
        joyY *= -1;


        //headless drive switch
        if (gamepad1.x)
            headlessDrive = true;
        else if (gamepad1.a)
            headlessDrive = false;

        //reset forward
        if(gamepad1.dpad_down)
            robot.ResetGlobalAngle();


        //create movement variables
        double robotXMovement = joyX;
        double robotYMovement = joyY;

        //if headless drive is on
        if(headlessDrive && (joyX != 0 || joyY != 0)){
            //update robotAngle
            robot.UpdateGlobalAngle();

            //get joystick angel in radians
            double joyAngle = Math.atan2(joyY, joyX) * -1 + Math.PI / 2;
            joyAngle = MathFunctions.clambAngleRadians(joyAngle);

            //get the angle the robot should move in
            double robotAngle = Math.toRadians(robot.globalAngle);
            double robotMoveAngle = joyAngle - robotAngle;
            robotMoveAngle = MathFunctions.clambAngleRadians(robotMoveAngle);

            //get the x and y movement values
            robotXMovement = Math.sin(robotMoveAngle);
            robotYMovement = Math.cos(robotMoveAngle);

            //still use the speed
            robotXMovement *= joyX;
            robotYMovement *= joyY;
        }


        //create motor power variables
        double LFPower = 0;
        double RFPower = 0;
        double RBPower = 0;
        double LBPower = 0;

        //add x movement
        LFPower += robotXMovement;
        RFPower -= robotXMovement;
        RBPower += robotXMovement;
        LBPower -= robotXMovement;

        //add y movement
        LFPower += robotYMovement;
        RFPower += robotYMovement;
        RBPower += robotYMovement;
        LBPower += robotYMovement;

        //add rotation
        LFPower += joyR;
        RFPower -= joyR;
        RBPower -= joyR;
        LBPower += joyR;

        //smooth out the acceleration
        LFPower = Math.pow(LFPower, 3);
        RFPower = Math.pow(RFPower, 3);
        RBPower = Math.pow(RBPower, 3);
        LBPower = Math.pow(LBPower, 3);

        //set motor power
        robot.wheelLF.setPower(LFPower);
        robot.wheelRF.setPower(RFPower);
        robot.wheelRB.setPower(RBPower);
        robot.wheelLB.setPower(LBPower);
    }
}
