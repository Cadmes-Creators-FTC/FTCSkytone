package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;


@SuppressWarnings({"RedundantThrows", "FieldCanBeLocal"})
@TeleOp(name = "Main_Robot", group = "TeleOp")
public class MainTeleop extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "initializing");
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry);

        robot.WaitForGyroCalibration();

        telemetry.addData("State", "initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        while (opModeIsActive()){

            //drive and turn
            double joyX = gamepad1.left_stick_x;
            double joyY = gamepad1.left_stick_y;
            double joyR = gamepad1.right_stick_x;

            robot.DriveWithController(joyX, joyY, joyR);


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


            //fol arm out
            if(gamepad2.right_bumper)
                robot.ZoneReachArmOut();

            if(gamepad2.left_bumper)
                robot.ZoneReachArmIn();
        }

        telemetry.addData("State", "Disabled");
        telemetry.update();
    }
}
