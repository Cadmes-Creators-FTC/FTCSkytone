package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotsConfigs.Robot;
import org.firstinspires.ftc.teamcode.misc.MathFunctions;


@SuppressWarnings({"FieldCanBeLocal"})
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


            //fol arm out
            if(gamepad2.right_bumper)
                robot.ZoneReachArmOut();

            if(gamepad2.left_bumper)
                robot.ZoneReachArmIn();


        }

        telemetry.addData("State", "Disabled");
        telemetry.update();
    }

    private void DriveWithController(){

        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        joyY *= -1;

        //robot drive control
        if (gamepad1.x )
            robot.drive_Controler = true;
        else if (gamepad1.a )
            robot.drive_Controler = false;



        double robotXMovement = joyX;
        double robotYMovement = joyY;

        if(robot.drive_Controler && (joyX != 0 || joyY != 0)){
            robot.UpdateGlobalAngle();

            double joyAngle = Math.atan2(joyY, joyX) * -1 + Math.PI / 2;
            joyAngle = MathFunctions.clambAngleRadians(joyAngle);

            telemetry.addData("joyAngle", Math.toDegrees(joyAngle));

            double robotAngle = Math.toRadians(robot.globalAngle);
            double robotMoveAngle = joyAngle - robotAngle;
            robotMoveAngle = MathFunctions.clambAngleRadians(robotMoveAngle);
            telemetry.addData("robotAngle", robot.globalAngle);
            telemetry.addData("robotMoveAngle", Math.toDegrees(robotMoveAngle));

            robotXMovement = Math.sin(robotMoveAngle);
            robotYMovement = Math.cos(robotMoveAngle);
            telemetry.addData("robotXMovement", Math.toDegrees(robotXMovement));
            telemetry.addData("robotYMovement", Math.toDegrees(robotYMovement));
            telemetry.update();
        }

        double inputLF = 0;
        double inputRF = 0;
        double inputLB = 0;
        double inputRB = 0;

        inputLF += robotXMovement;
        inputRF -= robotXMovement;
        inputRB += robotXMovement;
        inputLB -= robotXMovement;

        inputLF += robotYMovement;
        inputRF += robotYMovement;
        inputRB += robotYMovement;
        inputLB += robotYMovement;

        inputLF += joyR;
        inputRF -= joyR;
        inputRB -= joyR;
        inputLB += joyR;

        inputLF = Math.pow(inputLF, 3);
        inputRF = Math.pow(inputRF, 3);
        inputRB = Math.pow(inputRB, 3);
        inputLB = Math.pow(inputLB, 3);

        robot.wheelLF.setPower(inputLF);
        robot.wheelRF.setPower(inputRF);
        robot.wheelRB.setPower(inputRB);
        robot.wheelLB.setPower(inputLB);
    }
}
