package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@SuppressWarnings("RedundantThrows")
@TeleOp(name = "Main_Robot", group = "TeleOp")
public class Main_Robot extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode () throws InterruptedException{

        telemetry.addData("State", "initializing");
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry);

        //wait for gyro calibration
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("State", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("State", "Running");
            telemetry.update();

            //drive and turn
            DriveWithController();

            //turn intake wheels
            IntakeWheels();

            //move build plate
            MoveBuildPlate();

            //drop capstone
            CapStoneDrop();

            //fol arm out
            ArmFoldOut();
        }

        telemetry.addData("State", "Disabled");
        telemetry.update();
    }


    //drive
    private void DriveWithController(){
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        double inputLF = 0;
        double inputRF = 0;
        double inputLB = 0;
        double inputRB = 0;

        inputLF += joyX;
        inputRF -= joyX;
        inputRB += joyX;
        inputLB -= joyX;

        inputLF -= joyY;
        inputRF -= joyY;
        inputRB -= joyY;
        inputLB -= joyY;

        inputLF += joyR;
        inputRF -= joyR;
        inputRB -= joyR;
        inputLB += joyR;

        robot.wheelLF.setPower(inputLF * inputLF * inputLF);
        robot.wheelRF.setPower(inputRF * inputRF * inputRF);
        robot.wheelRB.setPower(inputRB * inputRB * inputRB);
        robot.wheelLB.setPower(inputLB * inputLB * inputLB);
    }


    //intake
    private void IntakeWheels(){
        if(gamepad2.dpad_up){
            robot.intakeWheelLeft.setPower(1);
            robot.intakeWheelRight.setPower(1);
        }else if (gamepad2.dpad_down){
            robot.intakeWheelLeft.setPower(-1);
            robot.intakeWheelRight.setPower(-1);
        }else if(gamepad2.dpad_left || gamepad2.dpad_right){
            robot.intakeWheelLeft.setPower(0);
            robot.intakeWheelRight.setPower(0);
        }
    }


    //move build plate
    private void MoveBuildPlate(){
        if(gamepad2.right_trigger > .1){
            robot.buildPlateServoLeft.setPosition(1);
            robot.buildPlateServoRight.setPosition(0);
        }else if (gamepad2.left_trigger > .1){
            robot.buildPlateServoLeft.setPosition(0);
            robot.buildPlateServoRight.setPosition(1);
        }
    }


    //drop capstone
    private void CapStoneDrop(){
        if(gamepad2.b){
            robot.dropCapStoneServo.setPosition(1);
        }else if (gamepad2.a){
            robot.dropCapStoneServo.setPosition(0);
        }
    }

    //arm
    private void ArmFoldOut(){
        if(gamepad2.left_bumper){
            robot.armFoldOutServo.setPosition(0);
        }else if (gamepad2.right_bumper){
            robot.armFoldOutServo.setPosition(1);
        }
    }
}
