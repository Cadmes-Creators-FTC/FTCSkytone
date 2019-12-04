package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Main_Robot", group = "MainGroup")
public class Main_Robot extends LinearOpMode {

    //drive
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    //intake
    private DcMotor intakeWheelLeft;
    private DcMotor intakeWheelRight;

    //build plate
    //move build plate
    private Servo buildPlateServoLeft;
    private Servo buildPlateServoRight;

    //capstone
    private Servo dropCapStoneServo;

    @Override
    public void runOpMode (){

        telemetry.addData("running?", "true");
        telemetry.update();

        MapHardware();

        waitForStart();

        while (opModeIsActive()){

            //drive and turn
            DriveWithController();

            //turn intake wheels
            IntakeWheels();

            //move build plate
            MoveBuildPlate();

            //drop capstone
            CapStoneDrop();

            //update telemetry
            telemetry.update();
        }

    }



    //map hardware
    private void MapHardware(){
        //assign drive wheels
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //set drive wheels encoders modes
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse drive wheels
        wheelRF.setDirection(DcMotor.Direction.REVERSE);
        wheelRB.setDirection(DcMotor.Direction.REVERSE);

        //assign intake wheels
        intakeWheelLeft = hardwareMap.get(DcMotor.class, "IntakeWheelLeft");
        intakeWheelRight = hardwareMap.get(DcMotor.class, "IntakeWheelRight");

        //reverse intake wheels
        intakeWheelRight.setDirection(DcMotor.Direction.REVERSE);

        //assign servos
        buildPlateServoLeft = hardwareMap.get(Servo.class, "BuildPlateServoLeft");
        buildPlateServoRight = hardwareMap.get(Servo.class, "BuildPlateServoRight");
        dropCapStoneServo = hardwareMap.get(Servo.class, "DropCapStoneServo");

        //set servo range
        buildPlateServoLeft.scaleRange(.5, 1);
        buildPlateServoRight.scaleRange(0, .5);
        dropCapStoneServo.scaleRange(.2, .7);

        //set servo to default position
        buildPlateServoLeft.setPosition(0);
        buildPlateServoRight.setPosition(1);
        dropCapStoneServo.setPosition(1);
    }



    private void DriveWithController(){
        double joyX = gamepad1.left_stick_x;
        double joyY = gamepad1.left_stick_y;
        double joyR = gamepad1.right_stick_x;

        double inputLF = 0;
        double inputRF = 0;
        double inputLB = 0;
        double inputRB = 0;

        inputLF -= joyX;
        inputRF += joyX;
        inputLB += joyX;
        inputRB -= joyX;

        inputLF += joyY;
        inputRF += joyY;
        inputLB += joyY;
        inputRB += joyY;

        inputLF -= joyR;
        inputRF += joyR;
        inputLB -= joyR;
        inputRB += joyR;

        telemetry.addData("LF", inputLF);
        telemetry.addData("RF", inputRF);
        telemetry.addData("RB", inputRB);
        telemetry.addData("LB", inputLB);

        wheelLF.setPower(inputLF);
        wheelRF.setPower(inputRF);
        wheelLB.setPower(inputLB);
        wheelRB.setPower(inputRB);
    }

    private void IntakeWheels(){
        if(gamepad2.dpad_up){
            intakeWheelLeft.setPower(1);
            intakeWheelRight.setPower(1);
        }else if (gamepad2.dpad_down){
            intakeWheelLeft.setPower(-1);
            intakeWheelRight.setPower(-1);
        }else if(gamepad2.dpad_left || gamepad2.dpad_right){
            intakeWheelLeft.setPower(0);
            intakeWheelRight.setPower(0);
        }
    }

    //move build plate servos
    private void MoveBuildPlate(){
        if(gamepad2.right_trigger > .1){
            buildPlateServoLeft.setPosition(1);
            buildPlateServoRight.setPosition(0);
        }else if (gamepad2.left_trigger > .1){
            buildPlateServoLeft.setPosition(0);
            buildPlateServoRight.setPosition(1);
        }
    }

    private void CapStoneDrop(){
        if(gamepad2.b){
            dropCapStoneServo.setPosition(1);
        }else if (gamepad2.a){
            dropCapStoneServo.setPosition(0);
        }
    }
}
