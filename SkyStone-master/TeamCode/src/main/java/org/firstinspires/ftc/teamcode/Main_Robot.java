package org.firstinspires.ftc.teamcode;


import android.content.res.Resources;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Main_Robot", group = "TeleOp")
public class Main_Robot extends LinearOpMode {

    //sound
    private MediaPlayer lightsaberSound;
    private MediaPlayer bruhSound;

    //wheels
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    //intake
    private DcMotor intakeWheelLeft;
    private DcMotor intakeWheelRight;

    //build plate
    private Servo buildPlateServoLeft;
    private Servo buildPlateServoRight;

    //arm
    private Servo armFoldOutServo;

    //capstone
    private Servo dropCapStoneServo;

    @Override
    public void runOpMode (){

        telemetry.addData("State", "initialized");
        telemetry.update();

        Setup();

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("State", "Running");
            telemetry.update();

            //play a sound
            PlaySound();

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


    //map hardware
    private void Setup(){
        //sound
        lightsaberSound = MediaPlayer.create(hardwareMap.appContext, R.raw.ss_light_saber);
        bruhSound = MediaPlayer.create(hardwareMap.appContext, R.raw.bruhsoundeffect);

        //assign drive wheels
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //reverse drive wheels
        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        //assign intake wheels
        intakeWheelLeft = hardwareMap.get(DcMotor.class, "IntakeWheelLeft");
        intakeWheelRight = hardwareMap.get(DcMotor.class, "IntakeWheelRight");

        //reverse intake wheels
        intakeWheelRight.setDirection(DcMotor.Direction.REVERSE);

        //assign servos
        buildPlateServoLeft = hardwareMap.get(Servo.class, "BuildPlateServoLeft");
        buildPlateServoRight = hardwareMap.get(Servo.class, "BuildPlateServoRight");
        dropCapStoneServo = hardwareMap.get(Servo.class, "DropCapStoneServo");
        armFoldOutServo = hardwareMap.get(Servo.class, "armFoldOutServo");

        //set servo range
        buildPlateServoLeft.scaleRange(.5, 1);
        buildPlateServoRight.scaleRange(0, .5);
        dropCapStoneServo.scaleRange(.2, .7);
        armFoldOutServo.scaleRange(0, 1);

        //set servo to default position
        buildPlateServoLeft.setPosition(0);
        buildPlateServoRight.setPosition(1);
        dropCapStoneServo.setPosition(1);
        armFoldOutServo.setPosition(0);
    }


    private void PlaySound(){
        if(gamepad1.x ||gamepad2.x){
            bruhSound.start();
        }
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

        wheelLF.setPower(Math.pow(inputLF, 3));
        wheelRF.setPower(Math.pow(inputRF, 3));
        wheelRB.setPower(Math.pow(inputRB, 3));
        wheelLB.setPower(Math.pow(inputLB, 3));
    }


    //intake
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


    //move build plate
    private void MoveBuildPlate(){
        if(gamepad2.right_trigger > .1){
            buildPlateServoLeft.setPosition(1);
            buildPlateServoRight.setPosition(0);
        }else if (gamepad2.left_trigger > .1){
            buildPlateServoLeft.setPosition(0);
            buildPlateServoRight.setPosition(1);
        }
    }


    //drop capstone
    private void CapStoneDrop(){
        if(gamepad2.b){
            dropCapStoneServo.setPosition(1);
        }else if (gamepad2.a){
            dropCapStoneServo.setPosition(0);
        }
    }

    //arm
    private void ArmFoldOut(){
        if(gamepad2.left_bumper){
            armFoldOutServo.setPosition(0);
            lightsaberSound.start();
        }else if (gamepad2.right_bumper){
            armFoldOutServo.setPosition(1);
            lightsaberSound.start();
        }
    }
}
