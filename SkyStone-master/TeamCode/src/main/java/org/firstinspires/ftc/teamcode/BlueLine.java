package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused"})
@Autonomous (name="BlueLine", group="MainGroup")
public class BlueLine extends LinearOpMode {

    //variables

    //drive
    //motors
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;
    //positions
    int wheelLFPos = 0;
    int wheelRFPos = 0;
    int wheelRBPos = 0;
    int wheelLBPos = 0;


    //build plate
    //move build plate
    private Servo buildPlateServoLeft;
    private Servo buildPlateServoRight;


    @Override
    public void runOpMode() throws InterruptedException{

        MapHardware();

        //wait for pressing play
        waitForStart();

        //if on start autonomous
        AutonomousSequence();

        while (opModeIsActive()){
            telemetry.addData("LF", wheelLF.getCurrentPosition());
            telemetry.addData("RF", wheelRF.getCurrentPosition());
            telemetry.addData("RB", wheelRB.getCurrentPosition());
            telemetry.addData("LB", wheelLB.getCurrentPosition());
            telemetry.update();
        }
    }

    //map hardware
    private void MapHardware(){
        //map wheels
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //reverse wheels
        wheelLF.setDirection(DcMotor.Direction.REVERSE);
        wheelLB.setDirection(DcMotor.Direction.REVERSE);

        //set target position before run to position
        wheelLF.setTargetPosition(0);
        wheelRF.setTargetPosition(0);
        wheelRB.setTargetPosition(0);
        wheelLB.setTargetPosition(0);

        //assign servos
        buildPlateServoLeft = hardwareMap.get(Servo.class, "BuildPlateServoLeft");
        buildPlateServoRight = hardwareMap.get(Servo.class, "BuildPlateServoRight");

        //set servo range
        buildPlateServoLeft.scaleRange(.5, 1);
        buildPlateServoRight.scaleRange(0, .5);

        //set servo to default position
        buildPlateServoLeft.setPosition(0);
        buildPlateServoRight.setPosition(1);
    }



    //autonomous sequence
    private void AutonomousSequence(){
        DriveForward(CMToTicks(10), 0.7);
        DriveRight(CMToTicks(65), 0.7);
        DriveBackward(CMToTicks(15), 0.4);


    }



    //Drive Forward with distance
    private void DriveForward(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(distance);
        wheelRF.setTargetPosition(distance);
        wheelRB.setTargetPosition(distance);
        wheelLB.setTargetPosition(distance);

        //initialize wheel positions
        wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
        wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
        wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
        wheelLBPos = Math.abs(wheelLB.getCurrentPosition());

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
            //set wheel powers
            wheelLF.setPower(power);
            wheelRF.setPower(power);
            wheelRB.setPower(power);
            wheelLB.setPower(power);

            //set wheelPositions
            wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
            wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
            wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
            wheelLBPos = Math.abs(wheelLB.getCurrentPosition());
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
    //Drive Backward with distance
    private void DriveBackward(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(-distance);
        wheelRF.setTargetPosition(-distance);
        wheelRB.setTargetPosition(-distance);
        wheelLB.setTargetPosition(-distance);

        //initialize wheel positions
        wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
        wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
        wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
        wheelLBPos = Math.abs(wheelLB.getCurrentPosition());

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
            //set wheel powers
            wheelLF.setPower(-power);
            wheelRF.setPower(-power);
            wheelRB.setPower(-power);
            wheelLB.setPower(-power);

            //set wheelPositions
            wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
            wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
            wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
            wheelLBPos = Math.abs(wheelLB.getCurrentPosition());
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }

    //Drive Left with distance
    private void DriveLeft(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(-distance);
        wheelRF.setTargetPosition(distance);
        wheelRB.setTargetPosition(-distance);
        wheelLB.setTargetPosition(distance);

        //initialize wheel positions
        wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
        wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
        wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
        wheelLBPos = Math.abs(wheelLB.getCurrentPosition());

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
            //set wheel powers
            wheelLF.setPower(-power);
            wheelRF.setPower(power);
            wheelRB.setPower(-power);
            wheelLB.setPower(power);

            //set wheelPositions
            wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
            wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
            wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
            wheelLBPos = Math.abs(wheelLB.getCurrentPosition());
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
    //Drive Right with distance
    private void DriveRight(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(distance);
        wheelRF.setTargetPosition(-distance);
        wheelRB.setTargetPosition(distance);
        wheelLB.setTargetPosition(-distance);

        //initialize wheel positions
        wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
        wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
        wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
        wheelLBPos = Math.abs(wheelLB.getCurrentPosition());

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance) {
            //set wheel powers
            wheelLF.setPower(power);
            wheelRF.setPower(-power);
            wheelRB.setPower(power);
            wheelLB.setPower(-power);

            //set wheelPositions
            wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
            wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
            wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
            wheelLBPos = Math.abs(wheelLB.getCurrentPosition());
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }

    //Turn Left with distance
    private void TurnLeft(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(-distance);
        wheelRF.setTargetPosition(distance);
        wheelRB.setTargetPosition(distance);
        wheelLB.setTargetPosition(-distance);

        //initialize wheel positions
        wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
        wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
        wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
        wheelLBPos = Math.abs(wheelLB.getCurrentPosition());

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
            //set wheel powers
            wheelLF.setPower(-power);
            wheelRF.setPower(power);
            wheelRB.setPower(power);
            wheelLB.setPower(-power);

            //set wheelPositions
            wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
            wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
            wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
            wheelLBPos = Math.abs(wheelLB.getCurrentPosition());
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
    //Turn Right with distance
    private void TurnRight(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        wheelLF.setTargetPosition(distance);
        wheelRF.setTargetPosition(-distance);
        wheelRB.setTargetPosition(-distance);
        wheelLB.setTargetPosition(distance);

        //initialize wheel positions
        wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
        wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
        wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
        wheelLBPos = Math.abs(wheelLB.getCurrentPosition());

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
            //set wheel powers
            wheelLF.setPower(power);
            wheelRF.setPower(-power);
            wheelRB.setPower(-power);
            wheelLB.setPower(power);

            //set wheelPositions
            wheelLFPos = Math.abs(wheelLF.getCurrentPosition());
            wheelRFPos = Math.abs(wheelRF.getCurrentPosition());
            wheelRBPos = Math.abs(wheelRB.getCurrentPosition());
            wheelLBPos = Math.abs(wheelLB.getCurrentPosition());
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }

    //convert cm to encoder ticks
    private int CMToTicks(double CM){
        double tickCM = 1120 / 26.928;
        tickCM *= (100f/141f);
        long ticks = Math.round(tickCM * CM);
        return (int) ticks;
    }


    //move build plate servos
    private void MoveBuildPlate(boolean Down){
        if(Down){
            buildPlateServoLeft.setPosition(1);
            buildPlateServoRight.setPosition(0);
        }else{
            buildPlateServoLeft.setPosition(0);
            buildPlateServoRight.setPosition(1);
        }
        sleep(500);
    }
}