package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused"})
@Autonomous (name="RobotAutonomous", group="MainGroup")
public class RobotAutonomous extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException{

        MapHardware();

        //wait for pressing play
        waitForStart();

        //if on start autonomous
        if(opModeIsActive())
            AutonomousSequence();

        while (opModeIsActive()) {
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

    }



    //autonomous sequence
    private void AutonomousSequence(){
        //autonomous sequence
        DriveForward(1, 3000);
        DriveLeft(1,4000);
        DriveBackward(1,3000);
        DriveRight(1,4000);
        TurnLeft(1, 5000);
        TurnRight(1, 5000);
    }



    //driving methods

    //drive forward with time
    private void DriveForwardTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(power);
        wheelRB.setPower(power);
        wheelLB.setPower(power);

        //wait
        sleep(time);
    }

    //drive backward with time
    private void DriveBackwardTime(double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(-power);
        wheelRB.setPower(-power);
        wheelLB.setPower(-power);

        //wait
        sleep(time);
    }


    //drive left with time
    private void DriveLeftTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(-power);
        wheelRB.setPower(power);
        wheelLB.setPower(-power);

        //wait
        sleep(time);
    }

    //drive right with time
    private void DriveRightTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(power);
        wheelRB.setPower(-power);
        wheelLB.setPower(power);

        //wait
        sleep(time);
    }


    //turn left with time
    private void TurnLeftTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(-power);
        wheelRF.setPower(power);
        wheelRB.setPower(power);
        wheelLB.setPower(-power);

        //wait
        sleep(time);
    }

    //turn right with time
    private void TurnRightTime (double power, int time){
        //stop if disabled
        if(!opModeIsActive())
            return;

        //set wheels
        wheelLF.setPower(power);
        wheelRF.setPower(-power);
        wheelRB.setPower(-power);
        wheelLB.setPower(power);

        //wait
        sleep(time);
    }


    //stop driving
    private void StopDriving(){
        //set wheels
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }


    //drive forward with distance
    private void DriveForward(double power, int distance){
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
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
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

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
    //drive backward with distance
    private void DriveBackward(double power, int distance){
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
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
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

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }

    //drive forward with distance
    private void DriveLeft(double power, int distance){
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
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
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

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
    //drive forward with distance
    private void DriveRight(double power, int distance){
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
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
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

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }

    //drive forward with distance
    private void TurnLeft(double power, int distance){
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
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
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

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
    //drive forward with distance
    private void TurnRight(double power, int distance){
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
        wheelLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){
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

        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
}