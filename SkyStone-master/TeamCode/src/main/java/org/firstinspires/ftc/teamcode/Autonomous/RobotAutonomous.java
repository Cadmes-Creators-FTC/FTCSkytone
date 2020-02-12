package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.Robot;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused", "WeakerAccess"})
@Disabled
public class RobotAutonomous extends LinearOpMode {

    private Robot robot;

    public RobotAutonomous(Robot inputRobot){
        robot = inputRobot;
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }


    //Drive Forward with distance
    public void DriveForward(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, false);

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize wheel positions
        robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
        robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
        robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
        robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && robot.wheelLFPos < distance && robot.wheelRFPos < distance && robot.wheelRBPos < distance && robot.wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            robot.wheelLF.setPower(power - wheelCorrection);
            robot.wheelRF.setPower(power + wheelCorrection);
            robot.wheelRB.setPower(power + wheelCorrection);
            robot.wheelLB.setPower(power - wheelCorrection);

            //set wheelPositions
            robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
            robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
            robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
            robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());
        }

        //set power to 0
        robot.wheelLF.setPower(0);
        robot.wheelRF.setPower(0);
        robot.wheelRB.setPower(0);
        robot.wheelLB.setPower(0);
    }
    //Drive Backward with distance
    public void DriveBackward(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, false);

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize wheel positions
        robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
        robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
        robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
        robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && robot.wheelLFPos < distance && robot.wheelRFPos < distance && robot.wheelRBPos < distance && robot.wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            robot.wheelLF.setPower(-power - wheelCorrection);
            robot.wheelRF.setPower(-power + wheelCorrection);
            robot.wheelRB.setPower(-power + wheelCorrection);
            robot.wheelLB.setPower(-power - wheelCorrection);

            //set wheelPositions
            robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
            robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
            robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
            robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());
        }

        //set power to 0
        robot.wheelLF.setPower(0);
        robot.wheelRF.setPower(0);
        robot.wheelRB.setPower(0);
        robot.wheelLB.setPower(0);
    }

    //Drive Left with distance
    public void DriveLeft(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, true);

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize wheel positions
        robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
        robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
        robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
        robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && robot.wheelLFPos < distance && robot.wheelRFPos < distance && robot.wheelRBPos < distance && robot.wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            robot.wheelLF.setPower(-power - wheelCorrection);
            robot.wheelRF.setPower(power + wheelCorrection);
            robot.wheelRB.setPower(-power + wheelCorrection);
            robot.wheelLB.setPower(power - wheelCorrection);

            //set wheelPositions
            robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
            robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
            robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
            robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());
        }

        //set power to 0
        robot.wheelLF.setPower(0);
        robot.wheelRF.setPower(0);
        robot.wheelRB.setPower(0);
        robot.wheelLB.setPower(0);
    }
    //Drive Right with distance
    public void DriveRight(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, true);

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize wheel positions
        robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
        robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
        robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
        robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());

        //set to run to position
        robot.wheelLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.wheelLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && robot.wheelLFPos < distance && robot.wheelRFPos < distance && robot.wheelRBPos < distance && robot.wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            robot.wheelLF.setPower(power - wheelCorrection);
            robot.wheelRF.setPower(-power + wheelCorrection);
            robot.wheelRB.setPower(power + wheelCorrection);
            robot.wheelLB.setPower(-power - wheelCorrection);

            //set wheelPositions
            robot.wheelLFPos = Math.abs(robot.wheelLF.getCurrentPosition());
            robot.wheelRFPos = Math.abs(robot.wheelRF.getCurrentPosition());
            robot.wheelRBPos = Math.abs(robot.wheelRB.getCurrentPosition());
            robot.wheelLBPos = Math.abs(robot.wheelLB.getCurrentPosition());
        }

        //set power to 0
        robot.wheelLF.setPower(0);
        robot.wheelRF.setPower(0);
        robot.wheelRB.setPower(0);
        robot.wheelLB.setPower(0);
    }

    //turning
    public void Turn(int turnAmount, double power){
        double flexibility = 5;

        //set targetAngle
        robot.targetAngle -= turnAmount;

        robot.targetAngle = MathFunctions.clambAngle(robot.targetAngle);


        while (robot.globalAngle < robot.targetAngle - flexibility && opModeIsActive()){

            //set power
            robot.wheelLF.setPower(power * -1);
            robot.wheelRF.setPower(power * 1);
            robot. wheelRB.setPower(power * 1);
            robot.wheelLB.setPower(power * -1);

            //update globalAngle
            robot.UpdateGlobalAngle();

            idle();
        }

        while (robot.globalAngle > robot.targetAngle + flexibility && opModeIsActive()){

            //set power
            robot.wheelLF.setPower(power * 1);
            robot.wheelRF.setPower(power * -1);
            robot.wheelRB.setPower(power * -1);
            robot.wheelLB.setPower(power * 1);

            //update globalAngle
            robot.UpdateGlobalAngle();

            idle();
        }

        //set power to 0
        robot.wheelLF.setPower(0);
        robot.wheelRF.setPower(0);
        robot.wheelRB.setPower(0);
        robot.wheelLB.setPower(0);
    }


    //imu gyro sensor
    private double GetWheelCorrection(){
        double gain = .05;

        robot.UpdateGlobalAngle();

        double correction = 0;

        if (robot.globalAngle != robot.targetAngle)
            correction = robot.targetAngle - robot.globalAngle;

        correction = correction * gain;

        return correction;
    }


    //move build plate servos
    public void MoveBuildPlate(boolean Down){
        if(Down){
            robot.buildPlateServoLeft.setPosition(1);
            robot.buildPlateServoRight.setPosition(0);
        }else{
            robot.buildPlateServoLeft.setPosition(0);
            robot.buildPlateServoRight.setPosition(1);
        }
        sleep(500);
    }
}