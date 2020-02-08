package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@SuppressWarnings({"RedundantThrows", "SameParameterValue", "unused"})
@Autonomous (name="BlueLine", group="Autonomous")
public class BlueLine extends LinearOpMode {

    //wheels
    //motors
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    //positions
    private int wheelLFPos = 0;
    private int wheelRFPos = 0;
    private int wheelRBPos = 0;
    private int wheelLBPos = 0;


    //servos
    private Servo buildPlateServoLeft;
    private Servo buildPlateServoRight;


    //IMU
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();

    private double correction;
    private double globalAngle;
    private double targetAngle;


    @Override
    public void runOpMode() throws InterruptedException{

        MapHardware();

        //wait for gyro calibration
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

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

        //assign servos
        buildPlateServoLeft = hardwareMap.get(Servo.class, "BuildPlateServoLeft");
        buildPlateServoRight = hardwareMap.get(Servo.class, "BuildPlateServoRight");

        //set servo range
        buildPlateServoLeft.scaleRange(.5, 1);
        buildPlateServoRight.scaleRange(0, .5);

        //set servo to default position
        buildPlateServoLeft.setPosition(0);
        buildPlateServoRight.setPosition(1);

        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }


    //autonomous sequence
    private void AutonomousSequence(){
        DriveForward(CMToTicks(10, false), 0.5);
        Turn(90, 0.5);
        DriveForward(CMToTicks(69, false), 0.5);
        DriveRight(CMToTicks(35, true),0.3);
    }


    //Drive Forward with distance
    private void DriveForward(int distance, double power){
        //set to run to position
        wheelLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            // Use gyro to drive in a straight line.
            correction = GetCorrection();

            wheelLF.setPower(power - correction);
            wheelRF.setPower(power + correction);
            wheelRB.setPower(power + correction);
            wheelLB.setPower(power - correction);

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

            // Use gyro to drive in a straight line.
            correction = GetCorrection();

            wheelLF.setPower(-power - correction);
            wheelRF.setPower(-power + correction);
            wheelRB.setPower(-power + correction);
            wheelLB.setPower(-power - correction);

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

            // Use gyro to drive in a straight line.
            correction = GetCorrection();

            wheelLF.setPower(-power - correction);
            wheelRF.setPower(power + correction);
            wheelRB.setPower(-power + correction);
            wheelLB.setPower(power - correction);

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

            // Use gyro to drive in a straight line.
            correction = GetCorrection();

            wheelLF.setPower(power - correction);
            wheelRF.setPower(-power + correction);
            wheelRB.setPower(power + correction);
            wheelLB.setPower(-power - correction);

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

    //turning
    private void Turn(int turnAmount, double power){
        double flexibility = 5;

        //set targetAngle
        targetAngle -= turnAmount;


        while (globalAngle < targetAngle - flexibility && opModeIsActive()){

            //set power
            wheelLF.setPower(power * -1);
            wheelRF.setPower(power * 1);
            wheelRB.setPower(power * 1);
            wheelLB.setPower(power * -1);

            //update globalAngle
            getAngle();

            idle();
        }

        while (globalAngle > targetAngle + flexibility && opModeIsActive()){

            //set power
            wheelLF.setPower(power * 1);
            wheelRF.setPower(power * -1);
            wheelRB.setPower(power * -1);
            wheelLB.setPower(power * 1);

            //update globalAngle
            getAngle();

            idle();
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }


    //imu gyro sensor
    private double GetCorrection(){
        double angle = getAngle();
        double gain = .05;

        if (angle == targetAngle)
            correction = 0;
        else
            correction = targetAngle - angle;

        correction = correction * gain;

        return correction;
    }

    private double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    //convert cm to encoder ticks
    private int CMToTicks(double CM, boolean side){
        if(side) {
            CM *= Math.sqrt(2);
        }

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