package org.firstinspires.ftc.teamcode.RobotsConfigs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.misc.MathFunctions;

@SuppressWarnings({"WeakerAccess", "unused", "StatementWithEmptyBody"})
@Disabled
public class Robot {

    //hardwareMap and telemetry
    HardwareMap hardwareMap;
    Telemetry telemetry;


    //motors
    public DcMotor wheelLF;
    public DcMotor wheelRF;
    public DcMotor wheelRB;
    public DcMotor wheelLB;
    public DcMotor intakeWheelLeft;
    public DcMotor intakeWheelRight;

    //motorPositions
    public double wheelLFPos;
    public double wheelRFPos;
    public double wheelRBPos;
    public double wheelLBPos;

    //servos
    public Servo buildPlateServoLeft;
    public Servo buildPlateServoRight;
    public Servo zoneReachArmServo;
    public Servo dropCapStoneServo;

    //IMU
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    public double targetAngle;



    //constructor
    public Robot(HardwareMap inputHardwareMap, Telemetry inputTelemetry){
        //assign hardwareMap and telemetry
        hardwareMap = inputHardwareMap;
        telemetry = inputTelemetry;

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
        zoneReachArmServo = hardwareMap.get(Servo.class, "armFoldOutServo");

        //set servo range
        buildPlateServoLeft.scaleRange(.5, 1);
        buildPlateServoRight.scaleRange(0, .5);
        dropCapStoneServo.scaleRange(.2, .7);
        zoneReachArmServo.scaleRange(0, 1);

        //set servo to default position
        buildPlateServoLeft.setPosition(0);
        buildPlateServoRight.setPosition(1);
        dropCapStoneServo.setPosition(1);
        zoneReachArmServo.setPosition(0);

        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }



    //imu
    public void WaitForGyroCalibration() throws InterruptedException{
        //wait for gyro calibration
        while (!imu.isGyroCalibrated()) {
            Thread.sleep(50);
        }
    }

    public void UpdateGlobalAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        globalAngle += deltaAngle;

        globalAngle = MathFunctions.clambAngle(globalAngle);

        lastAngles = angles;
    }
    private double GetWheelCorrection(){
        double gain = .05;

        UpdateGlobalAngle();

        double correction = 0;

        if (globalAngle != targetAngle)
            correction = targetAngle - globalAngle;

        correction = correction * gain;

        return correction;
    }



    //teleop driving
    public void DriveWithController(double joyX, double joyY, double joyR){
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



    //buildPlateHooks
    public void BuildPlateHooksDown(){
        buildPlateServoLeft.setPosition(1);
        buildPlateServoRight.setPosition(0);
    }
    public void BuildPlateHooksUp(){
        buildPlateServoLeft.setPosition(0);
        buildPlateServoRight.setPosition(1);
    }


    //IntakeWheels
    public void IntakeWheelsIn(){
        intakeWheelLeft.setPower(-1);
        intakeWheelRight.setPower(-1);
    }
    public void IntakeWheelsOut(){
        intakeWheelLeft.setPower(1);
        intakeWheelRight.setPower(1);
    }
    public void IntakeWheelsOff(){
        intakeWheelLeft.setPower(0);
        intakeWheelRight.setPower(0);
    }


    //capstoneDropper
    public void DropCapstone(){
        //drop capstone
        dropCapStoneServo.setPosition(0);

        //wait for being dropped
        while (dropCapStoneServo.getPosition() != 0){
            //wait
        }

        //turn servo back
        dropCapStoneServo.setPosition(1);
    }


    //zoneReachArm
    public void ZoneReachArmOut(){
        zoneReachArmServo.setPosition(1);
    }
    public void ZoneReachArmIn(){
        zoneReachArmServo.setPosition(0);
    }



    //autonomous driving methods
    public void DriveForward(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, false);

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

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            wheelLF.setPower(power - wheelCorrection);
            wheelRF.setPower(power + wheelCorrection);
            wheelRB.setPower(power + wheelCorrection);
            wheelLB.setPower(power - wheelCorrection);

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
    public void DriveBackward(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, false);

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

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            wheelLF.setPower(-power - wheelCorrection);
            wheelRF.setPower(-power + wheelCorrection);
            wheelRB.setPower(-power + wheelCorrection);
            wheelLB.setPower(-power - wheelCorrection);

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

    public void DriveLeft(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, true);

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

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            wheelLF.setPower(-power - wheelCorrection);
            wheelRF.setPower(power + wheelCorrection);
            wheelRB.setPower(-power + wheelCorrection);
            wheelLB.setPower(power - wheelCorrection);

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
    public void DriveRight(int distance, double power){
        distance = MathFunctions.CMToTicks(distance, true);

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

        while (wheelLFPos < distance && wheelRFPos < distance && wheelRBPos < distance && wheelLBPos < distance){

            // Use gyro to drive in a straight line.
            double wheelCorrection = GetWheelCorrection();

            wheelLF.setPower(power - wheelCorrection);
            wheelRF.setPower(-power + wheelCorrection);
            wheelRB.setPower(power + wheelCorrection);
            wheelLB.setPower(-power - wheelCorrection);

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

    public void Turn(int turnAmount, double power){
        double flexibility = 5;

        //set targetAngle
        targetAngle -= turnAmount;

        targetAngle = MathFunctions.clambAngle(targetAngle);


        while (globalAngle < targetAngle - flexibility){

            //set power
            wheelLF.setPower(power * -1);
            wheelRF.setPower(power * 1);
            wheelRB.setPower(power * 1);
            wheelLB.setPower(power * -1);

            //update globalAngle
            UpdateGlobalAngle();
        }

        while (globalAngle > targetAngle + flexibility){

            //set power
            wheelLF.setPower(power * 1);
            wheelRF.setPower(power * -1);
            wheelRB.setPower(power * -1);
            wheelLB.setPower(power * 1);

            //update globalAngle
            UpdateGlobalAngle();
        }

        //set power to 0
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }
}
