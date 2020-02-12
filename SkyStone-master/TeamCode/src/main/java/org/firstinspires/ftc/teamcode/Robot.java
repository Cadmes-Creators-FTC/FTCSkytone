package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("WeakerAccess")
@Disabled
public class Robot extends OpMode {

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
    public Servo armFoldOutServo;
    public Servo dropCapStoneServo;

    //IMU
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;
    public double targetAngle;

    public Robot(){
        //assign drive wheels
        this.wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        this.wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        this.wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        this.wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //reverse drive wheels
        this.wheelLF.setDirection(DcMotor.Direction.REVERSE);
        this.wheelLB.setDirection(DcMotor.Direction.REVERSE);

        //assign intake wheels
        this.intakeWheelLeft = hardwareMap.get(DcMotor.class, "IntakeWheelLeft");
        this.intakeWheelRight = hardwareMap.get(DcMotor.class, "IntakeWheelRight");

        //reverse intake wheels
        this.intakeWheelRight.setDirection(DcMotor.Direction.REVERSE);

        //assign servos
        this.buildPlateServoLeft = hardwareMap.get(Servo.class, "BuildPlateServoLeft");
        this.buildPlateServoRight = hardwareMap.get(Servo.class, "BuildPlateServoRight");
        this.dropCapStoneServo = hardwareMap.get(Servo.class, "DropCapStoneServo");
        this.armFoldOutServo = hardwareMap.get(Servo.class, "armFoldOutServo");

        //set servo range
        this.buildPlateServoLeft.scaleRange(.5, 1);
        this.buildPlateServoRight.scaleRange(0, .5);
        this.dropCapStoneServo.scaleRange(.2, .7);
        this.armFoldOutServo.scaleRange(0, 1);

        //set servo to default position
        this.buildPlateServoLeft.setPosition(0);
        this.buildPlateServoRight.setPosition(1);
        this.dropCapStoneServo.setPosition(1);
        this.armFoldOutServo.setPosition(0);

        //imu
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        this.imu.initialize(parameters);
    }

    public void UpdateGlobalAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        globalAngle += deltaAngle;

        globalAngle = MathFunctions.clambAngle(globalAngle);

        lastAngles = angles;
    }


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
