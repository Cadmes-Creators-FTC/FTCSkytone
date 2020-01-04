package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;


@TeleOp(name = "Test", group = "MainGroup")
public class Test extends LinearOpMode {

    //drive
    private DcMotor wheelLF;
    private DcMotor wheelRF;
    private DcMotor wheelRB;
    private DcMotor wheelLB;

    //IMU
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    double correction;
    double globalAngle;
    double power = .3;

    @Override
    public void runOpMode () {


        MapHardware();

        telemetry.addData("Mode", "calibrating");
        telemetry.update();

        //wait for gyro callibration
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Waiting for start");
        telemetry.addData("imu calibration status", imu.getCalibrationStatus());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            wheelLF.setPower(-power + correction);
            wheelLB.setPower(-power + correction);
            wheelRF.setPower(-power - correction);
            wheelRB.setPower(-power - correction);

            //update telemetry
            telemetry.update();
        }

        // turn the motors off.
        wheelLF.setPower(0);
        wheelRF.setPower(0);
        wheelRB.setPower(0);
        wheelLB.setPower(0);
    }



    //map hardware
    private void MapHardware(){
        //assign drive wheels
        wheelLF = hardwareMap.get(DcMotor.class, "WheelLF");
        wheelRF = hardwareMap.get(DcMotor.class, "WheelRF");
        wheelRB = hardwareMap.get(DcMotor.class, "WheelRB");
        wheelLB = hardwareMap.get(DcMotor.class, "WheelLB");

        //reverse drive wheels
        wheelRF.setDirection(DcMotor.Direction.REVERSE);
        wheelRB.setDirection(DcMotor.Direction.REVERSE);

        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    private double checkDirection()
    {
        double angle = getAngle();
        double gain = .05;

        if (angle == 0)
            correction = 0;
        else
            correction = -angle;

        correction = correction * gain;

        return correction;
    }

    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
