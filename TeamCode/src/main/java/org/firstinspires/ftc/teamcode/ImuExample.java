package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "ImuExample")


public class ImuExample extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();

        driveStraight(0.4, 1000);
        turnRight(90);
        driveStraight(-0.4, 1000);
        turnLeft(-90);

    }

    public void driveStraight(double power, long time) {

        motorLeft.setPower(power);
        motorRight.setPower(power);
        sleep(time);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(250);

    }


    public void turnRight(int angleReading) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.update();
        sleep(500);

        motorLeft.setPower(0.2);
        motorRight.setPower(-0.2);

        while (angles.firstAngle > -angleReading && !isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ", angles.firstAngle);
            telemetry.update();

        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(250);

    }


