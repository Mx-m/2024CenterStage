package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Demo Auto")

public class DemoAuto extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    @Override

    public void runOpMode()
    {

        motorLeft = hardwareMap.dcMotor.get ("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        driveByTime(1.0, 1000);
        sleep(1000);
        driveByTime(-1.0, 1000);

    }

    public void driveByTime (double power, long time){

        motorLeft.setPower(power);
        motorRight.setPower(power);

        sleep(time);

        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);
    }


}

