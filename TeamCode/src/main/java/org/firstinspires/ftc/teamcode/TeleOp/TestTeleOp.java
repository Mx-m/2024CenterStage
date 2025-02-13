package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestTeleOp")
public class TestTeleOp extends OpMode {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor arm;
    public DcMotor armRotate; //
    public Servo claw;
    public enum LiftState {
        LIFT_START,
        LIFT_DROP,
        LIFT_CLOSED,
        LIFT_RETRACT
    }

    LiftState liftState = LiftState.LIFT_START;

    ElapsedTime et = new ElapsedTime();


    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate"); //
        claw = hardwareMap.get(Servo.class, "claw");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }



    @Override
    public void loop() {

        /**
        switch (liftState) {

            case LIFT_START:

                if (gamepad1.a) {

                    arm.setPower(0.86);

                }

                liftState = LiftState.LIFT_DROP;
                break;

        }**/

        double minPower = -0.8;
        double maxPower = 0.8;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 0.75;
        double rx = gamepad1.right_stick_x * 0.5;

        frontLeft.setPower(Range.clip(y +x +rx,minPower,maxPower));
        backLeft.setPower(Range.clip(y -x +rx,minPower,maxPower));
        frontRight.setPower(Range.clip(y -x -rx,minPower,maxPower));
        backRight.setPower(Range.clip(y +x -rx,minPower,maxPower));


        //Up
        if (gamepad1.dpad_up) {

            arm.setPower(0.86);
            telemetry.addData("Position", arm.getCurrentPosition());

        } else if (!gamepad1.dpad_up) {

            if (arm.getCurrentPosition() > 400) {

                arm.setPower(-0.214);

            } else {

                arm.setPower(0.274);
            }
        }
        //guide
        if (gamepad1.dpad_down) {


            arm.setPower(-0.6);

        }


        //Rotate Claw
        if (gamepad1.a) {

            armRotate.setPower(0.2);

        } else if (!gamepad1.a) {

            armRotate.setPower(0);

        }

        //Reset Claw
        if (gamepad1.b) {

            armRotate.setPower(-0.2);

        }

        //Close
        if (gamepad1.right_bumper) {

            claw.setPosition(0.32);
        }

        //Open
        if (gamepad1.left_bumper) {

            claw.setPosition(0.23);
        }
    }
}
