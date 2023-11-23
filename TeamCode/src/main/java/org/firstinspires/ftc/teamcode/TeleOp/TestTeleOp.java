package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestTeleOp")
public class TestTeleOp extends OpMode {

    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    public DcMotor arm;
    public DcMotor armRotate; //
    public Servo claw;
    public Servo drone;
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

        fL = hardwareMap.get(DcMotor.class, "frontLeft");
        fR = hardwareMap.get(DcMotor.class, "frontRight");
        bL = hardwareMap.get(DcMotor.class, "backLeft");
        bR = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "armMove");
        armRotate = hardwareMap.get(DcMotor.class, "armSwivel"); //
        claw = hardwareMap.get(Servo.class, "clawGrab");

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
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

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 0.75;
        double rx = -gamepad1.right_stick_x * 0.75;
        int lim1 = 950;
        int lim2 = 1262;

        fL.setPower(Range.clip(y +x +rx,minPower,maxPower));
        bL.setPower(Range.clip(y -x +rx,minPower,maxPower));
        fR.setPower(Range.clip(y -x -rx,minPower,maxPower));
        bR.setPower(Range.clip(y +x -rx,minPower,maxPower));


        //Up
        if (gamepad1.dpad_up) {
            if (arm.getCurrentPosition() > lim1) {
                arm.setPower(0.28);
                telemetry.addData("Position", arm.getCurrentPosition());
            }
            else {
                arm.setPower(0.93);
                telemetry.addData("Position", arm.getCurrentPosition());
            }
        } else if (!gamepad1.dpad_up) {

            if (arm.getCurrentPosition() > lim1 && arm.getCurrentPosition() <= lim2) {

                arm.setPower(0);

            }
            else if (arm.getCurrentPosition() > lim2) {

                arm.setPower(-0.114);

            }
            else {

                arm.setPower(0.214);
            }
        }
        //guide
        if (gamepad1.dpad_down) {

            arm.setPower(-0.78);
            telemetry.addData("Position", arm.getCurrentPosition());

        }

        //slow movement
        if (gamepad1.y) {

            arm.setPower(0.47);
            telemetry.addData("Position", arm.getCurrentPosition());

        }
        //guide
        if (gamepad1.a) {

            arm.setPower(-0.27);
            telemetry.addData("Position", arm.getCurrentPosition());

        }


        //Rotate Claw
        if (gamepad1.left_bumper) {

            armRotate.setPower(0.3);

        } else if (!gamepad1.left_bumper) {

            armRotate.setPower(0);

        }

        //Reset Claw
        if (gamepad1.right_bumper) {

            armRotate.setPower(-0.3);

        }

        //Close
        if (gamepad1.b) {

            claw.setPosition(0.35);
        }

        //Open
        if (gamepad1.x) {

            claw.setPosition(0.20);
        }
    }
}
