//btw the numbers stand for which spike mark its on, starting from the left and working clockwise. The left and right is from the drivers perspective

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "BlueLeft3Auto")
public class BlueLeft3Auto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor armRotate;
    DcMotor armElevate;
    Servo claw;

    public int lfPos;
    public int rfPos;
    public int lbPos;
    public int rbPos;
    public int aePos;
    public int arPos;
    public int cPos;

    static final int tpi = 50; //length ratio, these is a value from last year, so it'll prob need to be tweaked
    static final double lr = 0.8535; //length ratio, these is a value from last year, so it'll prob need to be tweaked
    static final double dr = 0.1; //degree ratio, value needs to be tested & tweaked

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armElevate = hardwareMap.get(DcMotor.class, "armElevate");
        claw = hardwareMap.get(Servo.class, "claw");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armElevate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElevate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();


        while (opModeIsActive()) {
            forward(10,0.8); //just assuming that 10 inches is a tile, will be tweaked
            armRotate(90,0.8);
            armElevate(-90,0.8);
            claw(1); //also needs to be tweaked idk servo ratios. rn im using 1 for open and 0 for closed
            armElevate(90,0.8);
            claw(0);
            armRotate(-90,0.8);
            backward(10,0.8);
            left(20,0.8);

        }


    }

    public void forward(int inches, double speed) {

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            // fetch motor positions
            lfPos = frontLeft.getCurrentPosition();
            rfPos = frontRight.getCurrentPosition();
            lbPos = backLeft.getCurrentPosition();
            rbPos = backRight.getCurrentPosition();

            // calculate new targets
            lfPos -= (int) (inches * tpi * lr);
            rfPos -= (int) (inches * tpi * lr);
            lbPos -= (int) (inches * tpi * lr);
            rbPos -= (int) (inches * tpi * lr);

            // move robot to new position
            frontLeft.setTargetPosition(lfPos);
            frontRight.setTargetPosition(rfPos);
            backLeft.setTargetPosition(lbPos);
            backRight.setTargetPosition(rbPos);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {

                frontLeft.setPower(Math.abs(speed));
                frontRight.setPower(Math.abs(speed));
                backLeft.setPower(Math.abs(speed));
                backRight.setPower(Math.abs(speed));

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void backward(int inches, double speed) {

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            // fetch motor positions
            lfPos = frontLeft.getCurrentPosition();
            rfPos = frontRight.getCurrentPosition();
            lbPos = backLeft.getCurrentPosition();
            rbPos = backRight.getCurrentPosition();

            // calculate new targets
            lfPos -= (int) (inches * tpi * lr);
            rfPos -= (int) (inches * tpi * lr);
            lbPos -= (int) (inches * tpi * lr);
            rbPos -= (int) (inches * tpi * lr);

            // move robot to new position
            frontLeft.setTargetPosition(lfPos);
            frontRight.setTargetPosition(rfPos);
            backLeft.setTargetPosition(lbPos);
            backRight.setTargetPosition(rbPos);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {

                frontLeft.setPower(Math.abs(speed));
                frontRight.setPower(Math.abs(speed));
                backLeft.setPower(Math.abs(speed));
                backRight.setPower(Math.abs(speed));

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void left(int inches, double speed) {

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            // fetch motor positions
            lfPos = frontLeft.getCurrentPosition();
            rfPos = frontRight.getCurrentPosition();
            lbPos = backLeft.getCurrentPosition();
            rbPos = backRight.getCurrentPosition();

            // calculate new targets
            lfPos -= (int) (inches * tpi * lr);
            rfPos += (int) (inches * tpi * lr);
            lbPos += (int) (inches * tpi * lr);
            rbPos -= (int) (inches * tpi * lr);

            // move robot to new position
            frontLeft.setTargetPosition(lfPos);
            frontRight.setTargetPosition(rfPos);
            backLeft.setTargetPosition(lbPos);
            backRight.setTargetPosition(rbPos);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {

                frontLeft.setPower(Math.abs(speed));
                frontRight.setPower(Math.abs(speed));
                backLeft.setPower(Math.abs(speed));
                backRight.setPower(Math.abs(speed));

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void right(int inches, double speed) {

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            // fetch motor positions
            lfPos = frontLeft.getCurrentPosition();
            rfPos = frontRight.getCurrentPosition();
            lbPos = backLeft.getCurrentPosition();
            rbPos = backRight.getCurrentPosition();

            // calculate new targets
            lfPos += (int) (inches * tpi * lr);
            rfPos -= (int) (inches * tpi * lr);
            lbPos -= (int) (inches * tpi * lr);
            rbPos += (int) (inches * tpi * lr);

            // move robot to new position
            frontLeft.setTargetPosition(lfPos);
            frontRight.setTargetPosition(rfPos);
            backLeft.setTargetPosition(lbPos);
            backRight.setTargetPosition(rbPos);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {

                frontLeft.setPower(Math.abs(speed));
                frontRight.setPower(Math.abs(speed));
                backLeft.setPower(Math.abs(speed));
                backRight.setPower(Math.abs(speed));

            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void armRotate(int degrees, double speed) {

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            // fetch motor positions
            arPos = armRotate.getCurrentPosition();

            // calculate new targets
            arPos += (int) (degrees * dr);

            // move robot to new position
            armRotate.setTargetPosition(arPos);

            armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (armRotate.isBusy()) {

                armRotate.setPower(Math.abs(speed));

            }

            // Stop all motion;
            armRotate.setPower(0);

            // Turn off RUN_TO_POSITION
            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void armElevate(int degrees, double speed) {

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            // fetch motor positions
            aePos = armElevate.getCurrentPosition();

            // calculate new targets
            aePos += (int) (degrees * dr);

            // move robot to new position
            armElevate.setTargetPosition(arPos);

            armElevate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (armElevate.isBusy()) {

                armElevate.setPower(Math.abs(speed));

            }

            // Stop all motion;
            armElevate.setPower(0);

            // Turn off RUN_TO_POSITION
            armElevate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void claw(int position) {

        if (opModeIsActive()) {
            claw.setPosition(position);
        }
    }

}





