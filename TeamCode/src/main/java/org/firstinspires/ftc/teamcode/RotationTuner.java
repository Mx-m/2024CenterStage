package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "RotationTuner", group = "Tuner")
public class RotationTuner extends OpMode {

    private GamePadController gamepad;
    private Motor driveLeft, driveRight;
    private BNO055IMU imu;

    private Orientation lastAngles;
    private double currentHeading = 0;
    private PIDController pidRotate;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePadController(gamepad1);

        pidRotate = new PIDController(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
        pidRotate.setTolerance(Vals.rotate_position_tolerance, Vals.rotate_velocity_tolerance);

        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveRight.setInverted(true);
        driveLeft.setRunMode(Motor.RunMode.RawPower);
        driveRight.setRunMode(Motor.RunMode.RawPower);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }

    @Override
    public void loop() {
        gamepad.update();

        if(gamepad.isARelease()) {
            driveLeft.set(0);
            driveRight.set(0);
            resetAngle();
        }

        double power = rotate(Vals.rotate_target);

        double leftPower = -power;
        double rightPower = power;

        if(gamepad.gamepad.right_bumper) {
            leftPower += .4;
            rightPower += .4;
            leftPower = Math.max(.93, leftPower);
            rightPower = Math.max(.93, rightPower);
        } else if(gamepad.gamepad.left_bumper) {
            leftPower -= .4;
            rightPower -= .4;
            leftPower = Math.min(-.93, leftPower);
            rightPower = Math.min(-.93, rightPower);
        }

        driveLeft.set(leftPower);
        driveRight.set(rightPower);

        telemetry.addData("PID Error", pidRotate.getPositionError());
        telemetry.addData("Current Heading", lastAngles.firstAngle);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Current Power", power);

    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = 0;
        pidRotate.reset();
    }

    private double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < -180) {
            deltaAngle += 360;
        } else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        currentHeading += deltaAngle;
        lastAngles = angles;

        return currentHeading;
    }

    private double rotate(double degrees) {
        if(Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        double power = pidRotate.calculate(updateHeading(), degrees);
        return power;

    }

}
