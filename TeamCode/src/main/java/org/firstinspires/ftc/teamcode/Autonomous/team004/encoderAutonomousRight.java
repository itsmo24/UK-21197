package org.firstinspires.ftc.teamcode.Autonomous.team004;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@Autonomous(name = "encoderAutonomousRight")
public class encoderAutonomousRight extends LinearOpMode {

    // Declare Motors and Servos
    IMU imu;
    mainMethods move;
    YawPitchRollAngles orientation;
    DistanceSensor rangeSensor;
    //TouchSensor touchSensor;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private CRServo leftArm, rightArm, wrist;
    private Servo gripper;
    private double circumference = 2.95 * Math.PI;

    @Override
    public void runOpMode() {
        // Initialize
        ElapsedTime runtime = new ElapsedTime();

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        //touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");


        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(CRServo.Direction.REVERSE);

        // Reset Encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motors to Run Using Encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Values
        frontLeft.setVelocityPIDFCoefficients(1.15, 0.115, 0, 11.5);
        frontRight.setVelocityPIDFCoefficients(3.45, 0.345, 0, 34.5);
        backLeft.setVelocityPIDFCoefficients(1.2, 0.12, 0, 12);
        backRight.setVelocityPIDFCoefficients(1.13, 0.113, 0, 11.3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start autonomous movement
        if (opModeIsActive()) {
            sensor(20, 3000);
            armUp(5, 1);
            wrist.setPower(-0.4);
            sleep(1600);
            wrist.setPower(0);
            armUp(2, -1);
            grabber(false);
            wrist.setPower(0.4);
            sleep(1600);
            wrist.setPower(0);
            backward(30, 3000);
            sideways(50, 3000);



            telemetry.update();

        }
        /*while (opModeIsActive()) {
            if (touchSensor.isPressed()) {
                stopMotors();
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
        }*/
    }

    public void stopMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forward(double distance, double velocity) {
        resetEncoders();
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);

        double time = ((distance * 1120) / circumference) / velocity;
        telemetry.addData("Calculated time", time);
        telemetry.update();

        sleep((long) (time * 1000));
        stopMotors();
    }

    public void backward(double distance, double velocity) {
        resetEncoders();
        backLeft.setVelocity(-velocity);
        backRight.setVelocity(-velocity);
        frontLeft.setVelocity(-velocity);
        frontRight.setVelocity(-velocity);


        double time = ((distance * 1120) / circumference) / velocity;
        telemetry.addData("Calculated time", time);
        telemetry.update();

        sleep((long) (time * 1000));
        stopMotors();
    }

    public void sensor(int targetDistance, double velocity) {
        resetEncoders();

        int currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));

        while (currentDistance > targetDistance) {
            backLeft.setVelocity(velocity);
            backRight.setVelocity(velocity);
            frontLeft.setVelocity(velocity);
            frontRight.setVelocity(velocity);
            currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));
        }
        stopMotors();
        telemetry.addData("Current Distance", currentDistance);
        telemetry.update();
    }

    public void armUp(double angle, double power) {

        leftArm.setPower(power);
        rightArm.setPower(power);
        sleep((long) (10*(angle)));
        stopMotors();
    }

    public void grabber(boolean clawCheck) {
        double start = clawCheck ? 0.1 : 0.9;
        double end = clawCheck ? 1.0 : 0.0;
        double step = 0.1 * (clawCheck ? 1 : -1);

        for (double pos = start; clawCheck ? (pos <= end) : (pos >= end); pos += step) {
            gripper.setPosition(pos);
            sleep(200);
        }

        sleep(500);
    }

    public void rotateCCW(int targetOrientationAngle, float velocity) {
        double currentAngle = 0;
        imu.resetYaw();
        resetEncoders();

        backLeft.setPower(-velocity);
        backRight.setPower(velocity);
        frontLeft.setPower(-velocity);
        frontRight.setPower(velocity);

        while (currentAngle < targetOrientationAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        stopMotors();
    }
    public void rotateCW(int targetOrientationAngle, float velocity) {
        double currentAngle = 0;
        resetEncoders();
        imu.resetYaw();
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setPower(velocity);
        backRight.setPower(-velocity);
        frontLeft.setPower(velocity);
        frontRight.setPower(-velocity);

        while (currentAngle < targetOrientationAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }

        stopMotors();
    }
    public void sideways(double distance, double velocity) {
        double targetPosition = (distance / circumference) * 1120;
        resetEncoders();
        if (velocity > 0) {

            frontLeft.setPower(velocity);
            frontRight.setPower(-velocity);
            backLeft.setPower(-velocity);
            backRight.setPower(velocity);
        } else {

            frontLeft.setPower(-velocity);
            frontRight.setPower(velocity);
            backLeft.setPower(velocity);
            backRight.setPower(-velocity);
        }

        while (Math.abs(frontLeft.getCurrentPosition()) < targetPosition && opModeIsActive()) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", frontLeft.getCurrentPosition());
            telemetry.update();
        }


        stopMotors();
    }
}