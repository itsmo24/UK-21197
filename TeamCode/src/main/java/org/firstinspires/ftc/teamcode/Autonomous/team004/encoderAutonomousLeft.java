package org.firstinspires.ftc.teamcode.Autonomous.team004;

import static android.icu.lang.UProperty.MATH;
import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@Autonomous(name = "encoderAutonomousleft")
public class encoderAutonomousLeft extends LinearOpMode {

    // Declare Motors and Servos
    IMU imu;
    mainMethods move;
    YawPitchRollAngles orientation;
    DistanceSensor rangeSensor;
    //TouchSensor touchSensor;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight, rightArm, leftArm;
    private CRServo leftWrist, rightWrist ;
    private Servo gripper;
    private double circumference = 2.95 * Math.PI;

    private double imuZeroYaw = 0;

    @Override
    public void runOpMode() {
        // Initialize
        ElapsedTime runtime = new ElapsedTime();

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        leftArm = hardwareMap.get(DcMotorEx.class, "leftArm");
        rightArm = hardwareMap.get(DcMotorEx.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        leftWrist = hardwareMap.get(CRServo.class, "leftWrist");
        rightWrist = hardwareMap.get(CRServo.class, "rightWrist");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        //touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        sleep(500); // Allow some time for calibration
        imuZeroYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        // Set motor directions
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rightArm.setDirection(DcMotorEx.Direction.REVERSE);
        rightWrist.setDirection(CRServo.Direction.REVERSE);

        // Reset Encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motors to Run Using Encoder
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        gripper.setPosition(1);
        imuZeroYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


        telemetry.addData("IMU", "Initialized");
        telemetry.update();

        // Set PIDF Values
        /*frontLeft.setVelocityPIDFCoefficients(1.15, 0.115, 0, 11.5);
        frontRight.setVelocityPIDFCoefficients(3.45, 0.345, 0, 34.5);
        backLeft.setVelocityPIDFCoefficients(1.2, 0.12, 0, 12);
        backRight.setVelocityPIDFCoefficients(1.13, 0.113, 0, 11.3);*/

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start autonomous movement
        if (opModeIsActive()) {

            sensor(20, 3000);
            armUp(6, 0.5);
            sleep(300);
            wrist(1);
            sleep(800);
            wrist(0);
            armUp(4, -0.5);
            grabber(false);
            //armUp(10, 0.5);
            wrist(-1);
            sleep(800);
            wrist(0);
            backward(1, 3000);
            sideways(7, 3000);
            forward(14, 3000);
            sideways(7, 3000);
            backward(28, 3000);
            forward(10, 3000);
            armUp(25, 0.5);
            backward(12, 300);
            grabber(true);
            armUp(20, -0.5);
            sideways(18, -3000);
            sensor(20, 3000);
            armUp(4.5, 0.5);
            sleep(300);
            wrist(1);
            sleep(800);


            armUp(6, 0.5);

            grabber(false);
            wrist(-1);
            sleep(1300);
            wrist(0);
            backward(12, 3000);


            sideways(18, 3000);


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
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    public void stopMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void wrist(double power) {
        rightWrist.setPower(power);
        leftWrist.setPower(power);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

    public void left(double distance, double velocity) {
        resetEncoders();
        backLeft.setVelocity(velocity);
        backRight.setVelocity(-velocity);
        frontLeft.setVelocity(-velocity);
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
        sleep((long) (100 * (angle)));
        leftArm.setPower(0);
        rightArm.setPower(0);
        sleep(100);

    }


    public void grabber(boolean clawCheck) {
        double start = clawCheck ? 0.2 : 0.8;
        double end = clawCheck ? 1.0 : 0.0;
        double step = 0.2 * (clawCheck ? 1 : -1);

        for (double pos = start; clawCheck ? (pos <= end) : (pos >= end); pos += step) {
            gripper.setPosition(pos);
            sleep(100);
        }

        sleep(300);
    }

    public void rotateCCW(double targetOrientationAngle, float velocity) {
        double targetOrientationAngleRad = Math.toRadians(targetOrientationAngle);
        double currentAngle = 0;
        resetEncoders();

        backLeft.setPower(-velocity);
        backRight.setPower(velocity);
        frontLeft.setPower(-velocity);
        frontRight.setPower(velocity);

        while (currentAngle < targetOrientationAngleRad) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.RADIANS);
        }

        stopMotors();
    }

    /*public void rotateCW(double targetOrientationAngle, float velocity) {
        double targetOrientationAngleRad = Math.toRadians(targetOrientationAngle);
        double currentAngle = 0;
        resetEncoders();
        imu.resetYaw();

        backLeft.setPower(velocity);
        backRight.setPower(-velocity);
        frontLeft.setPower(velocity);
        frontRight.setPower(-velocity);

        while (currentAngle < targetOrientationAngleRad) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.RADIANS);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetOrientationAngle);
        }


        stopMotors();
    }*/
    public void rotateCW(double targetAngleDegrees, double power) {
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double target = normalizeAngle(startAngle + targetAngleDegrees);

        double current = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while (opModeIsActive() && Math.abs(normalizeAngle(target - current)) > 2) {
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            current = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            telemetry.addData("Start Angle", startAngle);
            telemetry.addData("Target", target);
            telemetry.addData("Current", current);
            telemetry.addData("Error", normalizeAngle(target - current));
            telemetry.update();
        }

        stopMotors();
    }


    public void turn(int targetAngle, double pPower) {
        // Store yaw offset to simulate "reset"
        double yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Get normalized yaw relative to zero
        double currentAngle = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset);

        // Determine direction
        double power = (currentAngle < targetAngle) ? pPower : -pPower;

        while (Math.abs(currentAngle - targetAngle) > 4 && opModeIsActive()) {
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);

            currentAngle = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset);

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }

        stopMotors();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // Returns current yaw relative to imuZeroYaw in degrees normalized between -180 and 180
    private double getRelativeYaw() {
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return normalizeAngle(currentYaw - imuZeroYaw);
    }


    public void sideways(double distance, double velocity) {
        double targetPosition = (distance / circumference) * 1120;
        resetEncoders();


        frontLeft.setPower(velocity);
        frontRight.setPower(-velocity);
        backLeft.setPower(-velocity);
        backRight.setPower(velocity);


        while (Math.abs(frontLeft.getCurrentPosition()) < targetPosition && opModeIsActive()) {
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", frontLeft.getCurrentPosition());
            telemetry.update();
        }


        stopMotors();
    }
}