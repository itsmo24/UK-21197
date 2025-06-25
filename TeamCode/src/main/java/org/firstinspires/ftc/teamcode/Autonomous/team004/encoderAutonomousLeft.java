package org.firstinspires.ftc.teamcode.Autonomous.team004;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@Autonomous(name = "encoderAutonomousREALREAL")
public class encoderAutonomousLeft extends LinearOpMode {

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
        imu = hardwareMap.get(IMU.class, "imu");
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
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper.setPosition(1);
        imu.resetYaw();



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


            rotateCCW(180,300);
            /*sensor(20, 3000);
            armUp(15, 0.5);
            sleep(300);
            wrist.setPower(1);
            sleep(1300);
            wrist.setPower(0);
            sleep(1300);
            armUp(9, -0.5);
            grabber(false);
            armUp(5, -0.5);
            wrist.setPower(-1);
            sleep(1300);
            wrist.setPower(0);
            backward(27, 3000);
            sideways(50, 3000);*/
            /*sideways(-25,3500);
            sensor(20, 3500);
            armUp(15, 0.5);
            wrist.setPower(1);
            sleep(1200);
            wrist.setPower(0);
            sleep(100);
            armUp(9, -0.5);
            grabber(false);
            armUp(4, -0.5);

            backward(12, 3500);
            sideways(60,3500);
            turn(180,1);
            sleep(1300);

            wrist.setPower(1);
            sleep(300);
            grabber(true);
            sleep(300);

            wrist.setPower(-1);
            sleep(1100);
            turn(0,1);
            sideways(-40,3500);
            sleep(1300);
            sensor(20, 3500);
            armUp(15, 0.5);
            wrist.setPower(1);
            sleep(1200);
            wrist.setPower(0);
            sleep(300);
            armUp(9, -0.5);
            grabber(false);
            backward(27, 3500);
            sideways(50, 3500);
            armUp(5, -0.5);
            wrist.setPower(-1);
            sleep(800);
            wrist.setPower(0);*/
            //turn(180,1);




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
            telemetry.addData("Yaw", orientation.getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
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
        sleep((long) (100*(angle)));
        leftArm.setPower(0);
        rightArm.setPower(0);
        sleep(100);

    }

    public void grabber(boolean clawCheck) {
        double start = clawCheck ? 0.2: 0.8;
        double end = clawCheck ? 1.0 : 0.0;
        double step = 0.2 * (clawCheck ? 1 : -1);

        for (double pos = start; clawCheck ? (pos <= end) : (pos >= end); pos += step) {
            gripper.setPosition(pos);
            sleep(100);
        }

        sleep(500);
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
    public void rotateCW(double targetOrientationAngle, float velocity) {
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
        }

        stopMotors();
    }

    public void turn(int targetAngle, double pPower) {
        // Reset yaw at the start of the turn
        int currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // Determine direction
        double power = (currentAngle < targetAngle) ? pPower : -pPower;

        while (Math.abs(currentAngle - targetAngle) > 4 && opModeIsActive()) { // Allow small error margin
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);

            currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }

        stopMotors(); // Stop motors after reaching the angle
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
