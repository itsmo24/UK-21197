package org.firstinspires.ftc.teamcode.Autonomous.team004;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;



public class Autonomous1 extends LinearOpMode {

     IMU imu;
     DcMotorEx backLeft;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;
     YawPitchRollAngles orientation;
    CRServo rightShoulder;
     CRServo leftShoulder;
     CRServo wrist;
     Servo gripper;
    int ticks = 1120;
    double TargetPosition;
    double circumference = 2.95*Math.PI;
    public void Move_right(double x) {
        TargetPosition = (x/circumference)*ticks;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(1);
        frontRight.setPower(-1);
        backLeft.setPower(-1);
        backRight.setPower(1);
        while (-backLeft.getCurrentPosition() < TargetPosition && -frontLeft.getCurrentPosition() < TargetPosition) {

        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
    }
    public void Move_left(double x) {
        TargetPosition = (x/circumference)*ticks;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-1);
        frontRight.setPower(1);
        backLeft.setPower(1);
        backRight.setPower(-1);
        while (backLeft.getCurrentPosition() < TargetPosition && frontLeft.getCurrentPosition() < TargetPosition) {

        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
    }
    public void Move_to_Position(double x) {
        TargetPosition = (x/circumference)*ticks;
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);
        while (backLeft.getCurrentPosition() < TargetPosition && frontRight.getCurrentPosition() < TargetPosition
        ){
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
        telemetry.addData("Status", "Moving forward");
        telemetry.addData("Encoder Values", "FL: %d, FR: %d, BL: %d, BR: %d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.update();
    }
    public void Move_back(double x) {
        TargetPosition = (x/circumference)*ticks;

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-1);
        frontRight.setPower(-1);
        backLeft.setPower(-1);
        backRight.setPower(-1);
        while (-backLeft.getCurrentPosition() < TargetPosition && -frontLeft.getCurrentPosition() < TargetPosition) {

        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
    }
    double currentAngle;
    public void rotateCCW(int targetOrientationAngle, float power) {
        currentAngle = 0;
        imu.resetYaw();
        backLeft.setPower(-power);
        backRight.setPower(power);
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        while (currentAngle < targetOrientationAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
    }
    public void rotateCW(int targetOrientationAngle, float power) {
        currentAngle = 0;

        imu.resetYaw();
        backLeft.setPower(power);
        backRight.setPower(-power);
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        while (-currentAngle < targetOrientationAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(250);
    }
    public void grabber(boolean gripperCheck){
        if (gripperCheck){
            gripper.setPosition(0.1);
            sleep(200);
            gripper.setPosition(0.2);
            sleep(200);
            gripper.setPosition(0.3);
            sleep(200);
            gripper.setPosition(0.4);
            sleep(200);
            gripper.setPosition(0.5);
            sleep(200);
            gripper.setPosition(0.6);
            sleep(200);
            gripper.setPosition(0.7);
            sleep(200);
            gripper.setPosition(0.8);
            sleep(200);
            gripper.setPosition(0.9);
            sleep(200);
            gripper.setPosition(1);
            sleep(1500);
        }
        else {
            gripper.setPosition(0.9);
            sleep(200);
            gripper.setPosition(0.8);
            sleep(200);
            gripper.setPosition(0.7);
            sleep(200);
            gripper.setPosition(0.6);
            sleep(200);
            gripper.setPosition(0.5);
            sleep(200);
            gripper.setPosition(0.4);
            sleep(200);
            gripper.setPosition(0.3);
            sleep(200);
            gripper.setPosition(0.2);
            sleep(200);
            gripper.setPosition(0);
            sleep(1400);

        }
    }
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightShoulder = hardwareMap.get(CRServo.class, "rightShoulder");
        leftShoulder = hardwareMap.get(CRServo.class, "leftShoulder");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        gripper = hardwareMap.get(Servo.class, "gripper");



        imu = hardwareMap.get(IMU.class, "imu");


        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        // Prompt user to press start button.

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rightShoulder.setDirection(CRServo.Direction.REVERSE);
        gripper.setPosition(1);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();




        if (opModeIsActive()) {
            // Put run blocks here.
            Move_to_Position(2);


            Move_back(2);


            Move_right(2);

            Move_left(2);

            //Move_to_Position(250);
            wrist.setPower(-0.4);
            sleep(1600);
            wrist.setPower(0);

            grabber( false);


            sleep(1400);
            Move_back(0.2);
            sleep(250);
            grabber(true);
            wrist.setPower(0.6);
            sleep(1600);
            wrist.setPower(0);
            Move_left(0.2);
            Move_to_Position(0.2);

            rotateCW(90,0.3f);

            Move_to_Position(0.2);


            Move_right(0.2);
            Move_to_Position(0.2);
            rotateCCW(90,0.3f);
            rotateCCW(90,0.3f);
            //Move_left(1120);


            rightShoulder.setPower(0.5);
            leftShoulder.setPower(0.5);
            sleep(1600);
            rightShoulder.setPower(0);
            leftShoulder.setPower(0);


            sleep(200);
            gripper.setPosition(0);
            sleep(700);

            rightShoulder.setPower(-0.5);
            leftShoulder.setPower(-0.5);
            sleep(1800);
            rightShoulder.setPower(0);
            leftShoulder.setPower(0);

            Move_left(0.2);



        }
    }



}

