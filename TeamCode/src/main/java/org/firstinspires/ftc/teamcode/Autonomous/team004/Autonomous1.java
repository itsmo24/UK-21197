package org.firstinspires.ftc.teamcode.Autonomous.team004;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "(004) Autonomous")
public class Autonomous1 extends LinearOpMode {

     IMU imu_IMU;
     DcMotor back_left_motor;
     DcMotor front_left_motor;
     DcMotor front_right_motor;
     DcMotor back_right_motor;
     YawPitchRollAngles orientation;
    CRServo rightShoulder;
     CRServo leftShoulder;
     CRServo wrist;
     Servo claw;
    int ticks = 1120;
    double TargetPosition;
    double circumference = 2.95*Math.PI;
    public void Move_right(double x) {
        TargetPosition = (x/circumference)*ticks;

        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left_motor.setPower(1);
        front_right_motor.setPower(-1);
        back_left_motor.setPower(-1);
        back_right_motor.setPower(1);
        while (-back_left_motor.getCurrentPosition() < TargetPosition && -front_left_motor.getCurrentPosition() < TargetPosition) {

        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    public void Move_left(double x) {
        TargetPosition = (x/circumference)*ticks;

        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setPower(-1);
        front_right_motor.setPower(1);
        back_left_motor.setPower(1);
        back_right_motor.setPower(-1);
        while (back_left_motor.getCurrentPosition() < TargetPosition && front_left_motor.getCurrentPosition() < TargetPosition) {

        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    public void Move_to_Position(double x) {
        TargetPosition = (x/circumference)*ticks;
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_left_motor.setPower(0.6);
        front_right_motor.setPower(0.6);
        back_left_motor.setPower(0.6);
        back_right_motor.setPower(0.6);
        while (back_left_motor.getCurrentPosition() < TargetPosition && front_right_motor.getCurrentPosition() < TargetPosition
        ){
        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
        telemetry.addData("Status", "Moving forward");
        telemetry.addData("Encoder Values", "FL: %d, FR: %d, BL: %d, BR: %d",
                front_left_motor.getCurrentPosition(), front_right_motor.getCurrentPosition(),
                back_left_motor.getCurrentPosition(), back_right_motor.getCurrentPosition());
        telemetry.update();
    }
    public void Move_back(double x) {
        TargetPosition = (x/circumference)*ticks;

        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setPower(-1);
        front_right_motor.setPower(-1);
        back_left_motor.setPower(-1);
        back_right_motor.setPower(-1);
        while (-back_left_motor.getCurrentPosition() < TargetPosition && -front_left_motor.getCurrentPosition() < TargetPosition) {

        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    double currentAngle;
    public void rotateCCW(int targetOrientationAngle, float power) {
        currentAngle = 0;
        imu_IMU.resetYaw();
        back_left_motor.setPower(-power);
        back_right_motor.setPower(power);
        front_left_motor.setPower(-power);
        front_right_motor.setPower(power);
        while (currentAngle < targetOrientationAngle) {
            orientation = imu_IMU.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    public void rotateCW(int targetOrientationAngle, float power) {
        currentAngle = 0;

        imu_IMU.resetYaw();
        back_left_motor.setPower(power);
        back_right_motor.setPower(-power);
        front_left_motor.setPower(power);
        front_right_motor.setPower(-power);
        while (-currentAngle < targetOrientationAngle) {
            orientation = imu_IMU.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    public void grabber(boolean clawCheck){
        if (clawCheck){
            claw.setPosition(0.1);
            sleep(200);
            claw.setPosition(0.2);
            sleep(200);
            claw.setPosition(0.3);
            sleep(200);
            claw.setPosition(0.4);
            sleep(200);
            claw.setPosition(0.5);
            sleep(200);
            claw.setPosition(0.6);
            sleep(200);
            claw.setPosition(0.7);
            sleep(200);
            claw.setPosition(0.8);
            sleep(200);
            claw.setPosition(0.9);
            sleep(200);
            claw.setPosition(1);
            sleep(1500);
        }
        else {
            claw.setPosition(0.9);
            sleep(200);
            claw.setPosition(0.8);
            sleep(200);
            claw.setPosition(0.7);
            sleep(200);
            claw.setPosition(0.6);
            sleep(200);
            claw.setPosition(0.5);
            sleep(200);
            claw.setPosition(0.4);
            sleep(200);
            claw.setPosition(0.3);
            sleep(200);
            claw.setPosition(0.2);
            sleep(200);
            claw.setPosition(0);
            sleep(1400);

        }
    }
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        rightShoulder = hardwareMap.get(CRServo.class, "rightShoulder");
        leftShoulder = hardwareMap.get(CRServo.class, "leftShoulder");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");



        imu_IMU = hardwareMap.get(IMU.class, "imu");


        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        // Prompt user to press start button.

        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        rightShoulder.setDirection(CRServo.Direction.REVERSE);
        claw.setPosition(1);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            claw.setPosition(0);
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

