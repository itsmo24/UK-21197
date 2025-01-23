package org.firstinspires.ftc.teamcode.Autonomous.team004;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous

public class AutonomousWithTime extends LinearOpMode{
    private IMU imu_IMU;
    private DcMotor back_left_motor;
    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_right_motor;
    private YawPitchRollAngles orientation;
    private CRServo rightShoulder;
    private CRServo leftShoulder;
    private CRServo wrist;
    private Servo claw;
    double inches = 24;
    private void forward(double distance, double power ){

        back_left_motor.setPower(power);
        back_right_motor.setPower(power);
        front_left_motor.setPower(power);
        front_right_motor.setPower(power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    private void backward(double distance, double power ){

        back_left_motor.setPower(-power);
        back_right_motor.setPower(-power);
        front_left_motor.setPower(-power);
        front_right_motor.setPower(-power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    private void right(double distance, double power ){
        back_left_motor.setPower(-power);
        back_right_motor.setPower(power);
        front_left_motor.setPower(power);
        front_right_motor.setPower(-power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    private void left(double distance, double power ){
        back_left_motor.setPower(power);
        back_right_motor.setPower(-power);
        front_left_motor.setPower(-power);
        front_right_motor.setPower(power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    private void stopMotors() {
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
    }
    double currentAngle;
    private void rotateCCW(int targetOrientationAngle, float power) {
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
    private void rotateCW(int targetOrientationAngle,float power) {
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
    private void grabber(boolean clawCheck){
        if (clawCheck == true){
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
            sleep(500);
        }
        else if (clawCheck == false){
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

            forward(40,1);
            //left(12, 1);
            rightShoulder.setPower(-0.6);
            leftShoulder.setPower(-0.6);
            sleep(1600);
            rightShoulder.setPower(0);
            leftShoulder.setPower(0);
            wrist.setPower(-0.4);
            sleep(1600);
            wrist.setPower(0);


            grabber( false);


      /*sleep(100);
      backward(4,1);
      sleep(250);
      grabber(true);
      wrist.setPower(0.6);
      sleep(1600);
      wrist.setPower(0);

      right(12,0.5);
      forward(24,1);

      rotateCW(90,0.8f);

      forward(72,1);


      right(24,1);
      rotateCCW(90,0.8f);
      rotateCCW(90,0.8f);
      backward(6,1);


      rightShoulder.setPower(-0.6);
      leftShoulder.setPower(-0.6);
      sleep(1600);
      rightShoulder.setPower(0.1);
      leftShoulder.setPower(0.1);


      sleep(200);
      claw.setPosition(0);
      sleep(700);

      rightShoulder.setPower(0.5);
      leftShoulder.setPower(0.5);
      sleep(1800);
      rightShoulder.setPower(0);
      leftShoulder.setPower(0);

      left(0,1);
      backward(0,1);
    */

        }
    }



    // todo: write your code here
}



