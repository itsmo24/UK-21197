package org.firstinspires.ftc.teamcode.Autonomous.team004;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "(004) Autonomousteamside")

public class AutonomousWithTime1 extends LinearOpMode{
    mainMethods move;
    IMU imu;
    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    YawPitchRollAngles orientation;
    CRServo rightArm;
    CRServo leftArm;
    CRServo wrist;
    Servo gripper;
    DistanceSensor rangeSensor;

    double inches = 24;
    public void forward(double distance, double power ){

        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void backward(double distance, double power ){

        backLeft.setPower(-power);
        backRight.setPower(-power);
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void right(double distance, double power ){
        backLeft.setPower(-power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void left(double distance, double power ){
        backLeft.setPower(power);
        backRight.setPower(-power);
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void stopMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
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
    public void rotateCW(int targetOrientationAngle,float power) {
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
    public void sensor(int targetDistance,double power){

        int currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));// If the distance in centimeters is less than 10, set the power to 0.3


        if (targetDistance < currentDistance){
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));
        }

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        }


    public void grabber(boolean clawCheck){
        if (clawCheck){
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
            sleep(500);
        }
        else{
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
        move = new mainMethods(hardwareMap);
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");




        imu = hardwareMap.get(IMU.class, "imu");
        int armUpPosition = 430;


        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        // Prompt user to press start button.

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(CRServo.Direction.REVERSE);
        gripper.setPosition(1);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        waitForStart();




        if (opModeIsActive()) {



            move.range(20);
            move.arm(armUpPosition);
            telemetry.addData("Encoder Values", "FL: %d");
            wrist.setPower(-0.4);
            sleep(1600);
            wrist.setPower(0);
            move.arm(-200);



            grabber( false);
            move.arm(300);
            wrist.setPower(0.4);
            sleep(1600);
            wrist.setPower(0);





      /*sleep(100);

      backward(4,1);
      sleep(250);
      gripper(true);
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



