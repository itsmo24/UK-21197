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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "(004) AutonomousWithTime")

public class AutonomousWithTime1 extends LinearOpMode{
    IMU imu;
    DcMotor back_left_motor;
    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor back_right_motor;
    YawPitchRollAngles orientation;
    CRServo rightShoulder;
    CRServo leftShoulder;
    CRServo wrist;
    Servo claw;
    DistanceSensor test_distance;

    double inches = 24;
    public void forward(double distance, double power ){

        back_left_motor.setPower(power);
        back_right_motor.setPower(power);
        front_left_motor.setPower(power);
        front_right_motor.setPower(power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void backward(double distance, double power ){

        back_left_motor.setPower(-power);
        back_right_motor.setPower(-power);
        front_left_motor.setPower(-power);
        front_right_motor.setPower(-power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void right(double distance, double power ){
        back_left_motor.setPower(-power);
        back_right_motor.setPower(power);
        front_left_motor.setPower(power);
        front_right_motor.setPower(-power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void left(double distance, double power ){
        back_left_motor.setPower(power);
        back_right_motor.setPower(-power);
        front_left_motor.setPower(-power);
        front_right_motor.setPower(power);
        sleep((long) (1000 * (distance/inches)));
        stopMotors();
    }
    public void stopMotors() {
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
    }
    double currentAngle;
    public void rotateCCW(int targetOrientationAngle, float power) {
        currentAngle = 0;
        imu.resetYaw();
        back_left_motor.setPower(-power);
        back_right_motor.setPower(power);
        front_left_motor.setPower(-power);
        front_right_motor.setPower(power);
        while (currentAngle < targetOrientationAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    public void rotateCW(int targetOrientationAngle,float power) {
        currentAngle = 0;

        imu.resetYaw();
        back_left_motor.setPower(power);
        back_right_motor.setPower(-power);
        front_left_motor.setPower(power);
        front_right_motor.setPower(-power);
        while (-currentAngle < targetOrientationAngle) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        }
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        sleep(250);
    }
    public void sensor(int targetDistance,double power){

        int currentDistance = (int) Math.round(test_distance.getDistance(DistanceUnit.CM));// If the distance in centimeters is less than 10, set the power to 0.3


        if (targetDistance < currentDistance){
            back_left_motor.setPower(power);
            back_right_motor.setPower(power);
            front_left_motor.setPower(power);
            front_right_motor.setPower(power);
            currentDistance = (int) Math.round(test_distance.getDistance(DistanceUnit.CM));
        }

        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);

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
            sleep(500);
        }
        else{
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
        back_right_motor = hardwareMap.get(DcMotor.class, "backRight");
        back_left_motor = hardwareMap.get(DcMotor.class, "backLeft");
        front_right_motor = hardwareMap.get(DcMotor.class, "frontRight");
        front_left_motor = hardwareMap.get(DcMotor.class, "frontLeft");
        rightShoulder = hardwareMap.get(CRServo.class, "rightArm");
        leftShoulder = hardwareMap.get(CRServo.class, "leftArm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "gripper");
        test_distance = hardwareMap.get(DistanceSensor.class, "test_distance");




        imu = hardwareMap.get(IMU.class, "imu");


        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
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


            sensor(30,1);
            sensor(1,0.2);
            rightShoulder.setPower(-0.6);
            leftShoulder.setPower(-0.6);
            sleep(1600);
            rightShoulder.setPower(0);
            leftShoulder.setPower(0);
            wrist.setPower(-0.4);
            sleep(1600);
            wrist.setPower(0);
            rightShoulder.setPower(0.6);
            leftShoulder.setPower(0.6);
            sleep(1100);
            rightShoulder.setPower(0);
            leftShoulder.setPower(0);



            grabber( false);
            rightShoulder.setPower(0.6);
            leftShoulder.setPower(0.6);
            sleep(500);
            wrist.setPower(0.4);
            sleep(1600);
            wrist.setPower(0);





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



