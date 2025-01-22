package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class MainMethods{
    IMU imu;
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor frontLeft;
    CRServo rightArm;
    CRServo leftArm;
    CRServo wrist;
    Servo gripper;
    DistanceSensor rangeSensor;

    int armUpTime = 1600;
    int armDownTime = 1600;
    int wristUpTime = 1400;
    int wristDownTime = 1400;
    
    int pauseTimer = 100;

    double gripperClosedPosition = 1.0;
    double gripperOpenPosition = 0;

    public MainMethods(HardwareMap hardwareMap) {
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    // GRIPPER
    public void gripperClose() {
        gripper.setPosition(gripperClosedPosition);
        sleep(pauseTimer);
    }
    public void gripperOpen() {
        gripper.setPosition(gripperOpenPosition);
        sleep(pauseTimer);
    }

    // ARM AND WRIST
    public void armUp(){
        rightArm.setPower(-0.8);
        leftArm.setPower(-0.8);
        sleep(armUpTime);
        rightArm.setPower(0);
        leftArm.setPower(0);
        sleep(pauseTimer);
    }
    public void armDown(){
        rightArm.setPower(0.8);
        leftArm.setPower(0.8);
        sleep(armDownTime);
        rightArm.setPower(0);
        leftArm.setPower(0);
        sleep(pauseTimer);
    }
    public void wristDown() {
        wrist.setPower(0.4);
        sleep(wristDownTime);
        wrist.setPower(0);
        sleep(pauseTimer);
    }
    public void wristUp() {
        wrist.setPower(-0.5);
        sleep(wristUpTime);
        wrist.setPower(0);
        sleep(pauseTimer);
    }

    // MOVEMENT
    public void movement(int time, double power){
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        sleep(time);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    
    // SIDEWAYS
    public void sideways(int time, double power){
        // Right positive power
        backLeft.setPower(-power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        sleep(time);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }



    // Function that finds difference of two inputs divides by the addition of them

    public double decimal(double num1, double num2){
        num1 = Math.abs(num1);
        num2 = Math.abs(num2);
        if (num1 > num2){
            return  Math.abs(num1 - num2) / num1;
        }
        else {
            return  Math.abs(num1 - num2) / num2;
        }
    }

    public void turn(int targetAngle){
        double power = 1;
        int currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
        // Looping until target angle is reached
        while (currentAngle != targetAngle){
            // Checking to see if needed to turn right or left
            if (currentAngle > targetAngle){
                power = -decimal(currentAngle, targetAngle);
            } else {
                power = decimal(currentAngle, targetAngle);
            }

            // Turning right if power is positive
            backLeft.setPower(power);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
        }
        sleep(pauseTimer);
    }

    public void range(int targetDistance){
        double power = 1;
        int currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));

        while (targetDistance != currentDistance){
            if (targetDistance > currentDistance){
                power = -decimal(currentDistance, targetDistance);
            } else {
                power = decimal(currentDistance, targetDistance);
            }
            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));
        }
        sleep(pauseTimer);
    }

}
