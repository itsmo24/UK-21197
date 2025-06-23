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


public class mainMethods {
    IMU imu;
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor rightArm;
    DcMotor leftArm;
    CRServo wrist;
    Servo gripper;
    DistanceSensor rangeSensor;

    int armUpTime = 1500;
    int armDownTime = 1350;
    int wristUpTime = 1250;
    int wristDownTime = 1200;

    int pauseTimer = 100;

    double gripperClosedPosition = 1.0;
    double gripperOpenPosition = 0;

    public mainMethods(HardwareMap hardwareMap) {
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    // GRIPPER
    public void gripperClose() {
        gripper.setPosition(gripperClosedPosition);

    }

    public void gripperOpen() {
        gripper.setPosition(gripperOpenPosition);

    }

    // ARM AND WRIST
    public void armT(int time, double power) {
        rightArm.setPower(power);
        leftArm.setPower(power);
        sleep(time);
        rightArm.setPower(0);
        leftArm.setPower(0);

    }

    public void wristUp(int time) {
        wrist.setPower(1);
        sleep(time);
        wrist.setPower(0);
    }

    public void wristDown(int time) {
        wrist.setPower(-1);
        sleep(time);
        wrist.setPower(0);
    }


    public void arm(int targetPosition, double pPower) {

        int currentPosition = rightArm.getCurrentPosition();
        double power = (currentPosition > targetPosition) ? pPower : -pPower;
        while (Math.abs(currentPosition - targetPosition) > 4) {
            rightArm.setPower(power);
            leftArm.setPower(power);
            currentPosition = rightArm.getCurrentPosition();
        }
        rightArm.setPower(0);
        leftArm.setPower(0);
    }

    // MOVEMENT

    public void movement(int time, double power) {
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
    public void sideways(int time, double power) {
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

//    public double decimal(double num1, double num2){
//        double power;
//        num1 = Math.abs(num1);
//        if (num2 == 0){
//            return 0;
//        }
//        power =  (num1 / num2);
//        if (power < 0.2 & power > 0) {
//            power = 0.2;
//        } else if (power > 0.2) {
//            power = 1;
//        } else {
//            power = 0;
//        }
//        return power;
//    }

//    public void turn(int targetAngle){
//        double power;
//        int currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
//        // Looping until target angle is reached
//        int angleToTurn = Math.abs(currentAngle-targetAngle);
//        while (currentAngle != targetAngle){
//            int amountToTurn = Math.abs(currentAngle-targetAngle);
//            // Checking to see if needed to turn right or left
//            if (currentAngle > targetAngle){
//                power = -decimal(amountToTurn, angleToTurn);
//            } else {
//                power = decimal(amountToTurn, angleToTurn);
//            }
//
//            // Turning right if power is positive
//            backLeft.setPower(power);
//            backRight.setPower(-power);
//            frontLeft.setPower(power);
//            frontRight.setPower(-power);
//            currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
//        }
//        
//    }

    public void turn(int targetAngle, double pPower) {
        int currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
        // Looping until target angle is reached

        while (Math.abs(currentAngle - targetAngle) > 2) {
            double power = (currentAngle < targetAngle) ? pPower : -pPower;
            backLeft.setPower(power);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void range(int targetDistance, double pPower) {
        int currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));
        double power = (currentDistance > targetDistance) ? pPower : -pPower;

        while (Math.abs(currentDistance - targetDistance) > 2) {
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
}