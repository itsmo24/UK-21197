package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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


    int countsPerRevolution = 288;
    int armUpTime = 1600;
    int armDownTime = 1600;
    int wristUpTime = 1400;
    int wristDownTime = 1400;
    
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
        sleep(pauseTimer);
    }
    public void gripperOpen() {
        gripper.setPosition(gripperOpenPosition);
        sleep(pauseTimer);
    }

    // ARM AND WRIST
    public void armUpT(){
        rightArm.setPower(-0.8);
        leftArm.setPower(-0.8);
        sleep(armUpTime);
        rightArm.setPower(0);
        leftArm.setPower(0);
        sleep(pauseTimer);
    }
    public void armDownT(){
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


    public void arm(int targetPosition, double pPower){
        double power;
        int currentPosition = rightArm.getCurrentPosition();
        while (targetPosition != currentPosition){
            if (targetPosition > currentPosition){
                power = -pPower;
            } else{
                power = pPower;
            }
            rightArm.setPower(power);
            leftArm.setPower(power);
            currentPosition = rightArm.getCurrentPosition();
        }
        leftArm.setPower(0);
        rightArm.setPower(0);

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
        double power;
        num1 = Math.abs(num1);
        if (num2 == 0){
            return 0;
        }
        power =  (num1 / num2);
        if (power < 0.2 & power > 0) {
            power = 0.2;
        } else if (power > 0.2) {
            power = 1;
        } else {
            power = 0;
        }
        return power;
    }

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
//        sleep(pauseTimer);
//    }

    public void turn(int targetAngle){
        double power;
        int currentAngle = (int) Math.round(imu.getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES.toAngleUnit()));
        // Looping until target angle is reached
        while (currentAngle != targetAngle){
            // Checking to see if needed to turn right or left
            if (currentAngle > targetAngle){
                power = -decimal(currentAngle, Math.abs(Math.abs(currentAngle) - Math.abs(targetAngle)));
            } else {
                power = decimal(currentAngle, Math.abs(Math.abs(currentAngle) - Math.abs(targetAngle)));;
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
        double power;
        int currentDistance = (int) Math.round(rangeSensor.getDistance(DistanceUnit.CM));
        double distanceToTravel = Math.abs(currentDistance-targetDistance);

        while (targetDistance != currentDistance){
            double distanceToGo = Math.abs(currentDistance-targetDistance);
            if (targetDistance > currentDistance){
                power = -decimal(distanceToGo, distanceToTravel);
            } else {
                power = decimal(distanceToGo, distanceToTravel);
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
