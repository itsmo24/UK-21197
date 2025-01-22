package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Straight then left")
public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode(){
        MainMethods move;
        IMU imu;
        DcMotor backRight;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor frontLeft;
        CRServo rightArm;

        move = new MainMethods(hardwareMap);
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        imu = hardwareMap.get(IMU.class, "imu");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.resetYaw();

        move.gripperClose();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        move.movement(500, 1);
        move.sideways(1000, -1);
        move.turn(0);
        move.range(5);
        move.armUp();
        move.wristUp();
        move.armDown();
        move.movement(500, 1);
        move.sideways(1000, 1);
        move.turn(90);
        move.range(30);
        move.turn(180);
        move.range(30);
        move.turn(0);
    }
}
