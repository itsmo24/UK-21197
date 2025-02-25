package org.firstinspires.ftc.teamcode.Autonomous.team182;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;

@Autonomous(name = "(182) specimen")
public class specimen extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        mainMethods move;
        IMU imu;
        DcMotor backRight;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor frontLeft;
        DcMotor rightArm;
        DcMotor leftArm;

        move = new mainMethods(hardwareMap);
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        imu = hardwareMap.get(IMU.class, "imu");

        int armUpPosition = 430;
        int wristUpTime = 1250;
        int wristDownTime = 1200;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu.resetYaw();

        move.gripperClose();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        // Go and hang specimen
        move.movement(500, 1);
        move.sideways(500, -1);
        move.turn(0, 0.3);
        move.arm(armUpPosition, 1);
        move.range(20, 0.5);
        move.wristDown(wristDownTime);
        move.arm(armUpPosition/2, 1);
        move.gripperOpen();
        move.wristUp(wristUpTime);
        move.arm(0, 1);
        // Go and grab another specimen
        move.movement(500, -1);
        move.turn(90, 0.7);
        move.range(40, 0.8);
        move.turn(180, 0.7);
        move.wristDown(600);
        move.range(20, 0.5);
        sleep(1000);
        move.gripperClose();
        // Go and hang other specimen
        move.movement(500, -1);
        move.turn(0, 0.7);
        move.sideways(1000, -1);
        move.range(20, 0.4);
        move.wristDown(wristDownTime);
        move.arm(armUpPosition/2, 1);
        move.gripperOpen();
        move.wristUp(wristUpTime);
        move.arm(0, 1);
        // Go and park
        move.range(70, 0.8);
        move.sideways(1000, 1);
    }
}
