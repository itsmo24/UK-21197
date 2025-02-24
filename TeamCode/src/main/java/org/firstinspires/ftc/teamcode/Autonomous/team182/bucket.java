package org.firstinspires.ftc.teamcode.Autonomous.team182;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Autonomous.mainMethods;
@Autonomous(name = "(182) bucket")
public class bucket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
        leftArm = hardwareMap.get(DcMotor.class, "rightArm");
        imu = hardwareMap.get(IMU.class, "imu");

        int armUpPosition = 350;

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
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



    }
}
