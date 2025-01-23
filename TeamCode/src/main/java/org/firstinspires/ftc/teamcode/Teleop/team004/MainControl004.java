package org.firstinspires.ftc.teamcode.Teleop.team004;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main Controls")

public class MainControl004 extends OpMode
{
    // Declare OpMode members.
    final ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    CRServo rightArm;
    CRServo leftArm;
    CRServo wrist;
    Servo gripper;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        rightArm = hardwareMap.get(CRServo.class, "rightArm");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        gripper = hardwareMap.get(Servo.class, "gripper");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(CRServo.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double topLeftPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double bottomLeftPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double topRightPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double bottomRightPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

        // Movement
        frontLeft.setPower(topLeftPower);
        backLeft.setPower(bottomLeftPower);
        frontRight.setPower(topRightPower);
        backRight.setPower(bottomRightPower);

        //arm and wrist
        wrist.setPower(gamepad2.right_stick_y);
        if (gamepad2.right_bumper) {
            gripper.setPosition(1);
        } else{
            gripper.setPosition(0);
        }

        //Arm
        leftArm.setPower(gamepad2.left_stick_y);
        rightArm.setPower(gamepad2.left_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


