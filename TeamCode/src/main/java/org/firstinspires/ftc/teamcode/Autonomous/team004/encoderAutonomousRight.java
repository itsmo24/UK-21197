package org.firstinspires.ftc.teamcode.Autonomous.team004;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "encoderAutonomousRight")
public class encoderAutonomousRight extends LinearOpMode {

    // Declare Motors and Servos
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private CRServo leftArm, rightArm, wrist;
    private Servo gripper;
    private double circumference = 2.95 * Math.PI;

    @Override
    public void runOpMode() {
        // Initialize
        ElapsedTime runtime = new ElapsedTime();

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        leftArm = hardwareMap.get(CRServo.class, "leftArm");
        rightArm= hardwareMap.get(CRServo.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(CRServo.class, "wrist");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setDirection(CRServo.Direction.REVERSE);

        // Reset Encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motors to Run Using Encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Values
        frontLeft.setVelocityPIDFCoefficients(1.15, 0.115, 0, 11.5);
        frontRight.setVelocityPIDFCoefficients(3.45, 0.345, 0, 34.5);
        backLeft.setVelocityPIDFCoefficients(1.2, 0.12, 0, 12);
        backRight.setVelocityPIDFCoefficients(1.13, 0.113, 0, 11.3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start autonomous movement
        if (opModeIsActive()) {
            forward(12, 500);  // Move forward 12 inches at 500 ticks/sec
        }
    }

    public void stopMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public void forward(double distance, double velocity) {
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);

        double time = ((distance * 1120) / circumference) / velocity;
        telemetry.addData("Calculated time", time);
        telemetry.update();

        sleep((long) (time * 1000));
        stopMotors();
    }


}