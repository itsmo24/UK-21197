package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp

public class MaxVelocityTest extends LinearOpMode {

    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    IMU imu;

    double currentVelocity;

    double maxVelocity = 0.0;


    @Override

    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        waitForStart();


        while (opModeIsActive()) {




            currentVelocity = backLeft.getVelocity();
            backLeft.setPower(1);


            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }




            telemetry.addData("current velocity", currentVelocity);

            telemetry.addData("maximum velocity", maxVelocity);

            telemetry.update();
            //004
            //front left max velocity is 2860
            //back left max velocity is 2720
            //front right max velocity is 980
            //back right max velocity is 2900
            //182


        }

    }

}