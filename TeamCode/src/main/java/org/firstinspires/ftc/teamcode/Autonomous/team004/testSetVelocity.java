package org.firstinspires.ftc.teamcode.Autonomous.team004;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "testSetVelocity")
public class testSetVelocity extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        double currentVelocity;

        double maxVelocity = 0.0;
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        // Start
        while (opModeIsActive()) {
            currentVelocity = frontLeft.getVelocity();

            frontLeft.setVelocity(3160);

            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }




            telemetry.addData("current velocity", currentVelocity);

            telemetry.addData("maximum velocity", maxVelocity);

            telemetry.update();
        }
    }
}
