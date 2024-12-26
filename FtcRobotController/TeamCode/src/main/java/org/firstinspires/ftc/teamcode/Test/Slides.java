package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
 * This OpMode demonstrates how to use a digital channel (touch sensor)
 * to control motor position with encoders.
 */
@TeleOp(name = "slides", group = "Sensor")
public class Slides extends LinearOpMode {
    DigitalChannel digitalTouch;  // Digital channel Object (Touch Sensor)
    DcMotor slide1, slide2;

    // Encoder constants
    private static final double TICKS_PER_REV = 2786.2;  // Encoder pulses per revolution
    private static final double RPM = 60;               // Motor RPM
    private static final double TICKS_PER_SECOND = TICKS_PER_REV / 60;  // Ticks per second

    @Override
    public void runOpMode() {

        // Initialize hardware
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        slide1 = hardwareMap.get(DcMotor.class, "slideup1");
        slide2 = hardwareMap.get(DcMotor.class, "slideup2");

            // motor config
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run motor with encoder feedback
        slide1.setDirection(DcMotor.Direction.FORWARD);    // Set motor direction
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder

        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run motor with encoder feedback
        slide2.setDirection(DcMotor.Direction.REVERSE);    // Set motor direction
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);  // Set touch sensor as input

        telemetry.addData("Slides", "Press start to continue...");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop while OpMode is active
        while (opModeIsActive()) {
            // Touch Sensor Logic
            if (!digitalTouch.getState()) {
                telemetry.addData("Button", "PRESSED");
                
                slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                moveMotorToTarget(30);
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }

            // Gamepad Control Logic
            if (gamepad1.a) {
                slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide1.setPower(0.5);
                slide2.setPower(0.5);
            } else if (gamepad1.b) {
                slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide1.setPower(-0.5);
                slide2.setPower(-0.5);
            } else {
                slide1.setPower(0);
                slide2.setPower(0);
            }

            // Telemetry
            int motorPosition1 = slide1.getCurrentPosition();
            int motorPosition2 = slide2.getCurrentPosition();
            telemetry.addData("Motor Position Slide1", motorPosition1);
            telemetry.addData("Motor Position Slide2", motorPosition2);
            telemetry.addData("Gamepad A Pressed", gamepad1.a);
            telemetry.addData("Gamepad B Pressed", gamepad1.b);
            telemetry.update();
        }
    }

    // Method to move the motor to a specific target position
    private void moveMotorToTarget(int targetPosition) {
        // Set the target position
        slide1.setTargetPosition(targetPosition);
        slide2.setTargetPosition(targetPosition);
        // Set motor mode to RUN_TO_POSITION after target position is set
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set motor power (50%)
        slide1.setPower(0.5);
        slide2.setPower(0.5);

        // Wait until the motor reaches the target position
        while (opModeIsActive() && slide1.isBusy()) {
            // Update telemetry with current position
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Motor Power", slide1.getPower());
            telemetry.addData("Motor Busy", slide1.isBusy());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Motor Power", slide2.getPower());
            telemetry.addData("Motor Busy", slide2.isBusy());
            telemetry.update();

        }

        // Stop the motor once the target position is reached
        slide1.setPower(0);
    }
}
