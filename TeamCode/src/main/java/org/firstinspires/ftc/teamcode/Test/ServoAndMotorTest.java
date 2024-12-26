package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class ServoAndMotorTest extends LinearOpMode {
    private DcMotor motor;
    private Servo servo;
    private boolean isMotorRunning = false;
    private boolean isServoExtended = false;
    private boolean isServoMode = false; // Toggle between motor and servo control

    @Override
    public void runOpMode() {
        // Initialize the motor and servo
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");

        // Reset and initialize the motor encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the servo to its initial position
        servo.setPosition(0.0);

        // Wait for the start button to be pressed
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Toggle between motor and servo mode when D-Pad up is pressed
            if (gamepad1.dpad_up) {
                isServoMode = !isServoMode; // Toggle the control mode
                sleep(200); // Small delay to debounce button presses
            }

            // Set isServoMode to true when D-Pad down is pressed
            if (gamepad1.dpad_down) {
                isServoMode = true; // Directly set to Servo mode
                sleep(200); // Small delay to debounce button presses
            }



            if (isServoMode) {
                // Servo control: toggle position with 'B'
                if (gamepad1.b) {
                    servo.setPosition(2);
                    } else if (gamepad1.a) {
                    servo.setPosition(0);
                }
                sleep(200); // Prevents button debounce
                } else if (!isServoMode) {

                if (gamepad1.b) {
                    motor.setPower(0.5);
                } else if (gamepad1.a) {
                    motor.setPower(0);
                }
                sleep(200); // Prevents button debounce
            }

            // Fetch the current position of the motor and servo
            int motorPosition = motor.getCurrentPosition();
            double servoPosition = servo.getPosition();

            // Send telemetry data
            telemetry.addData("Control Mode", isServoMode ? "Servo Mode" : "Motor Mode");
            telemetry.addData("Motor Position", motorPosition);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
