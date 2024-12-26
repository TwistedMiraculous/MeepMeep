package org.firstinspires.ftc.teamcode.Drive.Pos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Define initial pose
        Pose2d beginPose = new Pose2d(61.7, 36.7, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        // Check if stop is requested
        if (isStopRequested()) return;



        // Rotate to 45 degrees
        Actions.runBlocking(
                drive.actionBuilder(beginPose).turn(Math.toRadians(45)).waitSeconds(3).build());
        beginPose = new Pose2d(61.7, 36.7, Math.toRadians(45));



        Actions.runBlocking(
                // Rotate to 45 degrees

                drive.actionBuilder(beginPose).turn(Math.toRadians(-45)).waitSeconds(3).build());
        beginPose = new Pose2d(61.7, 36.7, Math.toRadians(0));
    }
}

