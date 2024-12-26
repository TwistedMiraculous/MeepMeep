package org.firstinspires.ftc.teamcode.Drive.Main.Hanging;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class BlueTop extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Define initial pose
        Pose2d beginPose = new Pose2d(61.7,36.7, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        // Check if stop is requested
        if (isStopRequested()) return;


        //  position for 1st sample
        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(61.7,48.7)).waitSeconds(3).build());

        // Update pose to the new position
        beginPose = new Pose2d(61.7,48.7, Math.toRadians(0));

        //  position for 2nd sample
        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(61.7,61.7)).waitSeconds(3).build());

        // Update pose to the new position
        beginPose = new Pose2d(61.7,61.7, Math.toRadians(0));



        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(64.7,35.7)).waitSeconds(3).build());

        // Update pose to the new position
        beginPose = new Pose2d(64.7,35.7 , Math.toRadians(0));


        //  position for 3rd sample
        // Rotate to 45 degrees
        Actions.runBlocking(
                drive.actionBuilder(beginPose).turn(Math.toRadians(45)).waitSeconds(3).build());
        beginPose = new Pose2d(64.7, 39.7, Math.toRadians(45));



        Actions.runBlocking(
                // Rotate to 45 degrees

                drive.actionBuilder(beginPose).turn(Math.toRadians(-45)).waitSeconds(3).build());
        beginPose = new Pose2d(64.7, 39.7, Math.toRadians(0));
    }
}

