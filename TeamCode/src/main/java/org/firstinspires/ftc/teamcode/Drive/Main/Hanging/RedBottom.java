package org.firstinspires.ftc.teamcode.Drive.Main.Hanging;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RedBottom extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Define initial pose
        Pose2d beginPose = new Pose2d(-36, -60, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        // Check if stop is requested
        if (isStopRequested()) return;

        // First movement to position 1
        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(-36, -60)).waitSeconds(2).build());

        // Update pose to the new position
        beginPose = new Pose2d(-36, -60, Math.toRadians(0));

        // Second movement to position 2
        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(-36, -12)).waitSeconds(2).build());

        // Update pose to the new position
        beginPose = new Pose2d(-36, -12, Math.toRadians(0));

    }
}
