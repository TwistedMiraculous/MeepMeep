package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class StNetPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Define initial pose
        Pose2d beginPose = new Pose2d(56.3,61.7, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        // Check if stop is requested
        if (isStopRequested()) return;

//top blue to observation
        //  position for 1st sample
        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(56.3, 61.7)).waitSeconds(2).build());

        // Update pose to the new position
        beginPose = new Pose2d(56.3, 61.7, Math.toRadians(0));

        //  position for 2nd sample
        Actions.runBlocking(
                drive.actionBuilder(beginPose).strafeTo(new Vector2d(-56.3, 61.7)).waitSeconds(2).build());  //(36.3, 61.7))
        beginPose = new Pose2d(-56.3, 61.7, Math.toRadians(0)); //(11.8, 61.7, Math.toRadians(0))

    }
}
