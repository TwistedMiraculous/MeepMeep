package org.firstinspires.ftc.teamcode.Drive.Pos;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous
public class MidtoBR extends LinearOpMode {

    @Override

    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        ElapsedTime time = new ElapsedTime();

        while(true){
            if (isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose).strafeTo(new Vector2d(-36.3, -61.7)).waitSeconds(2).build());  //(36.3, 61.7))
            if (time.time() > 1) {
                time.reset();

                beginPose = new Pose2d(-36.3, -61.7, Math.toRadians(0));  //(36.3, 61.7

                Actions.runBlocking(
                        drive.actionBuilder(beginPose).strafeTo(new Vector2d(0, 0)).waitSeconds(2).build());
                beginPose = new Pose2d(0, 0, Math.toRadians(0)); //(11.8, 61.7, Math.toRadians(0))
            }
        }
    }
}