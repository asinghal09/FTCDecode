package org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@Config
@Autonomous
public class LeaveAuto extends LinearOpMode{

    private static final Logger log = LoggerFactory.getLogger(LeaveAuto.class);

    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(54, 18, Math.toRadians(-4));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        Action leave = drive.actionBuilder(initialPose)
                .lineToX(20)
                .build();

        if (isStopRequested()) return;
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                    leave
                )
        );

    }
}
