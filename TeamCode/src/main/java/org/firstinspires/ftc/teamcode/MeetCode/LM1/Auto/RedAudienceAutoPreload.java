package org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Disabled
@Config
@Autonomous
public class RedAudienceAutoPreload extends LinearOpMode{

    private static final Logger log = LoggerFactory.getLogger(RedAudienceAutoPreload.class);

    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(54, 18, Math.toRadians(-4));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        Action backFromGoal = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28, 25,Math.toRadians(-52)), Math.toRadians(128))
                .build();

        Action back = drive.actionBuilder(new Pose2d (-26, 23, Math.toRadians(-52)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(0,24),0)
                .build();


        if (isStopRequested()) return;
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                                backFromGoal,
                                flywheels.wheelsOn(1)
                        ),
                        new SleepAction(1),
                        transfer.transferOn(),
                        new SleepAction(0.2),
                        intake.intakeOn(),
                        new SleepAction(2.5),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        back

                )
        );

    }
}
