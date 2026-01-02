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

import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Disabled
@Config
@Autonomous
public class RedGoalAuto3 extends LinearOpMode{

    private static final Logger log = LoggerFactory.getLogger(RedGoalAuto3.class);

    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-55, 52, Math.toRadians(-57));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        Action backFromGoal = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-28, 25,Math.toRadians(-52)), Math.toRadians(-52))
                .build();

        Action to1stLine = drive.actionBuilder(new Pose2d (-26, 23, Math.toRadians(-52)))
                .splineToSplineHeading(new Pose2d(-16,28, Math.toRadians(85)),Math.PI/2)
                .build();

        Action pickup1stLine = drive.actionBuilder(new Pose2d(-16,28, Math.toRadians(85)))
                .lineToY(47, new TranslationalVelConstraint(4.1)) // forward into 1st line of balls
                .build();

        Action toGoalWFirstLine = drive.actionBuilder(new Pose2d(-16, 47, Math.toRadians(85)))
                .setReversed(true)
                //.splineToLinearHeading(new Pose2d(-20, 23, Math.toRadians(30)), Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(-28,25,Math.toRadians(-52)), 26*Math.PI/35)
                .build();

        Action to2ndLine = drive.actionBuilder(new Pose2d (-30, 27, Math.toRadians(-52)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(6.5,28, Math.toRadians(85)),Math.PI/2)
                .build();

        Action pickup2ndLine = drive.actionBuilder(new Pose2d(6.5,28, Math.toRadians(85)))
                .lineToY(47, new TranslationalVelConstraint(4.1)) // forward into 1st line of balls
                .build();


        Action toGoalWSecondLine = drive.actionBuilder(new Pose2d(6.5, 47, Math.toRadians(85)))
                .setReversed(true)
                //.splineTo(new Vector2d(0, 23), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-28,25,Math.toRadians(-52)), 26*Math.PI/35)
                .build();
        Action moveFromLine = drive.actionBuilder(new Pose2d(-30, 27, Math.toRadians(-52)))
                .splineToConstantHeading(new Vector2d(-16,45), Math.toRadians(45))
                .build();


        if (isStopRequested()) return;
        trigger.triggerDown();
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                                backFromGoal,
                                flywheels.wheelsOn(1),
                                new SleepAction(0.2),
                                transfer.transferOn(),
                                new SleepAction(0.2),
                                intake.intakeOn()
                        ),
                        new SleepAction(2.5),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        to1stLine,
                        intake.intakeOn(),              //intake 1st line
                        transfer.transferOn(),
                        pickup1stLine,
                        new SleepAction(0.15),
                        transfer.transferOff(),
                        new SleepAction(0.25),
                        //intake.intakeOff(),
                        new ParallelAction(                 //launch 1st line
                                toGoalWFirstLine,
                                flywheels.wheelsOn(1),
                                new SleepAction(0.5),
                                transfer.transferOn()
                        ),
                        new SleepAction(2.5),
                        trigger.triggerUp(),
                        new SleepAction(1),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        to2ndLine,
                        intake.intakeOn(),              //intake 2nd line
                        transfer.transferOn(),
                        pickup2ndLine,
                        new SleepAction(0.15),
                        transfer.transferOff(),
                        new SleepAction(0.25),
                        new ParallelAction(                 //launch 2nd line
                                toGoalWSecondLine,
                                flywheels.wheelsOn(1),
                                new SleepAction(2.8),
                                transfer.transferOn()
                        ),
                        new SleepAction(1),
                        moveFromLine

                        /*trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown()

                         */

                )
        );

    }
}
