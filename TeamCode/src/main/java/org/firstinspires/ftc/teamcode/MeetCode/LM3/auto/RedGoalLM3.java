package org.firstinspires.ftc.teamcode.MeetCode.LM3.auto;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;


@Config
@Autonomous
public class RedGoalLM3 extends LinearOpMode{
    private VoltageSensor batteryVoltageSensor;


    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-55, 52, Math.toRadians(-52));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double targetVoltage = 12.3;
        double correction = targetVoltage / batteryVoltageSensor.getVoltage();


        Action backFromGoal = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-28, 25,Math.toRadians(-52)), Math.toRadians(-52))
                .build();

        //1st line paths
        Action to1stLine = drive.actionBuilder(new Pose2d (-26, 23, Math.toRadians(-52)))
                .splineToSplineHeading(new Pose2d(-16,28, Math.toRadians(85)),Math.PI/2)
                .build();

        Action pickup1stLine = drive.actionBuilder(new Pose2d(-16,28, Math.toRadians(85)))
                .lineToY(48, new TranslationalVelConstraint(10))  // forward into 1st line of balls
                .build();

        Action toGoalWFirstLine = drive.actionBuilder(new Pose2d(-16, 48, Math.toRadians(85)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28,25,Math.toRadians(-52)), 26*Math.PI/35)
                .build();

        //2nd line path
        Action to2ndLine = drive.actionBuilder(new Pose2d (-28, 25, Math.toRadians(-52)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(6.5,28, Math.toRadians(85)),Math.PI/2)
                .build();

        Action pickup2ndLine = drive.actionBuilder(new Pose2d(6.5,28, Math.toRadians(85)))
                .lineToY(48, new TranslationalVelConstraint(10)) // forward into 2nd line of balls
                .build();


        Action toGoalWSecondLine = drive.actionBuilder(new Pose2d(6.5, 48, Math.toRadians(85)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28,25,Math.toRadians(-52)), 26*Math.PI/35)
                .build();


        //3rd line path
        Action to3rdLine = drive.actionBuilder(new Pose2d (-28, 25, Math.toRadians(-52)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(30,30, Math.toRadians(85)),0)
                .build();

        Action pickup3rdLine = drive.actionBuilder(new Pose2d(30,30, Math.toRadians(85)))
                .lineToY(48, new TranslationalVelConstraint(10)) // forward into 2nd line of balls
                .build();

        Action toGoalWThirdLine = drive.actionBuilder(new Pose2d(29, 48, Math.toRadians(85)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28,25,Math.toRadians(-52)), 26*Math.PI/35)
                .build();


        Action moveFromLine = drive.actionBuilder(new Pose2d(-30, 27, Math.toRadians(-52)))
                .splineToConstantHeading(new Vector2d(-16,45), Math.toRadians(45))
                .build();


        if (isStopRequested()) return;

        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                            backFromGoal,
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        flywheels.wheelsOn(correction)
                                )
                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),
                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),

                        //1st line
                        new ParallelAction(
                            to1stLine,
                            intake.intakeOn(),              //intake 1st line
                            transfer.transferOnSlow()
                        ),
                        new ParallelAction(
                            pickup1stLine,           //intake 1st line
                            new SequentialAction(
                                new SleepAction(2),
                                transfer.transferOff()
                            )
                        ),
                        new SleepAction(0.25),
                        intake.intakeOff(),
                        new ParallelAction(                 //launch 1st line
                            toGoalWFirstLine,
                            new SequentialAction(
                                new SleepAction(1),
                                flywheels.wheelsOn(correction)
                            )

                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),


                        //2nd line
                        new ParallelAction(
                                to2ndLine,
                                intake.intakeOn(),              //intake 2nd line
                                transfer.transferOnSlow()
                        ),
                        new ParallelAction(
                                pickup2ndLine,
                                new SequentialAction(
                                        new SleepAction(2),
                                        transfer.transferOff()
                                )
                        ),
                        new SleepAction(0.25),
                        intake.intakeOff(),
                        new ParallelAction(                      //launch 2nd line
                                toGoalWSecondLine,
                                new SequentialAction(
                                        new SleepAction(2),
                                        flywheels.wheelsOn(correction)
                                )
                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),


                        //3rd line
                        new ParallelAction(
                                to3rdLine,
                                intake.intakeOn(),              //intake 3rd line
                                transfer.transferOnSlow()
                        ),
                        new ParallelAction(
                                pickup3rdLine,
                                new SequentialAction(
                                        new SleepAction(1.25),
                                        transfer.transferOff()
                                )
                        ),
                        new SleepAction(0.25),
                        intake.intakeOff(),
                        new ParallelAction(                      //launch 3rd line
                                toGoalWThirdLine,
                                new SequentialAction(
                                        new SleepAction(2),
                                        flywheels.wheelsOn(correction)
                                )
                        ),
                        transfer.transferOn(),
                        intake.intakeOn(),
                        new SleepAction(0.5),
                        trigger.triggerUp(),
                        new SleepAction(0.5),

                        flywheels.wheelsOff(),
                        trigger.triggerDown(),
                        intake.intakeOff(),
                        transfer.transferOff(),
                        moveFromLine
                )
        );

    }
}
