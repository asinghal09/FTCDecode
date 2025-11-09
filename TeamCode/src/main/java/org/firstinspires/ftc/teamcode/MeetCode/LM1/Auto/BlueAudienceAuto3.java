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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Flywheels;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Intake;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems.Trigger;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@Config
@Autonomous
public class BlueAudienceAuto3 extends LinearOpMode{

    private static final Logger log = LoggerFactory.getLogger(BlueAudienceAuto3.class);


    private Pose2d mirror(Pose2d p) {
        return new Pose2d(p.position.x,-p.position.y, -p.heading.toDouble());
    }

    private Vector2d mirror(Vector2d v) {
        return new Vector2d(
                v.x,      // X same
                -v.y      // Y flipped
        );
    }
    private double mirror(double y){
        return -y;
    }

    @Override
    public void runOpMode(){

        Pose2d initialPose = mirror(new Pose2d(54, 18, Math.toRadians(-4)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        Action backFromGoal = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(mirror(new Pose2d(-28, 25,Math.toRadians(-52))), mirror(Math.toRadians(128)))
                .build();

        Action to1stLine = drive.actionBuilder(mirror(new Pose2d (-26, 23, Math.toRadians(-52))))
                .setReversed(false)
                .splineToSplineHeading(mirror(new Pose2d(-16,28, Math.toRadians(85))),mirror(Math.PI/2))
                .build();

        Action pickup1stLine = drive.actionBuilder(mirror(new Pose2d(-16,28, Math.toRadians(85))))
                .lineToY(mirror(45), new TranslationalVelConstraint(4.2)) // forward into 1st line of balls
                .build();

        Action toGoalWFirstLine = drive.actionBuilder(mirror(new Pose2d(-16, 45, Math.toRadians(85))))
                .setReversed(true)
                //.splineToLinearHeading(new Pose2d(-20, 23, Math.toRadians(30)), Math.toRadians(210))
                .splineToLinearHeading(mirror(new Pose2d(-28,25,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();

        Action to2ndLine = drive.actionBuilder(mirror(new Pose2d (-30, 27, Math.toRadians(-52))))
                .setReversed(false)
                .splineToSplineHeading(mirror(new Pose2d(6.5,28, Math.toRadians(85))),mirror(Math.PI/2))
                .build();

        Action pickup2ndLine = drive.actionBuilder(mirror(new Pose2d(6.5,28, Math.toRadians(85))))
                .lineToY(mirror(45), new TranslationalVelConstraint(4.2)) // forward into 1st line of balls
                .build();


        Action toGoalWSecondLine = drive.actionBuilder(mirror(new Pose2d(6.5, 45, Math.toRadians(85))))
                .setReversed(true)
                //.splineTo(new Vector2d(0, 23), Math.toRadians(-90))
                .splineToLinearHeading(mirror(new Pose2d(-28,25,Math.toRadians(-52))), mirror(26*Math.PI/35))
                .build();
        Action moveFromLine = drive.actionBuilder(mirror(new Pose2d(-30, 27, Math.toRadians(-52))))
                .splineToConstantHeading(mirror(new Vector2d(-16,45)), mirror(Math.toRadians(45)))
                .build();


        if (isStopRequested()) return;
        trigger.triggerDown();
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(                 //launch preload
                                backFromGoal,
                                flywheels.wheelsOn(),
                                new SleepAction(2)
                        ),
                        transfer.transferOn(),
                        new SleepAction(0.2),
                        intake.intakeOn(),
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
                                flywheels.wheelsOn(),
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
                                flywheels.wheelsOn(),
                                new SleepAction(2.4),
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
