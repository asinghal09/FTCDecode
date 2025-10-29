package org.firstinspires.ftc.transfer.Auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.transfer.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.transfer.Auto.subsystems.Transfer;
import org.firstinspires.ftc.transfer.Auto.subsystems.Flywheels;
import org.firstinspires.ftc.transfer.Auto.subsystems.Intake;
import org.firstinspires.ftc.transfer.Auto.subsystems.Trigger;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@Config
@Autonomous
public class RedGoalAuto extends LinearOpMode{

    private static final Logger log = LoggerFactory.getLogger(RedGoalAuto.class);

    @Override
    public void runOpMode(){

        Pose2d initialPose = new Pose2d(-55, 52, Math.toRadians(-57));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Flywheels flywheels = new Flywheels(hardwareMap);
        Trigger trigger = new Trigger(hardwareMap);


        Action backFromGoal = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-26, 23), new TranslationalVelConstraint(10))
                .build();
        Action to1stLine = drive.actionBuilder(new Pose2d (-26, 23, Math.toRadians(-56)))
                .splineToSplineHeading(new Pose2d(-16,25, Math.toRadians(90)),Math.PI/2)
                .build();
        Action pickup1stLine = drive.actionBuilder(new Pose2d(-16,25, Math.toRadians(90)))
                .lineToY(45, new TranslationalVelConstraint(7)) // forward into 1st line of balls
                .build();
        Action toGoalWFirstLine = drive.actionBuilder(new Pose2d(-16, 40, Math.toRadians(90)))
                .strafeTo(new Vector2d(-17, 23))
                .splineToLinearHeading(new Pose2d(-24,25,Math.toRadians(-55)), 26*Math.PI/35)
                .build();



        if (isStopRequested()) return;
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        backFromGoal,
                        flywheels.wheelsOn(),           //launch preload
                        new SleepAction(2),
                        flywheels.wheelsOff(),
                        to1stLine,
                        intake.intakeOn(),              //intake 1st line
                        pickup1stLine,
                        intake.intakeOff(),
                        toGoalWFirstLine,
                        flywheels.wheelsOn(),           //launch 1st line
                        new SleepAction(2)


                )
        );


    }
}
