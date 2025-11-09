package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@Config
@TeleOp
public class SurgicalTubingTest extends LinearOpMode {

    CRServo tube;
    public static double power = 0.75;
    public void runOpMode(){
        tube = hardwareMap.get(CRServo.class, "tube");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a)
                tube.setPower(power);
            if(gamepad1.b){
                tube.setPower(-power);
            }
            if (gamepad1.x)
                tube.setPower(0);

        }

    }


}
