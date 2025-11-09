package org.firstinspires.ftc.teamcode.MeetCode.LM1.Auto.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config

public class Intake {
    private DcMotorEx intake;
    public static double intakePower = -1;

    public Intake(HardwareMap hardwareMap) {
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }


    public Action intakeOn() {
        return t -> {
            intake.setPower(intakePower);
            return false;
        };
    }
    public Action intakeOff() {
        return t -> {
            intake.setPower(0);
            return false;
        };
    }
}
