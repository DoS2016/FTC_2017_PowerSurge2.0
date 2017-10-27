package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by main2 on 10/20/2017.
 */
@Autonomous(name = "testingEncoders")
public class testingEncoders extends LinearOpMode{

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        for (int i = 0; i < 3000; i = rightDrive.getCurrentPosition()){
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }

}
