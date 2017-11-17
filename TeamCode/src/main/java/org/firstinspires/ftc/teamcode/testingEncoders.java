package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by main2 on 10/20/2017.
 */
@Disabled
@Autonomous(name = "testingEncoders")
public class testingEncoders extends LinearOpMode{

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo redArmServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        redArmServo = hardwareMap.get(Servo.class, "arm_servo_red");

        waitForStart();
        while (opModeIsActive())
        redArmServo.setPosition(0.9);

    }

}
