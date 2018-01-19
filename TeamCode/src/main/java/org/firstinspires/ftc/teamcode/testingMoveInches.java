package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by main2 on 12/26/2017.
 */
@Autonomous(name = "MoveInchesTest")
public class testingMoveInches extends Auto {



    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armServoBlue = hardwareMap.get(Servo.class, "arm_servo_blue");
        armServoRed = hardwareMap.get(Servo.class, "arm_servo_red");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");
        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        grabNabberLeft = hardwareMap.get(DcMotor.class, "grab_nabber_left");
        grabNabberRight = hardwareMap.get(DcMotor.class, "grab_nabber_right");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        initGyro();
        waitForStart();
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        //moveInches(20);
        //Thread.sleep(1000);
    }

}
