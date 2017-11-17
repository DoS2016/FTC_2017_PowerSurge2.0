package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoBlueMovement")
public class AutoBlueMovement extends Auto {



    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armServo = hardwareMap.get(Servo.class, "arm_servo_blue");
        centerDrive = hardwareMap.get(DcMotor.class, "center_drive");
        liftDrive = hardwareMap.get(DcMotor.class, "lift_drive");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");


        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        initPictoChecker();

        pictoChecker();

        initJewelChecker();

        waitForStart();

        jewelChecker();

        armServo.setPosition(0.05);

        initGyro();

        Thread.sleep(500);

        leftServo.setPosition(1);
        rightServo.setPosition(0);
        Thread.sleep(200);

        if (jewelColor == BLUE_RED){
            turnDegrees(-90, leftDrive, rightDrive);


        }
        else if (jewelColor == RED_BLUE){
            turnDegrees(90, leftDrive, rightDrive);
        }


        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armServo.setPosition(0.8);

        if (jewelColor == BLUE_RED){
        centerDrive.setPower(1);
        }
        else if (jewelColor == RED_BLUE){
            centerDrive.setPower(-1);
        }

        Thread.sleep(1000);
        centerDrive.setPower(0);


        // TODO: 11/11/2017 add code to place block into the correct collumn
    }






}

