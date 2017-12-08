package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.lasarobotics.vision.util.ScreenOrientation;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.CENTER;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.RIGHT;

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
        grabNabberLeft = hardwareMap.get(DcMotor.class, "grab_nabber_left");
        grabNabberRight = hardwareMap.get(DcMotor.class, "grab_nabber_right");


        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        pictoChecker();

        initJewelChecker(ScreenOrientation.PORTRAIT);

        waitForStart();

        jewelChecker();

        armServo.setPosition(0.05);

        initGyro();

        Thread.sleep(500);
        if (jewelColor == BLUE_RED) {
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(400);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            armServo.setPosition(0.8);
        }
        else if(jewelColor == RED_BLUE){
            turnDegrees(-10);
            armServo.setPosition(0.8);
            turnDegrees(10);
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armServo.setPosition(0.8);

        if(jewelColor == BLUE_RED){
            timer = timer - 400;
        }

        if(target == RIGHT) {
            timer = 4000;
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(timer);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            turnDegrees(90);
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(500);;
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }
        else if(target == CENTER){
            timer = 3000;
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(timer);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            turnDegrees(90);
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(500);;
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }
        else if(target == LEFT){
            timer = 2000;
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(timer);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            turnDegrees(90);
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(500);;
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }
        else{
            telemetry.addData("value", "was unknown.  Right auto was run automatically");
            telemetry.update();
        }
        grabNabberRight.setPower(0.5);
        grabNabberLeft.setPower(-0.5);
        Thread.sleep(300);
        grabNabberRight.setPower(0);
        grabNabberLeft.setPower(-0);
        // TODO: 11/17/2017 get another block from the center


    }


}

