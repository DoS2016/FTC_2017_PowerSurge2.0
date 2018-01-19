package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.lasarobotics.vision.util.ScreenOrientation;

import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.CENTER;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.LEFT;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.RIGHT;

@Autonomous(name = "AutoBlueMovementMiddle")
public class AutoBlueMovementMiddle extends Auto {



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
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        pictoChecker();

        initJewelChecker(ScreenOrientation.PORTRAIT);

        waitForStart();

        jewelChecker();

        armServoBlue.setPosition(0.05);


        initGyro();


        if (jewelColor == BLUE_RED) {
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(1000);
            armServoBlue.setPosition(0.8);
            Thread.sleep(500);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            Thread.sleep(200);
        }
        else if(jewelColor == RED_BLUE){
            turnDegrees(10, 1, 0.18);
            armServoBlue.setPosition(0.8);
            Thread.sleep(200);
            turnDegrees(-10, 1, 0.18);
            Thread.sleep(200);
            rightDrive.setPower(0.3);
            leftDrive.setPower(0.3);
            Thread.sleep(1500);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }

        armServoBlue.setPosition(0.8);
        leftDrive.setPower(-0.3);
        rightDrive.setPower(-0.3);
        Thread.sleep(1200);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        Thread.sleep(500);



        if(target == RIGHT) {
            moveInches(11);
        }
        else if(target == CENTER){
            moveInches(17);
        }
        else if(target == LEFT){
            moveInches(25);
        }
        else{
            telemetry.addData("value", "was unknown.  Right auto was run automatically");
            telemetry.update();
        }

        turnDegrees(90, 1, 0.2);
        leftDrive.setPower(0.3);
        rightDrive.setPower(0.3);
        Thread.sleep(1000);
        grabNabberRight.setPower(1);
        grabNabberLeft.setPower(-1);
        Thread.sleep(1000);
        leftDrive.setPower(-0.3);
        rightDrive.setPower(-0.3);
        Thread.sleep(500);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        grabNabberRight.setPower(0);
        grabNabberLeft.setPower(0);
        // TODO: 11/17/2017 get another block from the center


    }


}

