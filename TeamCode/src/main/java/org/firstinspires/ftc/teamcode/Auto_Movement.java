package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by main2 on 10/14/2017.
 */

public class Auto_Movement extends AutoBlueNoMovement {



    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final int     COUNTS_PER_INCH         = (int) ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                (WHEEL_DIAMETER_INCHES * 3.14));

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor centerDrive = null;
    //private DcMotor liftDrive = null;
    //private Servo leftServo = null;
    //private Servo rightServo = null;






    public void moveToRight(DcMotor leftDrive, DcMotor rightDrive) throws InterruptedException {

        final int RIGHT_SPACE_ENCODER = (34 * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(RIGHT_SPACE_ENCODER);
        rightDrive.setTargetPosition(RIGHT_SPACE_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);
        while (rightDrive.isBusy() && leftDrive.isBusy()){

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*for (int i = 0; i < RIGHT_SPACE_ENCODER; i = rightDrive.getCurrentPosition()){
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);*/


    }
    public void moveToCenter(DcMotor leftDrive, DcMotor rightDrive) throws InterruptedException {

        final int RIGHT_SPACE_ENCODER = (34 * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(RIGHT_SPACE_ENCODER);
        rightDrive.setTargetPosition(RIGHT_SPACE_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);
        while (rightDrive.isBusy() && leftDrive.isBusy()){

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*for (int i = 0; i < RIGHT_SPACE_ENCODER; i = rightDrive.getCurrentPosition()){
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);*/


    }
    public void moveToLeft(DcMotor leftDrive, DcMotor rightDrive) throws InterruptedException {

        final int RIGHT_SPACE_ENCODER = (34 * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(RIGHT_SPACE_ENCODER);
        rightDrive.setTargetPosition(RIGHT_SPACE_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);
        while (rightDrive.isBusy() && leftDrive.isBusy()){

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*for (int i = 0; i < RIGHT_SPACE_ENCODER; i = rightDrive.getCurrentPosition()){
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);*/


    }


    // Show the elapsed game time and wheel power.
    }

