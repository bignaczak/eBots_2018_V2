package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

//@Disabled
@Autonomous
public class Auton_Depot_BETA3 extends eBotsOpMode {


    @Override
    public void runOpMode(){
        //Prepare the gyro in the Expansion hub
        initializeImu();

        //Create an array for drive motors
        ArrayList<DcMotor> motorList= new ArrayList<>();
        initializeDriveMotors(motorList);

        //Initialize motors for manipulator
        initializeManipMotors();

        //Initialize the variables that are being used in the main loop
        double spinAngle;    //NOTE:  Positive spin is towards right
        double driveX;
        double driveY;
        double spinSpeed;
        double twistToAngleSpeed = 0.35;
        long delayTime;  //delay time in milliseconds
        long extraDelayTime;  //Based on drive trajectory, sometimes extra time is needed
        double driveComponentForSamplingDirection;
        boolean goldPositionDetermined = false;
        GoldPosition goldPosition = GoldPosition.UNKNOWN;
        //Wait for the game to start(driver presses PLAY)

        // Set up our telemetry dashboard
        //composeTelemetry();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        waitForStart();
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        boolean wasMoveSuccessful;
        long endTimer = 0;
//        wasMoveSuccessful = moveByDistance(24,0,0,motorList);
//
//        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
/*        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){
//            telemetry.addData("success?", wasMoveSuccessful);
//            telemetry.addData("Position0", motorList.get(0).getCurrentPosition());
//            telemetry.addData("Position1", motorList.get(1).getCurrentPosition());
//            telemetry.addData("Position2", motorList.get(2).getCurrentPosition());
//            telemetry.addData("Position3", motorList.get(3).getCurrentPosition());
//            telemetry.addData("Heading", getCurrentHeading());
//            telemetry.update();
        }*/

//        wasMoveSuccessful = moveByDistance(-24,0,0,motorList);
//
//        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
//        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){
////            telemetry.addData("success?", wasMoveSuccessful);
////            telemetry.addData("Position0", motorList.get(0).getCurrentPosition());
////            telemetry.addData("Position1", motorList.get(1).getCurrentPosition());
////            telemetry.addData("Position2", motorList.get(2).getCurrentPosition());
////            telemetry.addData("Position3", motorList.get(3).getCurrentPosition());
////            telemetry.addData("Heading", getCurrentHeading());
////            telemetry.update();
//        }

        wasMoveSuccessful = moveByDistance(24,0,45,motorList,"Debug");

        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){

        }
//        wasMoveSuccessful = moveByDistance(-24,0,-45,motorList);
//        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
//        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){
//        }


        turnToFieldHeading(0, motorList);

//        wasMoveSuccessful = moveByDistance(-24,0,0,motorList);
//
//        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
//        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){
//        }


/*
        wasMoveSuccessful = moveByDistance(0,0,90,motorList);

        endTimer = System.nanoTime()/1000000 + 2000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer){
            telemetry.addData("success?", wasMoveSuccessful);
            telemetry.addData("Position0", motorList.get(0).getCurrentPosition());
            telemetry.addData("Position1", motorList.get(1).getCurrentPosition());
            telemetry.addData("Position2", motorList.get(2).getCurrentPosition());
            telemetry.addData("Position3", motorList.get(3).getCurrentPosition());
            telemetry.addData("Heading", getCurrentHeading());
            telemetry.update();
        }

        wasMoveSuccessful = moveByDistance(0,0,-90,motorList);

        endTimer = System.nanoTime()/1000000 + 2000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer){
            telemetry.addData("success?", wasMoveSuccessful);
            telemetry.addData("Position0", motorList.get(0).getCurrentPosition());
            telemetry.addData("Position1", motorList.get(1).getCurrentPosition());
            telemetry.addData("Position2", motorList.get(2).getCurrentPosition());
            telemetry.addData("Position3", motorList.get(3).getCurrentPosition());
            telemetry.addData("Heading", getCurrentHeading());
            telemetry.update();
        }
*/





    }


}
