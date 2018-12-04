package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.ArrayList;

//@Disabled
@Autonomous
public class Auton_Crater_BETA extends eBotsOpMode {

    private void executeCraterLeftAuton(ArrayList<DcMotor> motorList){
        long endTimer = 0;

        double [] sampleLeftMove1 = new double[] {34, -72, (Math.PI)/4};
        double [] sampleLeftMove2 = new double[] {40,40,0};
        double [] sampleLeftMove3 = new double[] {-20,-20,Math.PI};
        double [] sampleLeftMove4 = new double[] {-55,-55,0};
        double sampleLeftTurn1 = 45;
        double sampleLeftTurn2 = 45;
        double sampleLeftTurn3 = -135;
        double sampleLeftTurn4 = 47;
        double sampleLeftTurn5 = 41;

        //Move to Sample
        moveByDistance(sampleLeftMove1[0],sampleLeftMove1[1],sampleLeftMove1[2],motorList,"TimedTranslateAndSpin");

        //Lower the latch
        lowerLatchToDrivePosition();

        //straighten for drive
        //turnToFieldHeading(sampleLeftTurn4, motorList);  //now align for the drive

        //drive to depot
        moveByDistance(sampleLeftMove2[0],sampleLeftMove2[1],sampleLeftMove2[2],motorList,"TimedTranslateAndSpin");

        //straighten to claim and drive drive
        turnToFieldHeading(sampleLeftTurn5, motorList);  //now align for the drive

        //wait for claiming
        endTimer = System.nanoTime()/1000000 + 2000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){

        }
        //start moving back and turn around
        //moveByDistance(sampleLeftMove3[0],sampleLeftMove3[1],sampleLeftMove3[2],motorList,"TimedTranslateAndSpin");

        turnToFieldHeading(sampleLeftTurn3, motorList);  //overspin away from the wall

        //drive to crater
        moveByDistance(sampleLeftMove4[0],sampleLeftMove4[1],sampleLeftMove4[2],motorList,"TimedTranslateAndSpin");

    }

    private void executeCraterRightAuton(ArrayList<DcMotor> motorList){
        long endTimer = 0;

        double [] sampleRightMove1 = new double[] {-23, -31, -(Math.PI*3)/6};
        double [] sampleRightMove2 = new double[] {6, -35, (Math.PI*1)/6};
        double [] sampleRightMove3 = new double[] {-0.5, 0.5, 0};
        double [] sampleRightMove4 = new double[] {-.5,-0.5,0};
        double [] sampleRightMove5 = new double[] {-2,2,0};
        double sampleRightTurn1 = -45;
        double sampleRightTurn2 = 135;
        double sampleRightTurn3 = 125;
        double sampleRightTurn4 = 133;

        //Move to Sample
        moveByDistance(sampleRightMove1[0],sampleRightMove1[1],sampleRightMove1[2],motorList,"TimedTranslateAndSpin");

        //Lower the latch
        lowerLatchToDrivePosition();

        //Push to Depot
        moveByDistance(sampleRightMove2[0],sampleRightMove2[1],sampleRightMove2[2],motorList,"TimedTranslateAndSpin");

        //Align for turn back
        turnToFieldHeading(sampleRightTurn1, motorList);

        //move back a little
        moveByDistance(sampleRightMove3[0],sampleRightMove3[1],sampleRightMove3[2],motorList,"TimedTranslateAndSpin");
        //wait for claiming
        endTimer = System.nanoTime()/1000000 + 2000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){

        }

        //turn towards crater
        turnToFieldHeading(sampleRightTurn2, motorList);

        //align with wall
        moveByDistance(sampleRightMove4[0],sampleRightMove4[1],sampleRightMove4[2],motorList,"TimedTranslateAndSpin");

        //straighten for drive
        turnToFieldHeading(sampleRightTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleRightTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleRightMove5[0],sampleRightMove5[1],sampleRightMove5[2],motorList,"TimedTranslateAndSpin");

        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){

        }

    }
    private void executeCraterCenterAuton(ArrayList<DcMotor> motorList){
        long endTimer = 0;

        double [] sampleCenterMove1 = new double[] {-3, -31, -(Math.PI*3)/6};
        double [] sampleCenterMove2 = new double[] {0, -33, 0};
        double [] sampleCenterMove3 = new double[] {0.5, 0.5, 0};
        double [] sampleCenterMove4 = new double[] {.5,-0.5,0};
        double [] sampleCenterMove5 = new double[] {5,5,0};
        double sampleCenterTurn1 = -135;
        double sampleCenterTurn2 = 45;
        double sampleCenterTurn3 = 55;
        double sampleCenterTurn4 =47;

        //Move to Sample
        moveByDistance(sampleCenterMove1[0],sampleCenterMove1[1],sampleCenterMove1[2],motorList,"TimedTranslateAndSpin");

        //Lower the latch
        lowerLatchToDrivePosition();

        //Push to Depot
        moveByDistance(sampleCenterMove2[0],sampleCenterMove2[1],sampleCenterMove2[2],motorList,"TimedTranslateAndSpin");

        //Align for turn back
        turnToFieldHeading(sampleCenterTurn1, motorList);

        //move back a little
        moveByDistance(sampleCenterMove3[0],sampleCenterMove3[1],sampleCenterMove3[2],motorList,"TimedTranslateAndSpin");
        //wait for claiming
        endTimer = System.nanoTime()/1000000 + 2000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){

        }

        //turn towards crater
        turnToFieldHeading(sampleCenterTurn2, motorList);

        //align with wall
        moveByDistance(sampleCenterMove4[0],sampleCenterMove4[1],sampleCenterMove4[2],motorList,"TimedTranslateAndSpin");

        //straighten for drive
        turnToFieldHeading(sampleCenterTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleCenterTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleCenterMove5[0],sampleCenterMove5[1],sampleCenterMove5[2],motorList,"TimedTranslateAndSpin");

        endTimer = System.nanoTime()/1000000 + 4000;//Set timer for 5 seconds
        while((System.nanoTime()/1000000)<endTimer && opModeIsActive()){

        }

    }

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

        goldPosition=landAndLocateGoldMineral();

        if(goldPosition == GoldPosition.LEFT){
            executeCraterLeftAuton(motorList);
        }else if(goldPosition == GoldPosition.RIGHT){
            executeCraterRightAuton(motorList);
        }else  {        //CENTER or UNKNOWN
            executeCraterCenterAuton(motorList);
        }

        latchMotor.setTargetPosition(0);
        latchMotor.setPower(1);
        while(latchMotor.isBusy() && opModeIsActive()){
            //do nothing
        }
        latchMotor.setPower(0);
    }
}
