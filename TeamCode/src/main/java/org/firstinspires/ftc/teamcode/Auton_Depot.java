package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.ArrayList;

//@Disabled
@Autonomous
public class Auton_Depot extends eBotsOpMode {

    private void executeDepotLeftAuton(ArrayList<DcMotor> motorList){
        //Overview
        /*push away*/                           double [] sampleLeftMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face depot*/                  double sampleLeftTurn0 = -90;
        /*move in front of sample*/             double [] sampleLeftMove1 = new double[] {20, -20, 0};
        //Lower latch and move arm to dump position
        /*push gold into depot*/                double [] sampleLeftMove2 = new double[] {1, -35, -(Math.PI*1)/6};
        /*turn to align with wall*/             double sampleLeftTurn1 = -135;
        /*move back and towards wall little*/   double [] sampleLeftMove3 = new double[] {2, 2, 0};
        //Deposit marker and claim depot
        //Extend arm & move to armAngle to travel position
        /*turn for crater drive*/               double sampleLeftTurn2 = 45;
        /*Align with wall*/                     double [] sampleLeftMove4 = new double[] {.5,-0.5,0};
        /*straighten for drive (Overturn)*/     double sampleLeftTurn3 = 55;
        /*straighten for drive slight angle*/   double sampleLeftTurn4 = 47;
        /*drive to crater*/                     double [] sampleLeftMove5 = new double[] {40,40,0};


        //Push off
        moveByDistance(sampleLeftMove0[0],sampleLeftMove0[1],sampleLeftMove0[2],motorList,"TimedTranslateAndSpin");

        moveArmtoTravelPosition();
        extendArm();

        turnToFieldHeading(sampleLeftTurn0, motorList);

        //Move in front of Sample
        moveByDistance(sampleLeftMove1[0],sampleLeftMove1[1],sampleLeftMove1[2],motorList,"TimedTranslateAndSpin");

        //MOVE ARM TO DUMP MARKER
        moveArmToDumpPosition();

        //Lower the latch
        lowerLatchToDrivePosition();

        //Push sample to depot
        moveByDistance(sampleLeftMove2[0],sampleLeftMove2[1],sampleLeftMove2[2],motorList,"TimedTranslateAndSpin");

        //Turn to align with wall
        turnToFieldHeading(sampleLeftTurn1, motorList);

        //move back and towards wall little
        moveByDistance(sampleLeftMove3[0],sampleLeftMove3[1],sampleLeftMove3[2],motorList,"TimedTranslateAndSpin");

        //wait for claiming
        depositMarkerInDepot(motorList);

        //FOLD ARE TO TRAVEL POSITION
        moveArmtoTravelPosition();

        //turn for crater drive
        turnToFieldHeading(sampleLeftTurn2, motorList);

        //push against wall
        moveByDistance(sampleLeftMove4[0],sampleLeftMove4[1],sampleLeftMove4[2],motorList,"TimedTranslateAndSpin");

        //straighten for drive
        turnToFieldHeading(sampleLeftTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleLeftTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleLeftMove5[0],sampleLeftMove5[1],sampleLeftMove5[2],motorList,"TimedTranslateAndSpin");

        waitForArmsToMove();

    }
    private void executeDepotCenterAuton(ArrayList<DcMotor> motorList){

        /*push away*/                           double [] sampleCenterMove0 = new double[] {0, -3, 0};
        //Extend arm & move to travel position
        /*turn to face depot*/                  double sampleCenterTurn0 = -90;
        /*move in front of sample*/             double [] sampleCenterMove1 = new double[] {0, -20, 0};
        //Deposit marker and claim depot
        //Lower latch and move arm to travel position
        /*push gold into depot*/                double [] sampleCenterMove2 = new double[] {0, -18, 0};

        /*move back and towards wall little*/   double [] sampleCenterMove3 = new double[] {0, 3, 0};//Move back
        /*turn to align with wall*/             double sampleCenterTurn1 = 45;
        /*Align with wall*/                     double [] sampleCenterMove4 = new double[] {6,-6,0};//Align to wall
        /*straighten for drive (Overturn)*/     double sampleCenterTurn3 = 55;
        /*straighten for drive slight angle*/   double sampleCenterTurn4 =47;
        /*drive to crater*/                     double [] sampleCenterMove5 = new double[] {52,52,0};

        //--------------------------------------------

        //Move to Sample
        moveByDistance(sampleCenterMove0[0],sampleCenterMove0[1],sampleCenterMove0[2],motorList,"TimedTranslateAndSpin");

        moveArmtoTravelPosition();
        extendArm();

        //Turn to depot
        turnToFieldHeading(sampleCenterTurn0,motorList);

        //Move to Sample
        moveByDistance(sampleCenterMove1[0],sampleCenterMove1[1],sampleCenterMove1[2],motorList,"TimedTranslateAndSpin");

        //MOVE ARM TO DUMP MARKER
        moveArmToDumpPosition();

        //wait for claiming
        depositMarkerInDepot(motorList);
        //FOLD ARE TO TRAVEL POSITION
        moveArmtoTravelPosition();
        //Lower the latch
        lowerLatchToDrivePosition();

        //Push to Depot
        moveByDistance(sampleCenterMove2[0],sampleCenterMove2[1],sampleCenterMove2[2],motorList,"TimedTranslateAndSpin");


        //move back a little
        moveByDistance(sampleCenterMove3[0],sampleCenterMove3[1],sampleCenterMove3[2],motorList,"TimedTranslateAndSpin");


        //turn towards crater
        turnToFieldHeading(sampleCenterTurn1, motorList);

        //align with wall
        moveByDistance(sampleCenterMove4[0],sampleCenterMove4[1],sampleCenterMove4[2],motorList,"TimedTranslateAndSpin");

        //straighten for drive
        turnToFieldHeading(sampleCenterTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleCenterTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleCenterMove5[0],sampleCenterMove5[1],sampleCenterMove5[2],motorList,"TimedTranslateAndSpin");

        waitForArmsToMove();

    }
    private void executeDepotRightAuton(ArrayList<DcMotor> motorList){
        //Overview
        /*push away*/                           double [] sampleRightMove0 = new double[] {0, -3, 0};  //push away
        //Extend arm & move armAngle travel position
        /*turn to face depot*/                  double sampleRightTurn0 = -90;
        /*move in front of sample*/             double [] sampleRightMove1 = new double[] {-23, -25, 0};
        //Lower latch and move arm to dump position
        /*push gold into depot*/                double [] sampleRightMove2 = new double[] {6, -35, (Math.PI*1)/6};
        /*turn to align with wall*/             double sampleRightTurn1 = -45;
        /*move back and towards wall little*/   double [] sampleRightMove3 = new double[] {-2, 2, 0};
        //Deposit marker and claim depot
        //lower latch and move to armAngle to travel position
        /*turn for crater drive*/               double sampleRightTurn2 = 135;
        /*Align with wall*/                     double [] sampleRightMove4 = new double[] {-.5,-0.5,0};
        /*straighten for drive (Overturn)*/     double sampleRightTurn3 = 125;
        /*straighten for drive slight angle*/   double sampleRightTurn4 = 133;
        /*drive to crater*/                     double [] sampleRightMove5 = new double[] {-42,42,0};




        //------------------------------------------------------
        //push away
        moveByDistance(sampleRightMove0[0],sampleRightMove0[1],sampleRightMove0[2],motorList,"TimedTranslateAndSpin");

        moveArmtoTravelPosition();
        extendArm();

        turnToFieldHeading(sampleRightTurn0,motorList);

        //Move to Sample
        moveByDistance(sampleRightMove1[0],sampleRightMove1[1],sampleRightMove1[2],motorList,"TimedTranslateAndSpin");

        //MOVE ARM TO DUMP MARKER
        moveArmToDumpPosition();

        //Lower the latch
        lowerLatchToDrivePosition();

        //Push to Depot
        moveByDistance(sampleRightMove2[0],sampleRightMove2[1],sampleRightMove2[2],motorList,"TimedTranslateAndSpin");

        //Align for turn back
        turnToFieldHeading(sampleRightTurn1, motorList);

        //move back a little
        moveByDistance(sampleRightMove3[0],sampleRightMove3[1],sampleRightMove3[2],motorList,"TimedTranslateAndSpin");

        //wait for claiming
        depositMarkerInDepot(motorList);

        //Lower latch
        lowerLatchToDrivePosition();

        //FOLD ARE TO TRAVEL POSITION
        moveArmtoTravelPosition();

        //turn towards crater
        turnToFieldHeading(sampleRightTurn2, motorList);

        //align with wall
        moveByDistance(sampleRightMove4[0],sampleRightMove4[1],sampleRightMove4[2],motorList,"TimedTranslateAndSpin");

        //straighten for drive
        turnToFieldHeading(sampleRightTurn3, motorList);  //overspin away from the wall
        turnToFieldHeading(sampleRightTurn4, motorList);  //now align for the drive

        //drive to crater
        moveByDistance(sampleRightMove5[0],sampleRightMove5[1],sampleRightMove5[2],motorList,"TimedTranslateAndSpin");

        waitForArmsToMove();

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
            executeDepotLeftAuton(motorList);
        }else if(goldPosition == GoldPosition.RIGHT){
            executeDepotRightAuton(motorList);
        }else  {        //CENTER or UNKNOWN
            executeDepotCenterAuton(motorList);
        }

    }
}
