package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class BlindAutonRedTeamDriverSide extends LinearOpMode {
    
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  leftLaunch = null;
    public DcMotor  rightLaunch = null;
    public DcMotor  leftConveyor = null;
    public DcMotor  rightConveyor = null; 
    public CRServo  intakeServo = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    
    @Override
    public void runOpMode() {
        
        // Initialization
        
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        intakeServo = hardwareMap.get(CRServo.class, "EntryServo");
        intakeServo.setPower(0);
        leftConveyor = hardwareMap.get(DcMotor.class, "leftConveyor");
        rightConveyor = hardwareMap.get(DcMotor.class, "rightConveyor");
        leftLaunch = hardwareMap.get(DcMotor.class, "leftLaunch");
        rightLaunch = hardwareMap.get(DcMotor.class, "rightLaunch");
        
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftLaunch.setDirection(DcMotor.Direction.REVERSE);
        rightLaunch.setDirection(DcMotor.Direction.FORWARD);
        leftConveyor.setDirection(DcMotor.Direction.REVERSE);
        rightConveyor.setDirection(DcMotor.Direction.FORWARD);
        
        
        
        
        // Actual code
        
        waitForStart();
        
        
        
        // Move forward
        leftFrontDrive.setPower(0.8*0.2584);
        rightFrontDrive.setPower(0.8);
        leftRearDrive.setPower(0.8);
        rightRearDrive.setPower(0.8);
        
        sleep(4000);
        
        // Stop moving
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
        
        sleep(400);
        
        // Turn right
        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(0.5);
        leftRearDrive.setPower(-0.5);
        rightRearDrive.setPower(0.5);
        
        sleep(400);
        
        // Stop turning
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
        
        sleep(200);
        
        // Move on the launch line (leftward)
        leftFrontDrive.setPower(-0.5*0.2584);
        rightFrontDrive.setPower(0.5);
        leftRearDrive.setPower(0.5);
        rightRearDrive.setPower(-0.5);
        
        sleep(600);
        
        // Stop turning
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
        
        // Prep the shooting motors
        leftLaunch.setPower(0.52);
        rightLaunch.setPower(0.52);
        
        sleep(1000);
        
        // Stop moving forward
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
        
        sleep(1000);
        
        // Shoot artifacts
        intakeServo.setPower(1.0);
        
        leftConveyor.setPower(0.5);
        rightConveyor.setPower(0.5);
        
        sleep(6000);
        
        // Stop shooting
        leftLaunch.setPower(0.0);
        rightLaunch.setPower(0.0);
        
        leftConveyor.setPower(0.0);
        rightConveyor.setPower(0.0);
        
        intakeServo.setPower(0.0);
        
        // Move off the launch line (rightward)
        leftFrontDrive.setPower(0.5*0.2584);
        rightFrontDrive.setPower(-0.5);
        leftRearDrive.setPower(-0.5);
        rightRearDrive.setPower(0.5);
        
        sleep(3000);
        
        // Stop moving
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftRearDrive.setPower(0.0);
        rightRearDrive.setPower(0.0);
        
    }
}
