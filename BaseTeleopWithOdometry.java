/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This program creats an OpMode with a Mechanum wheel base and a Gecko wheel
 * and belt launching system
 * Author: 31140 FirstEnergy
 * Original sample OpMode by firstinspires.org
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class BaseTeleopWithOdometry extends OpMode{

    // Creating objects
    
    ElapsedTime timer = new ElapsedTime();
    
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  leftLaunch = null;
    public DcMotor  rightLaunch = null;
    public DcMotor  leftConveyor = null;
    public DcMotor  rightConveyor = null; 
    public CRServo  intakeServo = null;
    
    public DcMotor encoderLeft = null;
    public DcMotor encoderRight = null;
    public DcMotor encoderAux = null;

    double clawOffset = 0;
    
    static final double MID_SERVO   =  0.5 ;
    static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power
    
    static final double L = 30.5;
    static final double B = 15.25;
    static final double R = 16.0;
    static final double N = 2000.0;
    static final double cm_per_tick = 2.0*Math.PI*R/N;
    
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;
    
    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;
    
    public XyhVector START_POS = new XyhVector(213, 102, Math.toRadians(-174));
    public XyhVector pos = new XyhVector(START_POS);
    
    
    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;
    
        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();
    
        int dn1 = currentLeftPosition  - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;
    
        // the robot has moved and turned a tiny bit between two measurements:
        double dtheta = cm_per_tick * ((dn2-dn1) / (LENGTH));
        double dx = cm_per_tick * ((dn1+dn2) / 2.0);
        double dy = cm_per_tick * (dn3 + ((dn2-dn1) / 2.0));
    
        telemetrydx = dx;
        telemetrydy = dy;
        telemetrydh = dtheta;
    
        // small movement of the robot gets added to the field coordinate system:
        pos.h += dtheta / 2;
        pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
        pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
        pos.h += dtheta / 2;
        pos.h = normDiff(pos.h);
    }
    
    

    /*
     * Code to run ONCE when the driver hits INIT
     */
     
    @Override
    public void init() {
        // Define and initialize motors
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
        
        // leftArm    = hardwareMap.get(DcMotor.class, "left_arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftLaunch.setDirection(DcMotor.Direction.REVERSE);
        rightLaunch.setDirection(DcMotor.Direction.FORWARD);
        leftConveyor.setDirection(DcMotor.Direction.REVERSE);
        rightConveyor.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        // leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        // rightClaw = hardwareMap.get(Servo.class, "right_hand");
        // leftClaw.setPosition(MID_SERVO);
        // rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        
        // Movement input
        
        // Forward/backward. Negate because joystick goes negative when pushed forward
        double drivePower = gamepad1.left_stick_y;
        // Left/right
        double strafePower = -gamepad1.left_stick_x;
        // Rotate in place
        double turnPower = gamepad1.right_stick_x;
        // double turnPower;
        // if (gamepad1.right_bumper) {
        //     turnPower = 1.0;
        // } else if (gamepad1.left_bumper) {
        //     turnPower = -1.0;
        // }
        
        double leftFrontPower = -((drivePower + strafePower + turnPower)*0.2584*1.4285);
        double rightFrontPower = -((drivePower - strafePower - turnPower)*1.0);
        double leftRearPower = -((drivePower - strafePower + turnPower)*1.0);
        double rightRearPower = -((drivePower + strafePower - turnPower)*1.0);
        
        // double leftFrontPower = -((drivePower + strafePower + turnPower)*0.2584);
        // double rightFrontPower = -((drivePower - strafePower - turnPower)*0.7);
        // double leftRearPower = -((drivePower - strafePower + turnPower)*0.7);
        // double rightRearPower = -((drivePower + strafePower - turnPower)*0.7);
        
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);

        if (gamepad1.a) {
            intakeServo.setPower(1.0);
            
        } else if (gamepad1.b && !gamepad1.a) {
            intakeServo.setPower(-1.0);
            
        } else {
            intakeServo.setPower(0.0);
        }
        
        
        if (gamepad1.left_bumper) {
            leftConveyor.setPower(0.5);
            rightConveyor.setPower(0.5);
            
        } else if (gamepad1.left_trigger > 0.2 && !gamepad1.left_bumper) {
            leftConveyor.setPower(-0.5);
            rightConveyor.setPower(-0.5);
            
        } else {
            leftConveyor.setPower(0.0);
            rightConveyor.setPower(0.0);
        }

        if (gamepad1.right_bumper) {
            leftLaunch.setPower(0.52);
            rightLaunch.setPower(0.52);
        } else if (gamepad1.right_trigger > 0.2 && !gamepad1.right_bumper) {
            leftLaunch.setPower(gamepad1.right_trigger*0.1+0.45);
            rightLaunch.setPower(gamepad1.right_trigger*0.1+0.45);
        } else {
            leftLaunch.setPower(0.0);
            rightLaunch.setPower(0.0);
        }
        
        
        odometry();
        telemetry.addData("RLA","%6d   %6d   %6d",currentRightPosition,currentLeftPosition,currentAuxPosition);
        telemetry.addData("xyh","%6.1f cm   %6.1f cm   %6.1f deg",pos.x,pos.y,Math.toDegrees(robot.pos.h));
        telemetry.addData("time","%.1f ms",timer.milliseconds());
        telemetry.update();
        timer.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // world.execute(me);
    }
}
