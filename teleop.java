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

/*
 * This program creats an OpMode with a Mechanum wheel base and a Gecko wheel
 * and belt launching system
 * Author: 31140 FirstEnergy
 * Original sample OpMode by firstinspires.org
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class BaseTeleop extends OpMode{

    // Lay down your pieces
    // And let's begin OBJECT CREATION
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  leftLaunch = null;
    public DcMotor  rightLaunch = null;
    public DcMotor  leftConveyor = null;
    public DcMotor  rightConveyor = null; 
    public CRServo  intakeServo = null;

    double clawOffset = 0;
    
    // Fill in my data parameters
    // INITIALIZATION
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

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
        
        // Forward/backward. NOT NECESSARILY TRUE: Negate because joystick goes negative when pushed forward
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
        } else {
            intakeServo.setPower(0.0);
        }
        
        if (gamepad1.left_bumper) {
            leftConveyor.setPower(0.5);
            rightConveyor.setPower(0.5);
        } else {
            leftConveyor.setPower(0.0);
            rightConveyor.setPower(0.0);
        }

        if (gamepad1.right_bumper) {
            leftLaunch.setPower(0.55);
            rightLaunch.setPower(0.55);
        } else {
            leftLaunch.setPower(0.0);
            rightLaunch.setPower(0.0);
        }
        // Use gamepad left & right Bumpers to open and close the claw
        // if (gamepad1.right_bumper)
        //     clawOffset += CLAW_SPEED;
        // else if (gamepad1.left_bumper)
        //     clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        // clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        // leftClaw.setPosition(MID_SERVO + clawOffset);
        // rightClaw.setPosition(MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        // if (gamepad1.y)
        //     leftArm.setPower(ARM_UP_POWER);
        // else if (gamepad1.a)
        //     leftArm.setPower(ARM_DOWN_POWER);
        // else
        //     leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        // telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        // telemetry.addData("left",  "%.2f", left);
        // telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // world.execute(me);
    }
}
