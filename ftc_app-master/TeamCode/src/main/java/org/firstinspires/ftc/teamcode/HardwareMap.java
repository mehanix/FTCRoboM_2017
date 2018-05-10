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

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Hardware Map pt robotul nostru
 */
public class HardwareMap
{
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveBack = null;
    public DcMotor liftMotor = null;
    public DcMotor totemArmMotor = null;


    public ColorSensor groundColor;
    public ColorSensor armColor;
    public BNO055IMU imu;


    public Servo totemClawServo;
    public Servo arm;
    public Servo leftClawServo;
    public Servo rightClawServo;



    double liftSpeed=0.6;
    double totemArmSpeed=0.4;
    String armStatus;
    String clawStatus;

    public  BNO055IMU.Parameters parameters;

    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDriveBack  = hwMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");
        liftMotor = hwMap.get(DcMotor.class,"lift_motor");
        totemArmMotor = hwMap.get(DcMotor.class,"totem_arm_motor");

        armColor = hwMap.get(ColorSensor.class, "arm_color");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        ;
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        // Define and initialize ALL installed servos.
        arm = hwMap.get(Servo.class, "arm_servo");
        //arm.setPosition(MID_SERVO);
        leftClawServo = hwMap.get(Servo.class,"left_claw_servo");
        rightClawServo = hwMap.get(Servo.class, "right_claw_servo");
        totemClawServo = hwMap.get(Servo.class, "totem_claw_servo");

        openClaw();
        liftArm();

    }
    public void closeClaw() {
        leftClawServo.setPosition(0.2);
        rightClawServo.setPosition(0.8);
        clawStatus="closed";
    }

    public void openClaw() {
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(0);
        clawStatus="opened";
    }

    public void closeTotemClaw() {
        totemClawServo.setPosition(0);

    }

    public void openTotemClaw() {
        totemClawServo.setPosition(1);

    }

    public void driveForward(double time,double direction ,AutonomV4 op) {

        setMotorPowers(0.6*direction,0.6*direction);
        op.sleep((long)(time * 1000));
        setMotorPowers(0,0);
    }

    public void simpleRotateLeft(long time,String rotatie,long sens, AutonomV4 op) {
        if(rotatie=="left") {


            setMotorPowers(-0.6*sens,0.6*sens);
            op.sleep(time * 1000);
            setMotorPowers(0,0);

        }
        else {

            setMotorPowers(0.6*sens,-0.6*sens);
            op.sleep(time * 1000);
            setMotorPowers(0,0);
        }
    }


    public void liftArm() {
        arm.setPosition(0);
        armStatus="lifted";
    }
    public void lowerArm() {
        arm.setPosition(0.6);
        armStatus="lowered";
    }

    public void setMotorPowers(double leftMotorPower, double rightMotorPower)
    {
        leftDrive.setPower(leftMotorPower);
        leftDriveBack.setPower(leftMotorPower);

        rightDrive.setPower(rightMotorPower);
        rightDriveBack.setPower(rightMotorPower);
    }
}

