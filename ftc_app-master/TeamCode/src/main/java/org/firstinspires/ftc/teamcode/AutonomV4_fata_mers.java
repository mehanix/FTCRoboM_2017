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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

//import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous(name="Autonom v4FATAmerssiatat", group="Autonom")

public class AutonomV4_fata_mers extends AutonomV4 {

    // Declare OpMode members.
    private HardwareMap robot = new HardwareMap();
    public ElapsedTime runtime = new ElapsedTime();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    String allianceColor = "red", ballColor;
    VuMarkFinder vmf;
    RelicRecoveryVuMark vuMark;
    DriveByEncoder drive = new DriveByEncoder();
    int keyRow = -2;
    double turn_speed = 0.5, drive_speed = 0.6;

    double value90DegreeTurn = 4;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //  robot.rightDrive.setMode(DcMotor.RunMode.);

        //init chestie vuforia
        //vmf = new VuMarkFinder(hardwareMap, "RelicVuMark", true, VuforiaLocalizer.CameraDirection.BACK);

        //gyroscop


        //aici init motoare
        //aici setat directii

        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        runtime.reset();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.closeClaw();

         //extins mana

        robot.lowerArm();
        sleep(2000);
        robot.driveForward(0.5,1.0,this);
        robot.liftArm();
        robot.driveForward(4.5,1.0,this);
        sleep(30000);

    }

    public String detectColor(ColorSensor s) {
        if (s.blue() + s.red() > 20) {
            if (s.blue() > s.red()) return "blue";
            else return "red";
        }
        return "none";
    }


}