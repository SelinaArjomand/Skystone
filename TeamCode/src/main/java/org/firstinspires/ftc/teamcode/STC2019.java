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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="STC2019", group="Pushbot")
public class STC2019 extends OpMode
{

    double liftPower = 0;

    public DcMotor leftfront;
    public DcMotor leftback;
    public DcMotor rightfront;
    public DcMotor rightback;

    public DcMotor lift;

    public Servo arm1; //right?
    public Servo arm2; //left?

    @Override
    public void init() {
        leftfront = hardwareMap.dcMotor.get("leftfront");
        leftback = hardwareMap.dcMotor.get("leftback");
        rightfront = hardwareMap.dcMotor.get("rightfront");
        rightback = hardwareMap.dcMotor.get("rightback");
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.dcMotor.get("lift");

        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");

        arm2.setDirection(Servo.Direction.REVERSE);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        float x;
        float y;
        float z;


        //Gamepad 1 Controls

        if (Math.abs(gamepad1.left_stick_x) > .2) {      // drive train
            x = -gamepad1.left_stick_x;
        } else {
            x = 0;
        }

        if (Math.abs(gamepad1.left_stick_y) > .2) {
            y = gamepad1.left_stick_y;
        } else {
            y = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) > .2) {
            z = -gamepad1.right_stick_x;
        } else {
            z = 0;
        }


        //Gamepad 2 Controls

        if (Math.abs(gamepad2.left_stick_y) > .2) {      //  linear slide lift
            liftPower = gamepad2.right_stick_y;
        } else {
            liftPower = 0;
        }

        if (gamepad2.a) {                               // arms
            arm1.setPosition(1);
            arm2.setPosition(1);
        }

        if (gamepad2.b) {
            arm1.setPosition(0);
            arm2.setPosition(0);
        }




        leftback.setPower((x - y - z)*.75);                   // powers
        leftfront.setPower((-x - y - z)*.75);
        rightback.setPower((-x - y + z)*.75);
        rightfront.setPower((x - y + z)*.75);

        lift.setPower(liftPower);

    }

}