/**
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Example that uses the Driver Module to control a RGB LED strip by using low-side outputs.
 * This example can be setup with the instructions found on the GitHub repository.
 * 
 * If a Talon SRX and a Pigeon IMU are included within your build, ensure your Talon SRX has been flashed 
 * with the included firmware. By default, code  both your Pigeon IMU and Talon SRX I.D. has been set to 0. 
 * 
 * DisplayBoard has two operation modes; Pigeon and Controller. 
 * 
 * Pigeon Mode allows you to use the Pigeon IMU to control the LED strip by tilting the board (Pitch and Roll). 
 * The color from tilting represents the color you would find from a surface of a HSV cylindrical 3D model.
 * 
 * Controller Mode allows you to control the LED strip with the left and right joystick.
 * The left joystick allows you to  control the hue.
 * The right joystick allows you to brightness whenever the left joystick is in use.
 * When both joysticks are idle (centered), DisplayBoard will begin cycling through the color sequence found
 * in ColorSequencer.cs
 * You can modify the brightness of the color cycling by modifying the brightness while using the two joysticks
 * 1. Use the left joystick to exit color cycling.
 * 2. Use the right joystick at the same time to modify brightness.
 * 3. Once the desired brightness has been found, hold the right joystick and let go of the left joystick.
 * 4. Your color cycling will now operate at the brightness choosen.
 * 
 * The values of the HSV are explained here, https://en.wikipedia.org/wiki/HSL_and_HSV
 */

using System;
using System.Threading;
using Microsoft.SPOT;
using CTRE.HERO.Module;

namespace Hero_DisplayBoard
{
    public class Program
    {
        /* DisplayBoard operation modes */
        private enum States
        {
            Controller,
            Pigeon
        }
        static States OperationState;

        /** Create LED strip controller */
        static CTRE.LEDStripController _LEDStripController = new CTRE.LEDStripController(CTRE.HERO.IO.Port3);

        /** Create a Pigeon IMU for Yaw, Pitch, and Roll (Pigeon over CAN defined with I.D.) */
        static CTRE.PigeonImu _Pigeon = new CTRE.PigeonImu(0);

        /** Create a Talon SRX for controlling the Driver Module (Talon defined with I.D.) */
        static CTRE.TalonSrx _Talon = new CTRE.TalonSrx(0);
        /** TalonID parameter for CommandLedStrip_Talon() */
        static int _TalonID = (int)_Talon.GetDeviceNumber();

        /** Create gamepad */
        static CTRE.Gamepad _Gamepad = new CTRE.Gamepad(CTRE.UsbHostDevice.GetInstance());

        /** Create color sequence */
        static ColorSequencer _ColorSequencer = new ColorSequencer();

        public static void Main()
        {
            /* Inital brightness of LED strip when color cycling */
            float _Brightness = 0.5f;

            /* Flashes LED strip */
            bool On = true;
            /* Variable for timing the flashes */
            byte i = 0;

            while (true)
            {
                /* Determines if controller is connected */
                if (_Gamepad.GetConnectionStatus() == CTRE.UsbDeviceConnection.Connected)
                    /* Controller connected for Controller Mode */
                    OperationState = States.Controller;
                else
                    /* Controller disconnected for Pigeon IMU Mode */
                    OperationState = States.Pigeon;

                if (OperationState == States.Controller)
                {
                    /* X-Axis of left joystick for hue */
                    float LeftX = _Gamepad.GetAxis(0);
                    /* Y-Axis of left joystick for hue (Stick inverted) */
                    float LeftY = -1f * _Gamepad.GetAxis(1);
                    /* Y-Axis of right stick for brightness (Stick inverted)*/
                    float RightY = -1f * _Gamepad.GetAxis(5);

                    /* Deadband the 3 joysticks */
                    Deadband(ref LeftX);
                    Deadband(ref LeftY);
                    Deadband(ref RightY);

                    if (LeftX != 0 || LeftY != 0)
                    {
                        /** Left joystick in use, stop color cycling and giver user control */

                        /* Grab brightness from the right joystick ([-1,1] + 1 * 0.5 => [0,1]) */
                        float Brightness = (RightY + 1f) * 0.5f;
                        /* Update brightness for color cylcing */
                        _Brightness = Brightness;
                        /* Update LED strip with left joystick, right joystick, and brightness */
                        UpdateLedStrip(Brightness, LeftX, LeftY);
                    }
                    else
                    {
                        /** Left joystick not in use, start color cycling */

                        /* You can change the sequence in ColorSequencer.cs by ordering the premade colors or creating your own values */
                        _ColorSequencer.Process();
                        /* Go through a color sequence at half brightness when idle */
                        UpdateLedStrip(_Brightness, _ColorSequencer.Red, _ColorSequencer.Green, _ColorSequencer.Blue);
                    }
                }
                else if (OperationState == States.Pigeon)
                {
                    /* Check status of Pigeon to see if it is connected */
                    CTRE.PigeonImu.PigeonState _PigeonState = _Pigeon.GetState();
                    if (_PigeonState == CTRE.PigeonImu.PigeonState.Ready)
                    {
                        /** Pigeon connected, giver user tilt control */

                        /* Pull Yaw, Pitch, and Roll from Pigeon */
                        float[] YPR = new float[3];
                        _Pigeon.GetYawPitchRoll(YPR);
                        float Yaw = YPR[0];
                        float Pitch = YPR[1];
                        float Roll = YPR[2];

                        /* Mulitply Pitch and Roll by PI and divide by 180 to get radians for trig functions */
                        float CPitch = Pitch * (float)System.Math.PI / 180;
                        float CRoll = Roll * (float)System.Math.PI / 180;
                        /* Find sine of Pitch and Roll */
                        CPitch = (float)System.Math.Sin(CPitch);
                        CRoll = (float)System.Math.Sin(CRoll);
                        /* Calculate inverse tangent of Pitch and Roll */
                        float Value = (float)System.Math.Atan2(CPitch, CRoll);
                        /* Convert back into degrees */
                        Value = Value * (float)(180 / System.Math.PI);

                        /* Limit the value */
                        if (Value < 0)
                            Value += 360;

                        /* Update LED strip */
                        UpdateLedStrip_Pigeon(Value);
                    }
                    else
                    {
                        /* Pigeon is not Ready/Available, so flash us */
                        i++;
                        if (i >= 40)
                        {
                            On = !On;
                            i = 0;
                        }
                        /* Decide if strip is white or off */
                        if (On == true)
                            /* White */
                            UpdateLedStrip(1, 255, 255, 255);
                        else if (On == false)
                            /* Off */
                            UpdateLedStrip(1, 0, 0, 0);
                    }
                }
                /** Quick sleep */
                Thread.Sleep(5);
            }
        }

        /**
         * Updates the LED strip (Talon/HERO) when given Brightness, R, G, and B
         * 
         * @param   Brightness  Brightness value from 0 to 1, modifies RGB values
         * @param   R           Red value of RGB
         * @param   G           Green value of RGB
         * @param   B           Blue value of RGB
         */
        static void UpdateLedStrip(float Brightness, float R, float G, float B)
        {
            /* Modify RGB values with the given Brightness */
            R *= Brightness;
            G *= Brightness;
            B *= Brightness;

            /* Update RGB values with current RGB */
            _LEDStripController.Red = R;
            _LEDStripController.Green = G;
            _LEDStripController.Blue = B;

            /* Update the LED strip through HERO */
            _LEDStripController.Process();
            /* Update the LED strip through Talon SRX */
            CommandLedStrip_Talon(R, G, B, 10, _TalonID);
        }

        /**
         * Updates the LED strip (Talon/HERO) when given Brightness, X, and Y
         * 
         * @param   Brightness  Brightness value from 0 to 1, modifies RGB values
         * @param   X           X value used when finding the angle for hue
         * @param   Y           Y value used when finding the angle for hue
         */
        static void UpdateLedStrip(float Brightness, float X, float Y)
        {
            /* The values of the HSV are explained here,
             * https://en.wikipedia.org/wiki/HSL_and_HSV */

            /* Square it to get bright quickly */
            Brightness = Brightness * Brightness;

            /* Angle */
            float HueDeg = 0;
            /* Finds the angle of the left Stick for hue */
            if (Y != 0 || X != 0)
            {
                /* Find the inverse tangent of X-axis and Y-axis of left joystick for angle */
                HueDeg = (float)System.Math.Atan2(Y, X) * 180f / (float)System.Math.PI;
                /* Keep the angle positive */
                if (HueDeg < 0)
                {
                    HueDeg += 360.0f;
                }
            }

            /* Find the saturation of HSV based on the X and Y value */
            float Saturation = (float)System.Math.Sqrt(X * X + Y * Y);
            /* Constant the value of HSV */
            float Value = 1.0f;

            /* Output after HSV to RGB conversion */
            uint R, G, B;
            /* Convert HSV to RGB */
            HsvToRgb.Convert(HueDeg, Saturation, Value, out R, out G, out B);

            /* Modify RGB values based on brightness */
            float Red = R * 1f / 255f * Brightness;
            float Green = G * 1f / 255f * Brightness;
            float Blue = B * 1f / 255f * Brightness;

            /* Update RGB values with current RGB */
            _LEDStripController.Red = Red;
            _LEDStripController.Green = Green;
            _LEDStripController.Blue = Blue;

            /* Update the LED strip through HERO */
            _LEDStripController.Process();
            /* Update the LED strip through Talon SRX */
            CommandLedStrip_Talon(Red, Green, Blue, 10, _TalonID);
        }

        /**
         * Updates the LED strip (Talon/HERO) when given angle. To be used with Pigeon
         * 
         * @param   Hue     Angle in degrees
         */
        static void UpdateLedStrip_Pigeon(float Hue)
        {
            /* 75% brigtness */
            float Brightness = 0.75f;
            /* Hue provided angle from pigeon */
            float HueDeg = Hue;
            /* Constant saturation */
            float Saturation = 1;
            /* Constant value */
            float Value = 1;
            /* Output after HSV to RGB conversion */
            uint R, G, B;
            /* Convert HSV to RGB */
            HsvToRgb.Convert(HueDeg, Saturation, Value, out R, out G, out B);

            /* Modify RGB values based on brightness */
            float Red = R * 1f / 255f * Brightness;
            float Green = G * 1f / 255f * Brightness;
            float Blue = B * 1f / 255f * Brightness;

            /* Update RGB values with current RGB */
            _LEDStripController.Red = Red;
            _LEDStripController.Green = Green;
            _LEDStripController.Blue = Blue;

            /* Update the LED strip through HERO */
            _LEDStripController.Process();
            /* Update the LED strip through Talon SRX */
            CommandLedStrip_Talon(Red, Green, Blue, 10, _TalonID);
        }

        /** 
         * Send CAN Frames to control LED strip over Talon SRX.
         * This requires the Talon to have beta firmware 2.35/10.35.
         * 
         * @param   R               Red value of RGB
         * @param   G               Green value of RGB
         * @param   B               Blue value of RGB
         * @param   LEDPeriodMs,    Period of LED
         * @param   TalonID         ID of Talon connected to Driver Module 
         */
        static void CommandLedStrip_Talon(float R, float G, float B, int LEDPeriodMs, int TalonID)
        {
            /* Talon ID based on inputted talon */
            int ID = 0x02040180 | TalonID;

            /* CAN Frame */
            byte[] frame = new byte[8];
            frame[0] = 0;   // ch3 - not supported
            frame[1] = 0;   // ch4
            frame[2] = 0;   // ch5
            frame[3] = (byte)(R * LEDPeriodMs);     // ch6
            frame[4] = (byte)(G * LEDPeriodMs);     // ch7
            frame[5] = (byte)(B * LEDPeriodMs);     // ch8
            frame[6] = (byte)LEDPeriodMs;
            frame[7] |= 1 << (3 - 1);   // ch3 - not supported
            frame[7] |= 1 << (4 - 1);   // ch4
            frame[7] |= 1 << (5 - 1);   // ch5
            frame[7] |= 1 << (6 - 1);   // ch6
            frame[7] |= 1 << (7 - 1);   // ch7
            frame[7] |= 1 << (8 - 1);   // ch8

            /* Convert CAN Frame into 64 Bit data for CAN */
            ulong data = (ulong)System.BitConverter.ToInt64(frame, 0);
            /* Send data/command to Talon SRX */
            CTRE.Native.CAN.Send((uint)ID, data, 8, 10);
        }

        /** 
         * Deadband for values under 10%
         * 
         * @param   f       the value compared to check if under 10%
         */
        static void Deadband(ref float f)
        {
            if (f < -0.1f)
            {
                /* Do nothing, outside deadband */
            }
            else if (f > +0.1f)
            {
                /* Do nothing, outside deadband */
            }
            else
                /* Within deadband, return 0 */
                f = 0;
        }
    }
}
