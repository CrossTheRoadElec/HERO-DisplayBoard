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

using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    /**
     * Controls a common-anode LED Strip using a CTRE Driver Module connected
     * to Port 3 on the HERO.  Three low-side channels are used on the Driver Module
     * to module RGB.  Port 3 [P4,P4,P6] are used to accomplish this.
     */
    public class LEDStripController
    {
        /** PWM signals, one for each RGB value */
        PWM[] _pwms = new PWM[3];               /* RGB */
        
        float[] _dutyCycle = new float[3];      /* RGB */
        float[] _trgDutyCycle = new float[3];   /* RGB */

        /* Period between pulses */
        const uint period = 2000;

        /** Creates RGB LEDStripController for Port 3 of HERO) */
        public LEDStripController(CTRE.HERO.Port3Definition port3)
        {
            /* Duration of pulse */
            uint duration = 0;

            //Gadgeteer Drive Module
            //PIN   J2     isPWM
            //3     P1
            //4     P2       Y
            //5     P3
            //6     P4       Y      Red
            //7     P5       Y      Green     
            //8     P6       Y      Blue
            //9     ---      Y

            /* PWM Pin 6 from Hero port 3 to P4 of Driver Module */
            _pwms[0] = new PWM(port3.PWM_Pin6, period, duration, PWM.ScaleFactor.Microseconds, false);
            /* PWM Pin 7 from Hero port 3 to P5 of Driver Module */
            _pwms[1] = new PWM(port3.PWM_Pin7, period, duration, PWM.ScaleFactor.Microseconds, false);
            /* PWM Pin 8 from Hero port 3 to P6 of Driver Module */
            _pwms[2] = new PWM(port3.PWM_Pin8, period, duration, PWM.ScaleFactor.Microseconds, false);

            /* Start all PWM Pins */
            foreach (PWM pwm in _pwms)
                pwm.Start();
        }
        private void Update()
        {
            /* Fast on sequence */
            float kStepOn = 1f;
            /* Slow off sequence */
            float kStepOff = 0.30f;

            for (int i = 0; i < 3; ++i)
            {
                if (_trgDutyCycle[i] >= _dutyCycle[i])
                {
                    float chunk = _trgDutyCycle[i] - _dutyCycle[i];
                    if (chunk > kStepOn)
                        chunk = kStepOn;
                    _dutyCycle[i] += chunk;
                }
                else
                {
                    float chunk = _dutyCycle[i] - _trgDutyCycle[i];
                    if (chunk > kStepOff)
                        chunk = kStepOff;
                    _dutyCycle[i] -= chunk;
                }
            }
            /* Update hardware */
            for (int i = 0; i < 3; ++i)
                _pwms[i].DutyCycle = _dutyCycle[i];
        }

        /** Simple process of constantly updating */
        public void Process()
        {
            Update();
        }

        /** Red Value */
        public float Red
        {
            set
            {
                float v = value;
                /* Limit value from 1 to 0 */
                if (v > +1) { v = +1; }
                if (v < 0) { v = 0; }
                _trgDutyCycle[0] = v;
            }
        }

        /** Green Value */
        public float Green
        {
            set
            {
                float v = value;
                /* Limit value from 1 to 0 */
                if (v > +1) { v = +1; }
                if (v < 0) { v = 0; }
                _trgDutyCycle[1] = v;
            }
        }

        /** Blue Value */
        public float Blue
        {
            set
            {
                float v = value;
                /* Limit value from 1 to 0 */
                if (v > +1) { v = +1; }
                if (v < 0) { v = 0; }
                _trgDutyCycle[2] = v;
            }
        }
    }
}

