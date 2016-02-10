/*
Copyright(c) 2016 Graham Chow

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClockApp
{
    public class Calculator
    {
        PathCalculator _pathCalculator = new PathCalculator();
        KalmanFilter _kalmanFilter = new KalmanFilter();
        Markov _markov = new Markov();
        double _oscillatorAngle = 0;
        double _lastArmAngle = 0;
        bool _cycle;

        public static double GetAngleDiff(double a, double b)
        {
            double diff = a - b;
            if (diff < -Math.PI)
                return diff + Math.PI * 2.0;
            else if(diff > Math.PI)
                return diff - Math.PI * 2.0;
            return diff;
        }

        double CalculateArmAngularVelocity(double oscillatorRadiansPerSecond, double deltaTime)
        {
            double a0 = _pathCalculator.DetermineArmAngleFromOscillarAngle(_oscillatorAngle, _cycle);
            double a1 = _pathCalculator.DetermineArmAngleFromOscillarAngle(_oscillatorAngle + oscillatorRadiansPerSecond * deltaTime, _cycle);
            return GetAngleDiff(a1, a0);
        }

        public double Process(double rawAngle, double rawDeltaAngle, double deltaTime, double radiansPerSecond, out double osc0, out double osc1)
        {
            double arm_delta_angle = CalculateArmAngularVelocity(radiansPerSecond, deltaTime);
            double delta_arm_angle;
            double arm_angle = _kalmanFilter.ProcessKalmanSample(rawAngle, rawDeltaAngle, deltaTime, arm_delta_angle, out delta_arm_angle);

            _lastArmAngle = arm_angle;

            osc0 = _pathCalculator.GetOscillarAngle(arm_angle, true);
            osc1 = _pathCalculator.GetOscillarAngle(arm_angle, false);

            _oscillatorAngle = _markov.GetOscillatorAngle(radiansPerSecond * deltaTime, osc0, osc1);

            double diff0 = Math.Abs(Calculator.GetAngleDiff(_oscillatorAngle, osc0));
            double diff1 = Math.Abs(Calculator.GetAngleDiff(_oscillatorAngle, osc1));

            _cycle = (diff0 < diff1);

            return _oscillatorAngle;
        }

        public double ArmAngle
        {
            get
            {
                return _kalmanFilter.ArmAngle;
            }
        }

        public double ArmDeltaAngle
        {
            get
            {
                return _kalmanFilter.ArmDeltaAngle;
            }
        }



    }
}
