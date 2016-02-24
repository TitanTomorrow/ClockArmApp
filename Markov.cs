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

    /// <summary>
    /// This implements a Markov probabilistic algorithm to resolve the ambiguity of where the oscillator is
    /// </summary>
    public class Markov
    {
        const int STATE_SPACE_SIZE = 360;
        const double ACT_SIGMA = 6.4;
        const int ACT_SIGMA_WIDTH = 45;
        const double SEE_SIGMA = 6.4;
        const int SEE_SIGMA_WIDTH = 45;
        const double SCALE = 180.0 / Math.PI;
        double[] _oscillatorAngle = new double[STATE_SPACE_SIZE];
        double[] _oscillatorAngleActCycle = new double[STATE_SPACE_SIZE];
        int _lastPeak = 0;

        public Markov()
        {
            InitialiseStateSpace();
        }

        void InitialiseStateSpace()
        {
            for (int i = 0; i < STATE_SPACE_SIZE; i++)
            {
                _oscillatorAngle[i] = 1.0 / STATE_SPACE_SIZE;
            }
        }

        double[] _actDistribution = CreateGaussianFilter(ACT_SIGMA, ACT_SIGMA_WIDTH);
        double[] _seeDistribution = CreateGaussianFilter(SEE_SIGMA, SEE_SIGMA_WIDTH);


        static double[] CreateGaussianFilter(double sigma, int width)
        {
            double[] filter = new double[width];
            double m = width / 2.0;
            double factor = 1 / (2.0 * Math.PI * sigma * sigma);
            for (int i = 0; i < width; i++)
            {
                double val = (double)((i - m) * (i - m));
                double v = Math.Exp(-val / (2.0 * sigma * sigma)) * factor;
                filter[i] = v;
            }
            return filter;
        }

        double ConvertToStateSpace(double angle)
        {
            return angle * SCALE;
        }

        double ConvertFromStateSpace(double angle)
        {
            return angle / SCALE;
        }

        int ToIndex(double angle, out double weightLow, out double weightHigh)
        {
            int low_index = (int)Math.Floor(angle);
            weightHigh = angle - low_index;
            weightLow = 1.0 - weightHigh;
            return low_index;
        }

        int SanatiseIndex(int index)
        {
            if (index < 0)
                index += (1 - (index / STATE_SPACE_SIZE)) * STATE_SPACE_SIZE;
            return index % STATE_SPACE_SIZE;
        }


        public double GetOscillatorAngle(double controlAngleChange, double measuredOscillatorAngle0, double measuredOscillatorAngle1)
        {
            double cac = ConvertToStateSpace(controlAngleChange);
            double moa0 = ConvertToStateSpace(measuredOscillatorAngle0);
            double moa1 = ConvertToStateSpace(measuredOscillatorAngle1);

            double a = GetOscillatorAngleStateSpace(cac, moa0, moa1);

            return ConvertFromStateSpace(a);
        }

        public static int GetAngleDiff(int a, int b)
        {
            int diff = a - b;
            if (diff < -180)
                return diff + 360;
            else if (diff > 180)
                return diff - 360;
            return diff;
        }

        /// <summary>
        /// Markov processing,
        /// see https://www.edx.org/course/autonomous-mobile-robots-ethx-amrx-0
        /// </summary>
        /// <param name="controlAngleChange"></param>
        /// <param name="measuredOscillatorAngle0"></param>
        /// <param name="measuredOscillatorAngle1"></param>
        /// <returns></returns>
        double GetOscillatorAngleStateSpace(double controlAngleChange, double measuredOscillatorAngle0, double measuredOscillatorAngle1)
        {
            // act cycle, update belief, convolve...
            double weight_low, weight_high;
            int offset_low = ToIndex(controlAngleChange, out weight_low, out weight_high);
            for (int i = 0; i < STATE_SPACE_SIZE; i++)
            {
                double p = 0;
                for (int ij = 0; ij < ACT_SIGMA_WIDTH; ij++)
                {
                    //if (ij >= (ACT_SIGMA_WIDTH / 2))
                    {
                        int j_low = SanatiseIndex((ij - ACT_SIGMA_WIDTH / 2) + i - offset_low);
                        int j_high = SanatiseIndex(j_low + 1);
                        if (GetAngleDiff(j_low, i) >= 0)
                        {
                            p += _actDistribution[ij] * (_oscillatorAngle[j_low] * weight_low + _oscillatorAngle[j_high] * weight_high);
                        }
                    }
                }
                _oscillatorAngleActCycle[i] = p;
            }

            // see cycle...
            // clear
            for (int i = 0; i < STATE_SPACE_SIZE; i++)
            {
                _oscillatorAngle[i] = 0;
            }
            // measuredOscillatorAngle0...
            double[] measured = new double[] { measuredOscillatorAngle0, measuredOscillatorAngle1 };
            double sum = 0;
            foreach (double m in measured)
            {
                offset_low = ToIndex(m, out weight_low, out weight_high);

                for (int ij = 0; ij < SEE_SIGMA_WIDTH; ij++)
                {
                    int j_low = SanatiseIndex((ij - SEE_SIGMA_WIDTH / 2) - offset_low);
                    int j_high = SanatiseIndex(j_low);
                    double l = _seeDistribution[ij] * _oscillatorAngleActCycle[j_low] * weight_low;
                    double h = _seeDistribution[ij] * _oscillatorAngleActCycle[j_high] * weight_high;
                    _oscillatorAngle[j_low] += l;
                    _oscillatorAngle[j_high] += h;
                    sum += l;
                    sum += h;
                }
            }
            // normalize
            int max_value = 0;
            if (sum == 0)
            {
                InitialiseStateSpace();
            }
            else
            {
                double max_strength = double.MinValue;
                for (int i = 0; i < STATE_SPACE_SIZE; i++)
                {
                    double v = (_oscillatorAngle[i] /= sum);
                    if (v > max_strength)
                    {
                        max_strength = v;
                        max_value = i;
                    }
                }
            }
            _lastPeak = max_value;

            return max_value;
        }
    }
}
