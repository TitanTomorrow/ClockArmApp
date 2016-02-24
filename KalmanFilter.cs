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
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;


namespace ClockApp
{
    public class KalmanFilter
    {
        const double HALF_PI = Math.PI * 0.5;

        public KalmanFilter()
        {
            // set default process and measurement noise 
            _q = DenseMatrix.OfArray(new double[,]
                { { 1, 0 }, { 0, 1 } });
            _r = DenseMatrix.OfArray(new double[,]
                { { 3, 0 }, { 0, 1 } });
            Reset();
        }


        public static double SanatiseAngle(double theta)
        {
            while (theta < 0)
                theta += Math.PI * 2;
            return Math.PI - (theta) % (Math.PI * 2);
        }

        public void Reset()
        {
            _position[0, 0] = _position[1, 0] = 0;
            _covariance[0, 0] = _covariance[0, 1] = _covariance[1, 0] = _covariance[1, 1] = 0;
        }

        Matrix<double> _position = DenseMatrix.Create(2, 1, 0);
        Matrix<double> _covariance = DenseMatrix.Create(2, 2, 0);
        readonly Matrix<double> _q;
        readonly Matrix<double> _r;

        public Matrix<double> Q
        {
            get
            {
                return _q;
            }
        }

        public Matrix<double> R
        {
            get
            {
                return _r;
            }
        }


        public double ArmAngle
        {
            get
            {
                return _position[0,0];
            }
        }

        public double ArmDeltaAngle
        {
            get
            {
                return _position[1, 0];
            }
        }

        /// <summary>
        /// Kalman Processing
        /// see https://www.edx.org/course/autonomous-mobile-robots-ethx-amrx-0
        /// </summary>
        /// <param name="rawAngle"></param>
        /// <param name="rawDeltaAngle"></param>
        /// <param name="deltaTime"></param>
        /// <param name="controlDeltaAngle"></param>
        /// <param name="deltaAngle"></param>
        /// <returns></returns>
        public double ProcessKalmanSample(double rawAngle, double rawDeltaAngle, double deltaTime, double controlAngleAcceleration, out double deltaAngle)
        {
            // transform function
            Matrix<double> state_transition = DenseMatrix.OfArray(new double[,]
                {{1, - deltaTime},
                {0, 1 }});

            Matrix<double> control = DenseMatrix.OfArray(new double[,]
                {{ 0.5 * controlAngleAcceleration * deltaTime * deltaTime  }, { controlAngleAcceleration * deltaTime }});

            // process noise...
            double n = Math.Abs(Math.Log10(Math.Abs(controlAngleAcceleration))) / 10;
            Matrix<double> q = _q * Math.Max(0.001, n);

            // measure noise
            Matrix<double> r = _r;

            // predict...the angle
            Matrix<double> position_hat = (state_transition * _position) + control; 
            Matrix<double> covariance_hat = ((state_transition * _covariance) * state_transition.Transpose()) + q;

            // update
            Matrix<double> z = DenseMatrix.OfArray(new double[,]
                { { rawAngle }, { rawDeltaAngle } });
            // state to observation mapping (the observation is the same as the state)

            Matrix<double> I = DenseMatrix.CreateIdentity(2);

            // update, calculate the innovations
            Matrix<double> position_innovation = z - position_hat;
            Matrix<double> covariance_innovation = (covariance_hat + r);

            Matrix<double> kalman_gain = covariance_hat * covariance_innovation.Inverse();

            Matrix<double> position = (position_hat + (kalman_gain * position_innovation));
            Matrix<double> covariance = (I - kalman_gain) * covariance_hat;

            position.CopyTo(_position);
            covariance.CopyTo(_covariance);

            deltaAngle = _position[1, 0];
            return _position[0, 0];
        }
    }
}
