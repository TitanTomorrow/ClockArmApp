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
    /// This class calculates the arm angle based upon the oscillator angle.
    /// It then creates a lookup table so that an inverse can be found, ie what is the oscillator angle based on the arm angle.
    /// There is an ambiguity when determining the oscillator angle.
    /// </summary>
    public class PathCalculator
    {
        const double _distanceBetweenPivots = 0.065; // meters
        const double _oscillatorLength = 0.04;  // meters
        const double _bossRadius = 0.0025;
        const double _armWidth = 0.02;
        const double _armOffset = (_bossRadius + _armWidth * 0.5);

        public static readonly PathCalculator Instance = new PathCalculator();

        public PathCalculator()
        {
            CreateLookup();
        }

        public double DetermineArmAngleFromOscillarAngle(double angle)
        {
            return DetermineArmAngleFromOscillarAngle(angle, true);
        }

        double DetermineArmAngleFromOscillarAngle(double angle, bool useOther)
        {
            // contact points are (x - _distanceBetweenPivots)^2 + y^2 = _oscillatorLength

            double cx = _distanceBetweenPivots + Math.Cos(angle) * _oscillatorLength;
            double cy = Math.Sin(angle) * _oscillatorLength;

            // line tangent to circle
            // y = k*x + aa
            // aa = cy - k*cx
            // circle to be tangent to..
            // x^2 + y^2 = _armOffset^2
            // x^2 + (k*x + aa)^2 = _armOffset^2
            // x^2 + (k^2*x^2 + 2*aa*k*x + aa^2 = _armOffset^2
            // (1 + k^2)*x^2 + 2*aa*k*x + aa^2 - _armOffset^2 = 0
            // polynomial a = (1 + k^2), b = 2*aa*k, c = aa^2 - _armOffset^2
            // b^2 = 4*a*c
            // 4*aa^2*k^2 = 4 * (1 + k^2) * (aa^2 - _armOffset^2)
            // aa^2*k^2 = (1 + k^2) * (aa^2 - _armOffset^2)
            // aa^2*k^2 = aa^2 - _armOffset^2 + k^2*aa^2 - k^2*_armOffset^2
            // 0 = aa^2 - _armOffset^2 - k^2*_armOffset^2
            // 0 = (cy - k*cx)*(cy - k*cx) - _armOffset^2 - k^2*_armOffset^2
            // 0 = cy^2 - 2*cy*cx*k + k^2*cx^2 - _armOffset^2 - k^2*_armOffset^2
            // 0 = (cx^2-_armOffset^2)*k^2 - (2*cy*cx)*k + cy^2 - _armOffset^2
            // solve for k 

            double ak = cx * cx - _armOffset * _armOffset;
            double bk = (2 * cy * cx);
            double ck = cy * cy - _armOffset * _armOffset;
            double sqrt2 = bk * bk - 4 * ak * ck;
            if (sqrt2 >= 0)
            {
                double sqrt = Math.Sqrt(sqrt2);
                double k0 = -(bk + sqrt) / (2 * ak);
                double k1 = -(bk - sqrt) / (2 * ak);
                double a = Math.Atan(useOther ? k1 : k0);
                return a;
            }
            return Double.NaN;
        }

        // create lookup table
        const int LOOKUP_LENGTH = 1024 * 16;
        readonly Tuple<double, double>[] _farLookUp = new Tuple<double, double>[LOOKUP_LENGTH];
        readonly Tuple<double, double>[] _closeLookUp = new Tuple<double, double>[LOOKUP_LENGTH];
        double _maxAngle = double.MinValue;
        double _minAngle = double.MaxValue;
        double _maxAnglePoint = 0;
        double _minAnglePoint = 0;
        double _lookupInc = 0;

        public double MinAngle
        {
            get
            {
                return _minAngle;
            }
        }

        public double MaxAngle
        {
            get
            {
                return _maxAngle;
            }
        }

        /// <summary>
        /// Get the oscillator angle from the arm angle
        /// </summary>
        /// <param name="angle">arm angle</param>
        /// <param name="close">which lookup table to use, the close one or far one</param>
        /// <returns></returns>
        public double GetOscillarAngle(double angle, bool close)
        {
            if (angle < _minAngle)
                angle = _minAngle;
            else if (angle > _maxAngle)
                angle = _maxAngle;

            // do some linear interpolation...
            double fi = (angle - _minAngle) / _lookupInc;
            int i_low = (int)(Math.Floor(fi));
            int i_high = (int)(Math.Ceiling(fi));
            double high_strength = fi - i_low;
            double low_strength = 1.0 - high_strength;

            System.Diagnostics.Debug.Assert(i_low >= 0);
            System.Diagnostics.Debug.Assert(i_high < LOOKUP_LENGTH);

            if(close)
            {
                return _closeLookUp[i_low].Item1 * low_strength + _closeLookUp[i_high].Item1 * high_strength;
            }
            else
            {
                return _farLookUp[i_low].Item1 * low_strength + _farLookUp[i_high].Item1 * high_strength;
            }
        }

        /// <summary>
        /// Create lookup table.
        /// </summary>
        void CreateLookup()
        {
            double inc = (Math.PI * 2) / (LOOKUP_LENGTH * 25);
            for (double ang = -Math.PI; ang <= Math.PI; ang += inc)
            {
                double a = DetermineArmAngleFromOscillarAngle(ang, true);
                if (a < _minAngle)
                {
                    _minAngle = a;
                    _minAnglePoint = ang;
                }
                if (a > _maxAngle)
                {
                    _maxAngle = a;
                    _maxAnglePoint = ang;
                }
            }
            double span = _maxAngle - _minAngle;
            _lookupInc = span / (LOOKUP_LENGTH - 1);

            for (int i = 0; i < LOOKUP_LENGTH; i++)
            {
                _closeLookUp[i] = new Tuple<double, double>(0, double.MaxValue);
                _farLookUp[i] = new Tuple<double, double>(0, double.MaxValue);
            }

            for (double ang = -Math.PI; ang <= Math.PI; ang += inc)
            {
                double a = DetermineArmAngleFromOscillarAngle(ang, true);
                double fi = (a - _minAngle) / _lookupInc;
                int i = (int)(Math.Round(fi));
                double ierror = Math.Abs(fi - i);
                i = Math.Max(0, Math.Min(LOOKUP_LENGTH - 1, i));
                if ((ang <= _maxAnglePoint) || (ang > _minAnglePoint))
                {
                    if (_closeLookUp[i].Item2 > ierror)
                    {
                        _closeLookUp[i] = new Tuple<double, double>(ang, ierror);
                    }
                }
                else
                {
                    if (_farLookUp[i].Item2 > ierror)
                    {
                        _farLookUp[i] = new Tuple<double, double>(ang, ierror);
                    }
                }
            }
        }

    }
}
