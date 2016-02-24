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
    /// Data class, just holds measurement data
    /// </summary>
    public class AngleData : IEquatable<AngleData>, IComparable<AngleData>
    {
        public AngleData(double rawArmAngle, double rawDeltaArmAngle, double armAngle, double armDeltaAngle, double oscillatorAngle, double osc0, double osc1, double armAcceleration, double time)
        {
            if(double.IsNaN(rawArmAngle) == false)
                RawArmAngle = rawArmAngle;
            if (double.IsNaN(rawDeltaArmAngle) == false)
                RawArmDeltaAngle = rawDeltaArmAngle;
            if (double.IsNaN(armAngle) == false)
                ArmAngle = armAngle;
            if (double.IsNaN(armDeltaAngle) == false)
                ArmDeltaAngle = armDeltaAngle;
            if (double.IsNaN(oscillatorAngle) == false)
                OscillatorAngle = oscillatorAngle;
            if (double.IsNaN(osc0) == false)
                Osc0 = osc0;
            if (double.IsNaN(osc1) == false)
                Osc1 = osc1;
            if(double.IsNaN(armAcceleration) == false)
                ArmAcceleration = armAcceleration;
            Time = time;
        }

        public double RawArmAngle { get; private set; }
        public double RawArmDeltaAngle { get; private set; }
        public double ArmAngle { get; private set; }
        public double ArmDeltaAngle { get; private set; }
        public double OscillatorAngle { get; private set; }
        public double Osc0 { get; private set; }
        public double Osc1 { get; private set; }
        public double ArmAcceleration { get; private set; }
        public double Time { get; private set; }

        bool IEquatable<AngleData>.Equals(AngleData other)
        {
            return Time == other.Time;
        }

        int IComparable<AngleData>.CompareTo(AngleData other)
        {
            return Time.CompareTo(other.Time);
        }
    }

    public class Controller
    {
        MPU6050 _mpu6050 = new MPU6050();
        Calculator _calculator = new Calculator();
        double _time = 0;
        List<AngleData> Data = new List<AngleData>();
        MotorDriver _motorDriver = new MotorDriver();
        double _angleOffset = -0.03;
        public double RadiansPerSecond { get; set; }
        public double MaxAngle { get; private set; }
        public double MinAngle { get; private set; }

        public Controller()
        {
            _mpu6050.InitHardware();
            _mpu6050.SensorInterruptEvent += _mpu6050_SensorInterruptEvent;
            MaxAngle = double.MinValue;
            MinAngle = double.MaxValue;
            RadiansPerSecond = 1;
        }

        private void _mpu6050_SensorInterruptEvent(object sender, MpuSensorEventArgs e)
        {
            try
            {
                lock (Data)
                {
                    foreach (MpuSensorValue sv in e.Values)
                    {
                        double raw_angle = KalmanFilter.SanatiseAngle(Math.Atan2(sv.AccelerationY, sv.AccelerationX)) + _angleOffset;
                        double raw_delta_angle = sv.GyroZ;

                        double osc0, osc1;

                        double oscillator_angle = _calculator.Process(raw_angle, raw_delta_angle, e.SamplePeriod, RadiansPerSecond, out osc0, out osc1);

                        Data.Add(new AngleData(raw_angle, raw_delta_angle, _calculator.ArmAngle, _calculator.ArmDeltaAngle, oscillator_angle, osc0, osc1, Math.Log10(Math.Abs(_calculator.ArmAcceleration)), _time));

                        if (_calculator.ArmAngle < MinAngle)
                            MinAngle = _calculator.ArmAngle;
                        if (_calculator.ArmAngle > MaxAngle)
                            MaxAngle = _calculator.ArmAngle;

                        _time += e.SamplePeriod;

                    }
                }
            }
            catch(Exception ex)
            {
                ex.ToString();
            }
        }

        public AngleData[] GetLatestData()
        {
            lock (Data)
            {
                AngleData[] d = new AngleData[Data.Count];
                Data.CopyTo(d);
                Data.Clear();
                return d;
            }
        }


        public double PowerLevel
        {
            get
            {
                return _motorDriver.PowerLevel;
            }
            set
            {
                _motorDriver.Move(value);
            }
        }

    }
}
