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
using Windows.Devices.Gpio;
using Windows.UI.Xaml;

namespace ClockApp
{
    public class MotorDriver
    {
        private readonly GpioPin _pin0;
        private readonly GpioPin _pin1;
        private readonly GpioPin _pinEnable;
        DispatcherTimer _timer;
        int _count;
        int _pin0OnCount =  0;
        int _pin0OffCount = 0;
        int _pin1OnCount = 0;
        int _pin1OffCount = 0;
        double _powerLevel = 0;


        public MotorDriver()
        {
            var gpio = GpioController.GetDefault();

            _pin0 = gpio.OpenPin(5);
            _pin0.Write(GpioPinValue.Low);
            _pin0.SetDriveMode(GpioPinDriveMode.Output);

            _pin1 = gpio.OpenPin(6);
            _pin1.Write(GpioPinValue.Low);
            _pin1.SetDriveMode(GpioPinDriveMode.Output);

            _pinEnable = gpio.OpenPin(23);
            _pinEnable.Write(GpioPinValue.High);
            _pinEnable.SetDriveMode(GpioPinDriveMode.Output);

            _timer = new DispatcherTimer();
            _timer.Interval = TimeSpan.FromMilliseconds(20);
            _timer.Tick += TimerTick;
            _timer.Start();
        }

        void TimerTick(object sender, object e)
        {
            if(_pin0OnCount == 0)
                _pin0.Write(GpioPinValue.Low);
            if(_pin1OnCount == 0)
                _pin1.Write(GpioPinValue.Low);
            if(_pin0OnCount > 0)
            {
                if(_count < _pin0OnCount)
                {
                    _pin1.Write(GpioPinValue.Low);
                    _pin0.Write(GpioPinValue.High);
                }
                else
                {
                    _pin0.Write(GpioPinValue.Low);
                    
                }
                _count++;
                if(_count >= _pin0OnCount + _pin0OffCount)
                {
                    _count = 0;
                }
            }
            else if (_pin1OnCount > 0)
            {
                if (_count < _pin1OnCount)
                {
                    _pin0.Write(GpioPinValue.Low);
                    _pin1.Write(GpioPinValue.High);
                }
                else
                {
                    _pin1.Write(GpioPinValue.Low);

                }
                _count++;
                if (_count >= _pin1OnCount + _pin1OffCount)
                {
                    _count = 0;
                }
            }
            if(_count == 0)
            {
                int on, off;
                ConvertToOnOff(out on, out off, Math.Abs(_powerLevel));
                if (_powerLevel > 0)
                {
                    _pin0OnCount = on;
                    _pin0OffCount = off;
                    _pin1OnCount = 0;
                    _pin1OffCount = 0;
                }
                else
                {
                    _pin0OnCount = 0;
                    _pin0OffCount = 0;
                    _pin1OnCount = on;
                    _pin1OffCount = off;
                }
            }
        }

        private void ConvertToOnOff(out int on, out int off, double rational)
        {
            on = (int)(Math.Round(rational * 2));
            off = 2 - on;
        }

        public void Move(double powerLevel)
        {
            _powerLevel = powerLevel;
        }

        public void Stop()
        {
            _powerLevel = 0;
            _pin0.Write(GpioPinValue.Low);
            _pin1.Write(GpioPinValue.Low);
        }

        public double PowerLevel
        {
            get
            {
                return _powerLevel;
            }
        }
    }
}
