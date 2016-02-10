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
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using Windows.Devices.Gpio;

namespace ClockApp
{
    public class MpuSensorValue
    {
        public double AccelerationX { get; set; }
        public double AccelerationY { get; set; }
        public double AccelerationZ { get; set; }
        public double GyroX { get; set; }
        public double GyroY { get; set; }
        public double GyroZ { get; set; }
    }

    public class MpuSensorEventArgs : EventArgs
    {
        public byte Status { get; set; }
        public double SamplePeriod { get; set; }
        public MpuSensorValue[] Values { get; set; }
    }

    public partial class MPU6050 : IDisposable
    {
        public event EventHandler<MpuSensorEventArgs> SensorInterruptEvent;

        #region Constants

        public const byte ADDRESS = 0x68;
        private const byte PWR_MGMT_1 = 0x6B;
        private const byte SMPLRT_DIV = 0x19;
        private const byte CONFIG = 0x1A;
        private const byte GYRO_CONFIG = 0x1B;
        private const byte ACCEL_CONFIG = 0x1C;
        private const byte FIFO_EN = 0x23;
        private const byte INT_ENABLE = 0x38;
        private const byte INT_STATUS = 0x3A;
        private const byte USER_CTRL = 0x6A;
        private const byte FIFO_COUNT = 0x72;
        private const byte FIFO_R_W = 0x74;

        private const int SensorBytes = 12;

        #endregion

        private const Int32 INTERRUPT_PIN = 18;
        I2cDevice _mpu6050Device = null;
        private GpioController IoController;
        private GpioPin InterruptPin;

        #region 12c

        private byte ReadByte(byte regAddr)
        {
            byte[] buffer = new byte[1];
            buffer[0] = regAddr;
            byte[] value = new byte[1];
            _mpu6050Device.WriteRead(buffer, value);
            return value[0];
        }

        private byte[] ReadBytes(byte regAddr, int length)
        {
            byte[] values = new byte[length];
            byte[] buffer = new byte[1];
            buffer[0] = regAddr;
            _mpu6050Device.WriteRead(buffer, values);
            return values;
        }

        public ushort ReadWord(byte address)
        {
            byte[] buffer = ReadBytes(FIFO_COUNT, 2);
            return (ushort)(((int)buffer[0] << 8) | (int)buffer[1]);
        }

        void WriteByte(byte regAddr, byte data)
        {
            byte[] buffer = new byte[2];
            buffer[0] = regAddr;
            buffer[1] = data;
            _mpu6050Device.Write(buffer);
        }

        void writeBytes(byte regAddr, byte[] values)
        {
            byte[] buffer = new byte[1 + values.Length];
            buffer[0] = regAddr;
            Array.Copy(values, 0, buffer, 1, values.Length);
            _mpu6050Device.Write(buffer);
        }

        #endregion


        public async void InitHardware()
        {
            try
            {
                IoController = GpioController.GetDefault();
                InterruptPin = IoController.OpenPin(INTERRUPT_PIN);
                InterruptPin.Write(GpioPinValue.Low);
                InterruptPin.SetDriveMode(GpioPinDriveMode.Input);
                InterruptPin.ValueChanged += Interrupt;

                string aqs = I2cDevice.GetDeviceSelector();
                DeviceInformationCollection collection = await DeviceInformation.FindAllAsync(aqs);

                I2cConnectionSettings settings = new I2cConnectionSettings(ADDRESS);
                settings.BusSpeed = I2cBusSpeed.FastMode; // 400kHz clock
                settings.SharingMode = I2cSharingMode.Exclusive;
                _mpu6050Device = await I2cDevice.FromIdAsync(collection[0].Id, settings);

                await Task.Delay(3); // wait power up sequence

                WriteByte(PWR_MGMT_1, 0x80);// reset the device
                await Task.Delay(100);
                WriteByte(PWR_MGMT_1, 0x02);
                WriteByte(USER_CTRL, 0x04); //reset fifo

                WriteByte(PWR_MGMT_1, 1); // clock source = gyro x
                WriteByte(GYRO_CONFIG, 0); // +/- 250 degrees sec
                WriteByte(ACCEL_CONFIG, 0); // +/- 2g

                WriteByte(CONFIG, 1); // 184 Hz, 2ms delay
                WriteByte(SMPLRT_DIV, 19);  // set rate 50Hz
                //WriteByte(CONFIG, 0); // 260 Hz, 0ms delay
                //WriteByte(SMPLRT_DIV, 79);  // set rate 100Hz

                WriteByte(FIFO_EN, 0x78); // enable accel and gyro to read into fifo
                WriteByte(USER_CTRL, 0x40); // reset and enable fifo
                WriteByte(INT_ENABLE, 0x1);
            }
            catch (Exception ex)
            {
                string error = ex.ToString();
            }
        }

        private void Interrupt(GpioPin sender, GpioPinValueChangedEventArgs args)
        {
            if (_mpu6050Device != null)
            {
                try
                {
                    int interrupt_status = ReadByte(INT_STATUS);
                    if ((interrupt_status & 0x10) != 0)
                    {
                        WriteByte(USER_CTRL, 0x44); // reset and enable fifo
                    }
                    if ((interrupt_status & 0x1) != 0)
                    {
                        MpuSensorEventArgs ea = new MpuSensorEventArgs();
                        ea.Status = (byte)interrupt_status;
                        ea.SamplePeriod = 0.02f;
                        List<MpuSensorValue> l = new List<MpuSensorValue>();

                        int count = ReadWord(FIFO_COUNT);
                        if (count >= SensorBytes)
                        {
                            int number_of_packets = count / SensorBytes;

                            byte[] data = ReadBytes(FIFO_R_W, number_of_packets * SensorBytes);

                            for (int i = 0; i < number_of_packets; i++)
                            {
                                int offset = i * SensorBytes;
                                short xa = (short)((int)data[offset + 0] << 8 | (int)data[offset + 1]);
                                short ya = (short)((int)data[offset + 2] << 8 | (int)data[offset + 3]);
                                short za = (short)((int)data[offset + 4] << 8 | (int)data[offset + 5]);

                                short xg = (short)((int)data[offset + 6] << 8 | (int)data[offset + 7]);
                                short yg = (short)((int)data[offset + 8] << 8 | (int)data[offset + 9]);
                                short zg = (short)((int)data[offset + 10] << 8 | (int)data[offset + 11]);

                                MpuSensorValue sv = new MpuSensorValue();
                                sv.AccelerationX = 9.81 * (double)xa / 16384.0;
                                sv.AccelerationY = 9.81 * (double)ya / 16384.0;
                                sv.AccelerationZ = 9.81 * (double)za / 16384.0;
                                sv.GyroX = (Math.PI / 180.0) * (double)xg / 131.0;
                                sv.GyroY = (Math.PI / 180.0) * yg / 131.0;
                                sv.GyroZ = (Math.PI / 180.0) * zg / 131.0;
                                l.Add(sv);
                            }
                        }
                        ea.Values = l.ToArray();

                        if (SensorInterruptEvent != null)
                        {
                            if (ea.Values.Length > 0)
                            {
                                SensorInterruptEvent(this, ea);
                            }
                        }
                    }
                }
                catch(Exception ex)
                {
                }
            }
        }

        #region IDisposable Support
        private bool disposedValue = false; // To detect redundant calls

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    if (_mpu6050Device != null)
                    {
                        _mpu6050Device.Dispose();
                        _mpu6050Device = null;
                    }
                    disposedValue = true;
                }
                disposedValue = true;
            }
        }

        // This code added to correctly implement the disposable pattern.
        void IDisposable.Dispose()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(true);
        }
        #endregion

    }
}
