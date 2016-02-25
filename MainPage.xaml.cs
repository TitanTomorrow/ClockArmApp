/*
Copyright(c) 2016 Graham Chow

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace ClockApp
{

    public class AngleDataCollection : ObservableCollection<AngleData>
    {
        public AngleDataCollection()
        {
        }
    }

    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        Controller _controller = new Controller();
        DateTime _startTime;
        DispatcherTimer _timer;
        const double TIME_WINDOW = 5;


        public MainPage()
        {
            this.InitializeComponent();
            _startTime = DateTime.Now;
            _timer = new DispatcherTimer();
            _timer.Interval = TimeSpan.FromMilliseconds(250);
            _timer.Tick += TimerTick;
            _timer.Start();
        }

        private void TimerTick(object sender, object e)
        {
            if (this.sensorChart.Series[0].ItemsSource == null) return;

            AngleData [] latest_data = _controller.GetLatestData();

            double time = (latest_data.Length > 0) ? (latest_data[latest_data.Length - 1].Time) : 0;

            // clear out old data from the chart...
            double old_time = time - TIME_WINDOW;
            AngleDataCollection data = this.sensorChart.DataContext as AngleDataCollection;
            while (data.Count > 0)
            {
                if (data[0].Time < old_time)
                    data.RemoveAt(0);
                else
                    break;
            }

            foreach (AngleData ad in latest_data)
            {
                if (ad.Time >= old_time)
                {
                    data.Add(new AngleData(ad.RawArmAngle * 180/Math.PI, ad.RawArmDeltaAngle * 180 / Math.PI, ad.ArmAngle * 180 / Math.PI, ad.ArmDeltaAngle * 180 / Math.PI, ad.OscillatorAngle * 180 / Math.PI, ad.Osc0 * 180 / Math.PI, ad.Osc1 * 180 / Math.PI, ad.ArmAcceleration * 180 / Math.PI, ad.Time));
                }
            }

            textBoxMaxAngle.Text = _controller.MaxAngle.ToString("0.00");
            textBoxMinAngle.Text = _controller.MinAngle.ToString("0.00");
            textBoxRadiansPerSecond.Text = _controller.RadiansPerSecond.ToString("0.00");
        }

        private void buttonNegativeRadiansPerSecond_Click(object sender, RoutedEventArgs e)
        {
            _controller.RadiansPerSecond = -Math.Abs(_controller.RadiansPerSecond);
            textBoxRadiansPerSecond.Text = _controller.RadiansPerSecond.ToString("0.00");
        }

        private void buttonPositiveRadiansPerSecond_Click(object sender, RoutedEventArgs e)
        {
            _controller.RadiansPerSecond = Math.Abs(_controller.RadiansPerSecond);
            textBoxRadiansPerSecond.Text = _controller.RadiansPerSecond.ToString("0.00");
        }

        private void textBoxRadiansPerSecond_TextChanged(object sender, TextChangedEventArgs e)
        {
            double res;
            if(double.TryParse(textBoxRadiansPerSecond.Text, out res))
            {
                _controller.RadiansPerSecond = res;
            }
            textBoxRadiansPerSecond.Text = _controller.RadiansPerSecond.ToString("0.00");
        }
    }
}
