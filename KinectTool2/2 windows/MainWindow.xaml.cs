/*
* The information in this file is
* Copyright(c) 2013, Andrea Nascetti andrea.nascetti@uniroma1.it, Roberta Ravanelli robertar88@hotmail.it
* and is subject to the terms and conditions of the
* GNU GPL License v3
* The license text is available from
* http://www.gnu.org/licenses/gpl-3.0.txt
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;

namespace _2_windows
{
    public partial class MainWindow : Window
    {
        public Kinect_calibrate sensore = new Kinect_calibrate();
        
        public MainWindow()
        {
            InitializeComponent();
            sensore.Starter();
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            sensore.display();        
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            sensore.Win.Hide();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            sensore.Stop();
        }

        private void slider1_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            sensore.ChangeAngle(slider1);
        }
    }
}
