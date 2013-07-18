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
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.ComponentModel;
using System.Threading.Tasks;
using System.Windows.Controls.Primitives;
using System.IO;
using Meta.Numerics;
using Meta.Numerics.Statistics;
using Meta.Numerics.Matrices;
using Meta.Numerics.Statistics.Distributions;

namespace _2_windows
{
    
    public partial class WindowCalibrate : Window
    {
        public int a;
        public int contcol = 0;
        private Popup codePopup;
        public Ellipse[] el = new Ellipse[100]; 
        public TextBlock[] txt = new TextBlock[100]; 
        
        private const string FILE_NAME1 = "Coordinate punti griglia nei vari frame.txt";
        public StreamWriter sw1 = File.CreateText(FILE_NAME1);
        private const string FILE_NAME2 = "Mediane.txt";
        public StreamWriter sw2 = File.CreateText(FILE_NAME2);
        private const string FILE_NAME3 = "Coordinate_baricentriche_Kinect.txt";
        public StreamWriter sw3 = File.CreateText(FILE_NAME3);
        private const string FILE_NAME5 = "Grid_point_coordinates.txt";
        public StreamWriter sw5 = File.CreateText(FILE_NAME5);
        private const string FILE_NAME6 = "Coordinate_immagine.txt";
        public StreamWriter sw6 = File.CreateText(FILE_NAME6);
        private const string FILE_NAME7 = "Parametri_di_calibrazione.txt";
        public StreamWriter sw7 = File.CreateText(FILE_NAME7);
        private const string FILE_NAME8 = "Coordinate_corrette_by_matrice.txt";
        public StreamWriter sw8 = File.CreateText(FILE_NAME8);
        private const string FILE_NAME9 = "Coordinate_corrette_by_quaternione.txt";
        public StreamWriter sw9 = File.CreateText(FILE_NAME9);
        private const string FILE_NAME10 = "Matrice_rotazione_Brot.txt";
        public StreamWriter sw10 = File.CreateText(FILE_NAME10);
        private const string FILE_NAME11 = "Scarti.txt";
        public StreamWriter sw11 = File.CreateText(FILE_NAME11);
        private const string FILE_NAME12 = "CalibrationParameters.txt";
        public StreamWriter sw12 = File.CreateText(FILE_NAME12);

        public int collimazione = 0;
        public int acquisizione = 0;

        public WindowCalibrate()
        {
            InitializeComponent();
        }
   
        private void ColorImageElement_MouseLeftButtonUp_1(object sender, MouseButtonEventArgs e)
        {
            Point p = e.GetPosition(this.ColorImageElement); 
            AddEllipse(p);
        }

        public void AddEllipse(Point p)
        {
            contcol++; 

            el[contcol] = new Ellipse();
            el[contcol].Stroke = System.Windows.Media.Brushes.Black;
            el[contcol].Fill = System.Windows.Media.Brushes.Red;
            el[contcol].Width = 6;
            el[contcol].Height = 6;
            el[contcol].VerticalAlignment = VerticalAlignment.Top;
            el[contcol].HorizontalAlignment = HorizontalAlignment.Left;
            el[contcol].Margin = new Thickness(p.X, p.Y, 0, 0);
            el[contcol].MouseRightButtonDown += new MouseButtonEventHandler(el_MouseRightButtonDown);
            el[contcol].MouseRightButtonUp += new MouseButtonEventHandler(el_MouseRightButtonUp);

            txt[contcol] = new TextBlock();
            txt[contcol].Text = string.Format("{0}", contcol);
            txt[contcol].FontSize = 11;
            txt[contcol].Foreground = Brushes.Red;
            txt[contcol].Margin = new Thickness(p.X + 1, p.Y + 6, 0, 0);

            canvas1.Children.Add(el[contcol]);
            canvas1.Children.Add(txt[contcol]);
        }

        private void ColorImageElement_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            Point p = e.GetPosition(ColorImageElement);
            double zoom = e.Delta > 0 ? .2 : -.2;
            ZoomImage(p, zoom);
        }
      
        public void ZoomImage(Point p, double zoom)
        {
            TransformGroup transformgroup = (TransformGroup)ColorImageElement.RenderTransform;
            ScaleTransform transform = (ScaleTransform)transformgroup.Children[0];
            
            transform.CenterX = p.X;
            transform.CenterY = p.Y;
            transform.ScaleX += zoom;
            transform.ScaleY += zoom;
        }

        private void el_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Ellipse current = sender as Ellipse;

            this.codePopup = new Popup();
            this.codePopup.IsOpen = true;
            this.codePopup.PlacementTarget = current;
            TextBlock popupText = new TextBlock();
            popupText.Background = Brushes.LightBlue;
            popupText.Foreground = Brushes.Blue;
            
            popupText.Text = string.Format("x={0} y={1}", current.Margin.Left, current.Margin.Top);
            this.codePopup.Child = popupText;
        }

        private void el_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.codePopup.IsOpen = false;
            codePopup.IsOpen = false; 
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {  
            e.Cancel = true;
            this.Hide();
        }

        private void DelEll_Click_1(object sender, RoutedEventArgs e)
        {
            for (int i = 1; i <= contcol; i++)
            {
                canvas1.Children.Remove(el[i]);
                canvas1.Children.Remove(txt[i]);
            }
           
            this.sw1.WriteLine();
          
            contcol = 0;
            acquisizione = 0;
        }

        private void floatingTip_Opened(object sender, EventArgs e)
        {
            
        }

        private void ColorImageElement_MouseLeave(object sender, MouseEventArgs e)
        {
            floatingTip.IsOpen = false;
        }

    }
}