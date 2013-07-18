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
using Microsoft.Kinect;
using Meta.Numerics;
using Meta.Numerics.Statistics;
using Meta.Numerics.Matrices;
using Meta.Numerics.Statistics.Distributions;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Windows.Media.Imaging;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Shapes;
using System.Windows.Media;
using System.Windows.Documents;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using System.Runtime.InteropServices;

namespace _2_windows
{
    public partial class Kinect_calibrate
    {
        #region Variabili Globali

        public KinectSensor _Kinect;
        public WindowCalibrate Win;
        public double lambdaX = 10.0;
        public double lambdaY = 10.0;
        private double a = 10;
        private double b = 10;
        private double c = 10;

        private WriteableBitmap _ColorImageBitmap;
        private Int32Rect _ColorImageBitmapRect;
        private int _ColorImageStride;
        private byte[] _ColorImagePixelData;
        private short[] RawDepthData;
       
        private DepthImagePoint[] depthPoints;
        private DepthImagePixel[] depthPixels;
        private ColorImageFormat currentColorImageFormat;
        private DepthImageFormat currentDepthImageFormat;
        private DepthImageFrame LastdepthFrame;
        private ColorImageFrame LastcolorFrame;

        private BackgroundWorker _Worker;
        private int messaggio = 0;

        Vector4 Accelerometer;

        const double ALFA = 58.5; 
        const double BETA = 45.6; 

        int CDFN = 0; //CurrentDepthFrameNumber 

        static int righe_griglia_calibrazione = 6;
        static int colonne_griglia_calibrazione = 9;
        static int dim = righe_griglia_calibrazione * colonne_griglia_calibrazione;
        static double step = 29; //grid step in mm
        static int collimazioni = 15;

        int check = 0;

        MultivariateSample punti_griglia_X; // statistic sample that contains the values of X coordinates of grid points in all the tot acquisitions for single collimation
        MultivariateSample punti_griglia_Y; // statistic sample that contains the values of Y coordinates of grid points in all the tot acquisitions for single collimation
        MultivariateSample punti_griglia_Z; // statistic sample that contains the values of Z coordinates of grid points in all the tot acquisitions for single collimation

        double[] X_Mediane;
        double[] Y_Mediane;
        double[] Z_Mediane;

        MultivariateSample MedianePuntiGrigliaX; //statistic sample that contains the medians of X coordinates in all the tot collimations
        MultivariateSample MedianePuntiGrigliaY; //statistic sample that contains the medians of Y coordinates in all the tot collimations
        MultivariateSample MedianePuntiGrigliaZ; //statistic sample that contains the medians of Z coordinates in all the tot collimations

        double[] X_Kinect; 
        double[] Y_Kinect; 
        double[] Z_Kinect; 

     

        #endregion Variabili Globali

        public void Starter()
        {
            this.Win = new WindowCalibrate();
            this._Worker = new BackgroundWorker();
            this._Worker.DoWork += Worker_DoWork;
            this._Worker.RunWorkerAsync();

            TransformGroup group = new TransformGroup();
            ScaleTransform xform = new ScaleTransform();
            group.Children.Add(xform);
            TranslateTransform tt = new TranslateTransform();
            group.Children.Add(tt);
            Win.ColorImageElement.RenderTransform = group;
            Win.ColorImageElement.MouseMove += new System.Windows.Input.MouseEventHandler(ColorImageElement_MouseMove);
            Win.ColorImageElement.MouseLeave += new System.Windows.Input.MouseEventHandler(ColorImageElement_MouseLeave);
            Win.button1.Click += new RoutedEventHandler(button1_Click);
            Win.button2.Click += new RoutedEventHandler(button2_Click);

            this.Win.sw1.WriteLine("ID_Acquisizione  DFN  ElevationAngle xAcc yAcc zAcc X1  Y1  Z1  X2 Y2 Z2 X3....");
            this.Win.checkBox2.IsEnabled = false; 
        }

        private void Worker_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;
            if (worker != null)
            {
                while (!worker.CancellationPending)
                {
                    DiscoverKinectSensor();
                    PollImageStream();
                }
            }
        }

        private void DiscoverKinectSensor()
        {
            if (this._Kinect != null && this._Kinect.Status != KinectStatus.Connected)
            {
                this._Kinect.ColorStream.Disable();
                this._Kinect.DepthStream.Disable();
                this._Kinect.SkeletonStream.Disable();
                this._Kinect.Stop();
                this._Kinect.AudioSource.Stop();
                this._Kinect = null;
            }
            if (this._Kinect == null)
            {
                this._Kinect = KinectSensor.KinectSensors.FirstOrDefault(x => x.Status == KinectStatus.Connected);
                if (this._Kinect != null)
                {
                    this._Kinect.ColorStream.Enable(ColorImageFormat.RgbResolution1280x960Fps12); 
                    this._Kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                    this._Kinect.DepthStream.Range = DepthRange.Default; //(Near from 400mm to 3000mm;   Default: xboxRange: from 800 mm to 4000mm)
                    this._Kinect.SkeletonStream.Enable();
                    this._Kinect.Start();
                    ColorImageStream colorStream = this._Kinect.ColorStream;
                    DepthImageStream depthStream = this._Kinect.DepthStream;

                    this.Win.ColorImageElement.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        this._ColorImageBitmap = new WriteableBitmap(colorStream.FrameWidth, colorStream.FrameHeight, 96, 96, PixelFormats.Bgr32, null);
                        this._ColorImageBitmapRect = new Int32Rect(0, 0, colorStream.FrameWidth, colorStream.FrameHeight);
                        this._ColorImageStride = colorStream.FrameWidth * colorStream.FrameBytesPerPixel;
                        this._ColorImagePixelData = new byte[colorStream.FramePixelDataLength];
                        this.depthPoints = new DepthImagePoint[colorStream.FrameWidth * colorStream.FrameHeight];
                        this.Win.ColorImageElement.Source = this._ColorImageBitmap;
                        
                        this.depthPixels = new DepthImagePixel[depthStream.FramePixelDataLength];
                        this.RawDepthData = new short[depthStream.FramePixelDataLength];
                    }));
                }
            }
        }

        private void PollImageStream()
        {
            if (this._Kinect == null)
            {
                if (messaggio == 0)
                {  
                    MessageBox.Show("There are no available sensors");
                    messaggio = 1; 
                }
            }
            else
            {
                // Retrieves and handles depth data
                DepthImageFrame Depthframe = this._Kinect.DepthStream.OpenNextFrame(100);

                if (Depthframe == null)
                {
                    return;
                }

                this.LastdepthFrame = Depthframe;
                this.currentDepthImageFormat = Depthframe.Format;

                Depthframe.CopyDepthImagePixelDataTo(this.depthPixels);

              

                //  Retrieves and handles RGB data
                ColorImageFrame Colorframe = this._Kinect.ColorStream.OpenNextFrame(100);

                if (Colorframe == null)
                {
                    return;
                }

                this.LastcolorFrame = Colorframe;
                this.currentColorImageFormat = Colorframe.Format;

                Colorframe.CopyPixelDataTo(this._ColorImagePixelData);

                CoordinateMapper mapper = _Kinect.CoordinateMapper;

                mapper.MapColorFrameToDepthFrame(this.currentColorImageFormat, this.currentDepthImageFormat, this.depthPixels, this.depthPoints);//show depth at a color point: metodo che riempie la matrice depth points
           
                this.Win.ColorImageElement.Dispatcher.BeginInvoke(new Action(() =>
                {
                    this._ColorImageBitmap.WritePixels(this._ColorImageBitmapRect, this._ColorImagePixelData, this._ColorImageStride, 0);
                }));

                this.Win.checkBox2.Dispatcher.BeginInvoke(new Action(() =>
                {
                    if (Win.checkBox2.IsChecked == true)
                    {
                        ComputeDistance();
                    }
                }));

                //Aggiorna i valori dell'accelerometro e li scrive nella textblock
                Accelerometer = new Vector4();
                Accelerometer = this._Kinect.AccelerometerGetCurrentReading();
                this.Win.textBlock3.Dispatcher.BeginInvoke(new Action(() =>
                {
                    Win.textBlock3.Text = string.Format("Accelerometer data:\nx = {0} g\ny = {1} g\nz = {2} g\nElevation Angle:\n{3}°", Math.Round(Accelerometer.X, 2), Math.Round(Accelerometer.Y, 2), Math.Round(Accelerometer.Z, 2),this._Kinect.ElevationAngle);
                }));
            }
        }

        public void display()
        {
                Win.Show();
        }

        public void Stop()
        {
            if (this._Kinect == null)
            {
                this._Kinect.ColorStream.Disable();
                this._Kinect.DepthStream.Disable();
                this._Kinect.SkeletonStream.Disable();
                this._Kinect.Stop();
                this._Kinect.AudioSource.Stop();
            }

            Win.sw1.Close();
            Win.sw2.Close();
            Win.sw3.Close();
            Win.sw5.Close();
            Win.sw6.Close();
            Win.sw7.Close();
            Win.sw8.Close();
            Win.sw9.Close();
            Win.sw10.Close();
            Win.sw11.Close();
            Win.sw12.Close();
            Application.Current.Shutdown();
        }

        public void ChangeAngle(Slider slider1)
        {
            if (_Kinect != null && _Kinect.IsRunning)
            {
                _Kinect.ElevationAngle = (int)slider1.Value;
            }
            System.Threading.Thread.Sleep(new TimeSpan(hours: 0, minutes: 0, seconds: 1));
        }

        void ColorImageElement_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            System.Windows.Point currentPos = e.GetPosition(Win.ColorImageElement);
            CursoreMouse(currentPos);
        }

        public void CursoreMouse(System.Windows.Point currentPos)
        {
            if (!Win.floatingTip.IsOpen) { Win.floatingTip.IsOpen = true; }

           
            Win.floatingTip.HorizontalOffset = currentPos.X + 10;
            Win.floatingTip.VerticalOffset = currentPos.Y;

            int colorPixelIndex = (int)(currentPos.X + ((int)currentPos.Y * this.LastcolorFrame.Width));

            DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
            int depthInMM = depthPoint.Depth;
            int y = depthPoint.Y;
            int x = depthPoint.X;

            //Calculates the pixel dimension in mm and the point coordinates

            //Axis x (pixel) ---> Axis X(mm)
            // (x = width/2 --> X=0): pixels on its right are positive 
            //const double ALFA = 58.5; // angolo di vista orizzontale nominale della DEPTH CAMERA: const equivale al define in c++
            double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension along x axes
            double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // X in mm (origin in the center of depth image)

            //Axis y (pixel) ---> Axis Y(mm)
            // y = heigth/2 --> Y=0)        
            //const double BETA = 45.6; // angolo di vista verticale nominale della DEPTH CAMERA
            double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //dimensione del pixel lungo y
            double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // X in mm (origin in the center of depth image)

            Win.textBlock6.Text = string.Format("x={0}   y={1} \n d={2} mm \n X={3} mm \n Y={4} mm ", Math.Round(currentPos.X, 2), Math.Round(currentPos.Y, 2), depthPoint.Depth, Math.Round(X, 3), Math.Round(Y, 3));// è meglio arrotondare anzichè mettere int
        }

        void ColorImageElement_MouseLeave(object sender, System.Windows.Input.MouseEventArgs e)
        {
            Win.floatingTip.IsOpen = false;
        }

        private void ComputeDistance()
        {
            if (this.LastdepthFrame.FrameNumber - CDFN > 1) //la differenza tra il frame iniziale e il frame attuale
            {
                CDFN = this.LastdepthFrame.FrameNumber;

                Win.acquisizione++;
                double[] X = new double[dim + 1];
                double[] Y = new double[dim + 1];
                double[] Z = new double[dim + 1];
                int[] DFN = new int[dim + 1];

                
                StringBuilder sb = new StringBuilder();
                sb.Append(Win.acquisizione);
                sb.Append("\t"); 
                sb.Append(CDFN);
                sb.Append("\t");
                sb.Append(this._Kinect.ElevationAngle);
                sb.Append("\t");
                sb.Append(Math.Round(this.Accelerometer.X, 3));
                sb.Append("\t");
                sb.Append(Math.Round(this.Accelerometer.Y, 3));
                sb.Append("\t");
                sb.Append(Math.Round(this.Accelerometer.Z, 3));
                sb.Append("\t");

                this.Win.sw1.Write(sb.ToString());

                //This cicle for ascribes ist own distance to every ellipse, and, as a consequence, the X and Y coordinates expressed in mm 
                for (int i = 1; i <= Win.contcol; i++)
                {
                    DFN[i] = this.LastdepthFrame.FrameNumber;
                    int colorPixelIndex = (int)(Win.el[i].Margin.Left + ((int)Win.el[i].Margin.Top * this.LastcolorFrame.Width));

                    DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
                    int depthInMM = depthPoint.Depth;
                    int y = depthPoint.Y;
                    int x = depthPoint.X;

                    //It calculates the pixel dimension in mm and the point coordinates
                    //Axis x (pixel) ---> Axis X(mm)
                    // (x = width/2 --> X=0)
                    double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension along X axis
                    double XMM = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // X in mm

                    //Axis y (pixel) ---> Axis Y(mm)
                    // y = heigth/2 --> Y=0       
                    double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //pixel dimension along Y axis
                    double YMM = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // Y in mm 
                    X[i] = XMM;
                    Y[i] = YMM;
                    Z[i] = (double)depthInMM; 
                    this.Win.sw1.Write("{0:#####.0000}    {1:#####.0000}    {2:#####.0000}    ", XMM, YMM, depthInMM);
                }

                this.Win.sw1.WriteLine();

                //For a single acquisition, the values of the three coordinates are added to the sample
                punti_griglia_X.Add(X);
                punti_griglia_Y.Add(Y);
                punti_griglia_Z.Add(Z);

            }
            
            if (Win.acquisizione == 100) // Stops the acquisition to 100 frames for single collimation
            {
                Win.collimazione++;
                Win.checkBox2.IsChecked = false;

                this.Win.sw1.WriteLine();
                this.Win.sw1.WriteLine("ID_Acquisizione  DFN  ElevationAngle xAcc yAcc zAcc X1  Y1  Z1  X2 Y2 Z2 X3....");
                this.Win.sw2.Write("{0}      ", Win.collimazione);
 
                for (int i = 1; i <= dim; i++)
                {  //Columna 1 --> Point 1;  Column 2 --> Point 2;  Column 3 --> Point 3 etc.
                    X_Mediane[i] = punti_griglia_X.Column(i).Median;
                    Y_Mediane[i] = punti_griglia_Y.Column(i).Median;
                    Z_Mediane[i] = punti_griglia_Z.Column(i).Median;
                    this.Win.sw2.Write("{0}      {1}      {2}      ", X_Mediane[i], Y_Mediane[i], Z_Mediane[i]);
                }
                this.Win.sw2.WriteLine();

                //For a single collimation, a median is added to the median sample
                MedianePuntiGrigliaX.Add(X_Mediane);
                MedianePuntiGrigliaY.Add(Y_Mediane);
                MedianePuntiGrigliaZ.Add(Z_Mediane);

                punti_griglia_X.Clear();
                punti_griglia_Y.Clear();
                punti_griglia_Z.Clear();

                for (int i = 1; i <= Win.contcol; i++)
                {
                    Win.canvas1.Children.Remove(Win.el[i]);
                    Win.canvas1.Children.Remove(Win.txt[i]);
                }

                Win.contcol = 0;
                Win.acquisizione = 0;
                Win.textBlock4.Text = string.Format("Nr. captured collimations: {0}", Win.collimazione);
                Win.button1.RaiseEvent(new RoutedEventArgs(Button.ClickEvent, Win.button1));
            }
        }

        private void button2_Click(object sender, RoutedEventArgs e)
        {
            Media_delle_Mediane();
            Calibration(X_Kinect, Y_Kinect, Z_Kinect);
        }

        public void Media_delle_Mediane()
        {
            for (int i = 1; i <= righe_griglia_calibrazione * colonne_griglia_calibrazione; i++)
            {   
                X_Kinect[i - 1] = MedianePuntiGrigliaX.Column(i).Mean;
                Y_Kinect[i - 1] = MedianePuntiGrigliaY.Column(i).Mean;
                Z_Kinect[i - 1] = MedianePuntiGrigliaZ.Column(i).Mean;
            }
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            if (check == 0)
            {
                this.Win.checkBox2.IsEnabled = true;
                Win.collimazione = 0;
                righe_griglia_calibrazione = Convert.ToInt32(Win.Rows.Text);
                colonne_griglia_calibrazione = Convert.ToInt32(Win.Columns.Text);
                dim = righe_griglia_calibrazione * colonne_griglia_calibrazione; 
                step = Convert.ToInt32(Win.Step.Text);
                collimazioni = Convert.ToInt32(Win.Collimations.Text);

                punti_griglia_X = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1); 
                punti_griglia_Y = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
                punti_griglia_Z = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
                X_Mediane = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
                Y_Mediane = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
                Z_Mediane = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
                MedianePuntiGrigliaX = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
                MedianePuntiGrigliaY = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
                MedianePuntiGrigliaZ = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);

                X_Kinect = new double[righe_griglia_calibrazione * colonne_griglia_calibrazione];
                Y_Kinect = new double[righe_griglia_calibrazione * colonne_griglia_calibrazione];
                Z_Kinect = new double[righe_griglia_calibrazione * colonne_griglia_calibrazione];
            }

            check = 1;

            System.Drawing.Size patternSize = new System.Drawing.Size(colonne_griglia_calibrazione, righe_griglia_calibrazione);

            Image<Bgra, Byte> image = new Image<Bgra, Byte>(this.LastcolorFrame.Width, this.LastcolorFrame.Height);
            image.Bytes = this._ColorImagePixelData;
            Image<Gray, Byte> InputImage = image.Convert<Gray, Byte>();


            // Create a buffer to store the chess board corner locations
            PointF[] corners = new PointF[righe_griglia_calibrazione * colonne_griglia_calibrazione];

            // Find the chess board corners
            corners = CameraCalibration.FindChessboardCorners(InputImage, patternSize, Emgu.CV.CvEnum.CALIB_CB_TYPE.ADAPTIVE_THRESH | Emgu.CV.CvEnum.CALIB_CB_TYPE.FILTER_QUADS);

            // Draw the chess board corner markers on the image
            CameraCalibration.DrawChessboardCorners(InputImage, patternSize, corners);

            System.Windows.Point[] ImageCoord = new System.Windows.Point[dim];

            for (int i = 0; i <= dim - 1; i++)
            {
                ImageCoord[i].X = corners[i].X;
                ImageCoord[i].Y = corners[i].Y;
                Win.AddEllipse(ImageCoord[i]);
                Win.sw6.WriteLine("{0}    {1}    ", ImageCoord[i].X, ImageCoord[i].Y);
            }

            this.Win.sw6.WriteLine();

           
            CvInvoke.cvWaitKey(0);

            if (Win.collimazione != collimazioni)
            {
                Win.checkBox2.IsChecked = true;
            }

            if (Win.collimazione == collimazioni)
            {
                Media_delle_Mediane();
                Calibration(X_Kinect, Y_Kinect, Z_Kinect);

                for (int i = 1; i <= Win.contcol; i++)
                {
                    Win.canvas1.Children.Remove(Win.el[i]);
                    Win.canvas1.Children.Remove(Win.txt[i]);
                }
                Win.collimazione = 0;
                Win.acquisizione = 0;
                Win.contcol = 0;
                check = 0;
            }
        }

        // Method that calculates the calibration parameters
        public void Calibration(double[] X_Kinect, double[] Y_Kinect, double[] Z_Kinect)
        {
            double[] X_Ref_B = new double[dim];
            double[] Y_Ref_B = new double[dim];
            double[] Z_Ref_B = new double[dim];
            
                if (colonne_griglia_calibrazione % 2 == 0 && righe_griglia_calibrazione % 2 == 0)
                {
                    for (int i = 0; i < righe_griglia_calibrazione; i++)
                    {
                        for (int j = 0; j < colonne_griglia_calibrazione; j++)
                        {
                            X_Ref_B[colonne_griglia_calibrazione*i + j] = (-colonne_griglia_calibrazione/2 + j)*step + step/2;
                            Y_Ref_B[colonne_griglia_calibrazione*i + j] = (righe_griglia_calibrazione/2 - i)*step - step/2;
                        }
                    }
                }

                if (colonne_griglia_calibrazione % 2 == 0 && righe_griglia_calibrazione % 2 != 0) 
                {
                    for (int i = 0; i < righe_griglia_calibrazione; i++)
                    {
                        for (int j = 0; j < colonne_griglia_calibrazione; j++)
                        {
                            X_Ref_B[colonne_griglia_calibrazione * i + j] = (-colonne_griglia_calibrazione / 2 + j) * step + step / 2;
                            Y_Ref_B[colonne_griglia_calibrazione * i + j] = ((righe_griglia_calibrazione-1)/ 2 - i) * step; 
                        }
                    }
                }

                if (colonne_griglia_calibrazione % 2 != 0 && righe_griglia_calibrazione % 2 == 0) 
                { 
                    for (int i = 0; i < righe_griglia_calibrazione; i++)
                    {
                        for (int j = 0; j < colonne_griglia_calibrazione; j++)
                        {
                            X_Ref_B[colonne_griglia_calibrazione*i + j] = (-(colonne_griglia_calibrazione-1)/2 + j)*step;
                            Y_Ref_B[colonne_griglia_calibrazione*i + j] = (righe_griglia_calibrazione/2 - i)*step - step/2;
                        }
                    } 
                }

                if (colonne_griglia_calibrazione % 2 != 0 && righe_griglia_calibrazione % 2 != 0) 
                {
                    for (int i = 0; i < righe_griglia_calibrazione; i++)
                    {
                        for (int j = 0; j < colonne_griglia_calibrazione; j++)
                        {
                            X_Ref_B[colonne_griglia_calibrazione * i + j] = (-(colonne_griglia_calibrazione - 1) / 2 + j) * step;
                            Y_Ref_B[colonne_griglia_calibrazione * i + j] = ((righe_griglia_calibrazione - 1) / 2 - i) * step;
                        }
                    }
                }

            // Grid coordinates
            for (int k = 0; k < dim; k++)
            {
                this.Win.sw5.WriteLine("{0:#####.0000}      {1:#####.0000}      {2:#####.0000}      ", X_Ref_B[k], Y_Ref_B[k], Z_Ref_B[k]);
            }
            this.Win.sw5.WriteLine();

            Sample XKINECT = new Sample(X_Kinect);
            Sample YKINECT = new Sample(Y_Kinect);
            Sample ZKINECT = new Sample(Z_Kinect);

            // barycenter of Kinect coordinates
            double X_Bar = XKINECT.Mean;
            double Y_Bar = YKINECT.Mean;
            double Z_Bar = ZKINECT.Mean;

            //Kinect barycentric coordinates contaneirs
            double[] X_Kinect_Bar = new double[dim];
            double[] Y_Kinect_Bar = new double[dim];
            double[] Z_Kinect_Bar = new double[dim];

            //Kinect barycentric coordinates 
            for (int i = 0; i < dim; i++)
            {
                X_Kinect_Bar[i] = X_Kinect[i] - X_Bar;
                Y_Kinect_Bar[i] = Y_Kinect[i] - Y_Bar;
                Z_Kinect_Bar[i] = Z_Kinect[i] - Z_Bar;

                this.Win.sw3.WriteLine("{0:#####.0000}      {1:#####.0000}      {2:#####.0000}      ", X_Kinect_Bar[i],  Y_Kinect_Bar[i],  Z_Kinect_Bar[i]);
            }
            this.Win.sw3.WriteLine();

            //Modello rotazione + 2 parametri di scala Yoss = AX+b  oppure B = Yoss-bb = AX  X = B*A^-1

            RectangularMatrix A = new RectangularMatrix(dim * 3, 5);
            ColumnVector B = new ColumnVector(dim * 3);
            ColumnVector bb = new ColumnVector(dim * 3);
            ColumnVector Yoss = new ColumnVector(dim * 3);
            ColumnVector X = new ColumnVector(5);
            SquareMatrix X1 = new SquareMatrix(5);
            ColumnVector X2 = new ColumnVector(5);

            for (int i = 0; i < dim; i++)
            {
                
                Yoss[3 * i ] = X_Ref_B[i];
                Yoss[3 * i + 1] = Y_Ref_B[i];
                Yoss[3 * i + 2] = Z_Ref_B[i];

                A[3 * i, 0] = Y_Kinect_Bar[i];
                A[3 * i, 1] = -Z_Kinect_Bar[i];
                A[3 * i, 2] = 0.0;
                A[3 * i, 3] = X_Kinect_Bar[i];
                A[3 * i, 4] = 0.0;

                A[3 * i + 1, 0] = -X_Kinect_Bar[i];
                A[3 * i + 1, 1] = 0.0;
                A[3 * i + 1, 2] = Z_Kinect_Bar[i];
                A[3 * i + 1, 3] = 0.0;
                A[3 * i + 1, 4] = Y_Kinect_Bar[i];

                A[3 * i + 2, 0] = 0.0;
                A[3 * i + 2, 1] = X_Kinect_Bar[i];
                A[3 * i + 2, 2] = -Y_Kinect_Bar[i];
                A[3 * i + 2, 3] = 0.0;
                A[3 * i + 2, 4] = 0.0;

                bb[3 * i] = X_Kinect_Bar[i];
                bb[3 * i + 1] = Y_Kinect_Bar[i];
                bb[3 * i + 2] = Z_Kinect_Bar[i];

                B[3 * i] = X_Ref_B[i] - X_Kinect_Bar[i];
                B[3 * i + 1] = Y_Ref_B[i] - Y_Kinect_Bar[i];
                B[3 * i + 2] = Z_Ref_B[i] - Z_Kinect_Bar[i];
            }

                X1 = (SquareMatrix)(A.Transpose() * A);
                X2 = (A.Transpose() * B);
                X = (X1.Inverse()) * X2;

                a = X[0];
                b = X[1];
                c = X[2];
                lambdaX = X[3];
                lambdaY = X[4];
                this.Win.sw12.Write("{0:#####.0000}      {1:#####.0000}      {2:#####.0000}      {3:#####.0000}      {4:#####.0000}      ", a, b, c, lambdaX, lambdaY);
                this.Win.sw12.WriteLine();

                this.Win.textBlock8.Text = string.Format("Calibration parameters:\na = {0} \nb = {1} \nc = {2} \nlambdaX = {3} \nlambdaY = {4}", a, b, c, lambdaX, lambdaY);
             
            SquareMatrix ROT = new SquareMatrix(3);
            ROT[0, 0] = 1 + lambdaX;
            ROT[0, 1] = a;
            ROT[0, 2] = -b;
            ROT[1, 0] = -a;
            ROT[1, 1] = 1 + lambdaY;
            ROT[1, 2] = c;
            ROT[2, 0] = b;
            ROT[2, 1] = -c;
            ROT[2, 2] = 1;
            

           
        }

        /// <summary>
        /// Delete a GDI object
        /// </summary>
        /// <param name="o">The poniter to the GDI object to be deleted</param>
        /// <returns></returns>
        [DllImport("gdi32")]
        private static extern int DeleteObject(IntPtr o);

        /// <summary>
        /// Convert an IImage to a WPF BitmapSource. The result can be used in the Set Property of Image.Source
        /// </summary>
        /// <param name="image">The Emgu CV Image</param>
        /// <returns>The equivalent BitmapSource</returns>
        public static BitmapSource ToBitmapSource(IImage image)
        {
            using (System.Drawing.Bitmap source = image.Bitmap)
            {
                IntPtr ptr = source.GetHbitmap(); //obtain the Hbitmap

                BitmapSource bs = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                    ptr,
                    IntPtr.Zero,
                    Int32Rect.Empty,
                    System.Windows.Media.Imaging.BitmapSizeOptions.FromEmptyOptions());

                DeleteObject(ptr); //release the HBitmap
                return bs;
            }
        }
    }
}