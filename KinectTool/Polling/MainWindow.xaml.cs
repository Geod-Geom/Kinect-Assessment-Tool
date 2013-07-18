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
using Microsoft.Kinect;
using System.ComponentModel;
using System.Threading.Tasks;
using System.Windows.Controls.Primitives;
using System.IO;
using Meta.Numerics;
using Meta.Numerics.Statistics;
using Meta.Numerics.Matrices;
using Meta.Numerics.Statistics.Distributions;


namespace Polling
{
    /// <summary>
    /// Logica di interazione per MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Variabili Globali
        private KinectSensor _Kinect;
        private WriteableBitmap _ColorImageBitmap;
        private Int32Rect _ColorImageBitmapRect;
        private int _ColorImageStride;
        private byte[] _ColorImagePixelData;
        private WriteableBitmap _DepthImageBitmap;
        private Int32Rect _DepthImageBitmapRect;
        private int _DepthImageStride;
        private int _DepthColoredImageStride;
        private short[] RawDepthData;
        private BackgroundWorker _Worker;
        private DepthImagePoint[] depthPoints;
        private ColorImagePoint[] colorPoints;
        private DepthImagePixel[] depthPixels;
        private ColorImageFormat currentColorImageFormat;
        private DepthImageFormat currentDepthImageFormat;
        private DepthImageFrame LastdepthFrame;
        private ColorImageFrame LastcolorFrame;
        private const string FILE_NAME = "Distanze su file (depth image).txt"; 
        private const string FILE_NAME1 = "Coordinate punti griglia nei vari frame.txt";
        private const string FILE_NAME2 = "Punti cliccati su depth.txt";
        private const string FILE_NAME3 = "Punti cliccati su colore.txt";
        //private const string FILE_NAME4 = "Distanze tra punti cliccati su colore.txt";
        private const string FILE_NAME5 = "Distanze tra punti cliccati su depth.txt";
        private const string FILE_NAME6 = "Distanze tra elllissi gialle su depth corrispondenti a ellissi rosse su RGB.txt";
        private const string FILE_NAME7 = "Dati accelerometro.txt";
        StreamWriter sw = File.CreateText(FILE_NAME);
        StreamWriter sw1 = File.CreateText(FILE_NAME1);
        StreamWriter sw2 = File.CreateText(FILE_NAME2);
        StreamWriter sw3 = File.CreateText(FILE_NAME3);
        //StreamWriter sw4 = File.CreateText(FILE_NAME4);
        StreamWriter sw5 = File.CreateText(FILE_NAME5);
        StreamWriter sw6 = File.CreateText(FILE_NAME6);
        StreamWriter sw7 = File.CreateText(FILE_NAME7);
        private Point origin;
        private Point start;
        private Popup codePopup;
        private Popup codePopup2;
        private Popup codePopup3;
        private int contcol = 0;
        private int contdepth = 0;
        private Ellipse[] el = new Ellipse[50]; 
        private TextBlock[] txt = new TextBlock[50]; 
        private Ellipse[] el2 = new Ellipse[50]; 
        private Ellipse[] el3 = new Ellipse[50]; 
        int cliccatadepth = 0;
        int cliccatagialla = 0;
        static int gray = 1000;
        int CDFN = 0; 
        int CDFN1 = 0; 
        int acquisizione = 0;
        int messaggio = 0;
        Vector4 Accelerometro;
        int[] n1 = new int[50];
        int[] n2 = new int[50];

        private const string FILE_NAME8 = "Mediane.txt";
        StreamWriter sw8 = File.CreateText(FILE_NAME8);
        private const string FILE_NAME9 = "Media delle mediane e parametri di calibrazione.txt";
        StreamWriter sw9 = File.CreateText(FILE_NAME9);

        static int righe_griglia_calibrazione = 3;
        static int colonne_griglia_calibrazione = 4;

        int collimazione = 0;

        MultivariateSample punti_griglia_bisX;// statistic sample that contains the values of X coordinates of grid points in all the tot acquisitions for single collimation
        MultivariateSample punti_griglia_bisY;// statistic sample that contains the values of Y coordinates of grid points in all the tot acquisitions for single collimation
        MultivariateSample punti_griglia_bisZ;// statistic sample that contains the values of Z coordinates of grid points in all the tot acquisitions for single collimation

        double[] X_Mediane;
        double[] Y_Mediane;
        double[] Z_Mediane;

        MultivariateSample MedianePuntiGrigliaX; //statistic sample that contains the medians of X coordinates in all the tot collimations
        MultivariateSample MedianePuntiGrigliaY;//statistic sample that contains the medians of Y coordinates in all the tot collimations
        MultivariateSample MedianePuntiGrigliaZ;//statistic sample that contains the medians of Z coordinates in all the tot collimations

        double[] X_Kinect;
        double[] Y_Kinect;
        double[] Z_Kinect;

        public double lambdaX=1;
        public double lambdaY=1;
        private double a;
        private double b;
        private double c;

        #endregion Variabili Globali

        
        public MainWindow()
        {
            InitializeComponent();
            this._Worker = new BackgroundWorker();
            this._Worker.DoWork += Worker_DoWork;
            this._Worker.RunWorkerAsync();
            this.Unloaded += (s, e) => { this._Worker.CancelAsync(); };
            TransformGroup group = new TransformGroup();
            ScaleTransform xform = new ScaleTransform();
            group.Children.Add(xform);
            TranslateTransform tt = new TranslateTransform();
            group.Children.Add(tt);
            ColorImageElement.RenderTransform = group;
            //Stampa l'intestazione del file "Distanze tra ellissi cliccate su colore"
            //this.sw4.WriteLine("ID_Acquisizione  DFN  ElevationAngle xAcc yAcc zAcc d1_2  d2_3  d3_4  d5_6  d6_7  d7_8  d9_10  d10_11  d11_12  d1_5  d2_6  d3_7  d4_8  d5_9  d6_10  d7_11  d8_12  d1_6  d2_5  d2_7  d3_6  d3_8  d4_7  d5_10  d6_9  d6_11  d7_10  d7_12  d8_11 d1_12  d4_9 d1_4  d5_8  d9_12 d1_7 d2_8 d5_11 d6_12 d3_5 d4_6 d7_9  d8_10  d1_10  d2_11  d3_12  d2_9  d3_10  d4_11  d1_3  d2_4  d5_7  d6_8  d9_11  d10_12  d1_9  d2_10  d3_11  d4_12");
            //Header of "Punti cliccati su colore" file
            this.sw3.WriteLine("DFN Elevation_Angle n°_punto xRGB yRGB xD yD d X Y  xAcc yAcc zAcc");
            //Header of "Punti cliccati su profondità" file
            this.sw2.WriteLine("DFN  Elevation_Angle n°_punto xD yD d X Y  xRGB yRGB  xAcc yAcc zAcc");
            //Header of "Coordinate punti griglia nei vari frame.txt" file
            this.sw1.WriteLine("ID_Acquisizione  DFN  ElevationAngle xAcc yAcc zAcc X1  Y1  Z1  X2 Y2 Z2 X3....");
            // Header of "Dati accelerometro" file
            this.sw7.WriteLine("CDFN   Elevation_Angle  xAcc  yAcc  zAcc");
            //Header of "Mediane" file
            this.sw8.WriteLine("ID_collimazione    Xmediana1     Ymediana1      Zmediana1      Xmediana2      Ymediana2      Zmediana2      Xmediana3 etc etc");
            //Header of "Media delle mediane"
            this.sw9.WriteLine("n° collimazioni mediate   Xmedia1     Ymedia1      Zmedia1   Xmedia2     Ymedia2      Zmedia2      Xmedia3  etc etc");
            
           
            n1[1] = 1; n2[1] = 7;
            n1[2] = 2; n2[2] = 8;
            n1[3] = 5; n2[3] = 11;
            n1[4] = 6; n2[4] = 12;
            n1[5] = 3; n2[5] = 5;
            n1[6] = 4; n2[6] = 6;
            n1[7] = 7; n2[7] = 9;
            n1[8] = 8; n2[8] = 10;
            n1[9] = 1; n2[9] = 10;
            n1[10] = 2; n2[10] = 11;
            n1[11] = 3; n2[11] = 12;
            n1[12] = 2; n2[12] = 9;
            n1[13] = 3; n2[13] = 10;
            n1[14] = 4; n2[14] = 11;
            //lati 2l
            n1[15] = 1; n2[15] = 3;
            n1[16] = 2; n2[16] = 4;
            n1[17] = 5; n2[17] = 7;
            n1[18] = 6; n2[18] = 8;
            n1[19] = 9; n2[19] = 11;
            n1[20] = 10; n2[20] = 12;
            n1[21] = 1; n2[21] = 9;
            n1[22] =2 ; n2[22] = 10;
            n1[23] = 3; n2[23] = 11;
            n1[24] = 4; n2[24] = 12;

            this.checkBox2.IsEnabled = false;
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
                    //this._Kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                    this._Kinect.ColorStream.Enable(ColorImageFormat.RgbResolution1280x960Fps12);
                    this._Kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                    this._Kinect.DepthStream.Range = DepthRange.Default; //(Near from 400mm to 3000mm;   Default: xboxRange: from 800 mm to 4000mm)
                    this._Kinect.SkeletonStream.Enable();
                    this._Kinect.Start();
                    ColorImageStream colorStream = this._Kinect.ColorStream;
                    DepthImageStream depthStream = this._Kinect.DepthStream;

                    this.ColorImageElement.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        this._ColorImageBitmap = new WriteableBitmap(colorStream.FrameWidth, colorStream.FrameHeight, 96, 96, PixelFormats.Bgr32, null);
                        this._ColorImageBitmapRect = new Int32Rect(0, 0, colorStream.FrameWidth, colorStream.FrameHeight);
                        this._ColorImageStride = colorStream.FrameWidth * colorStream.FrameBytesPerPixel;
                        this._ColorImagePixelData = new byte[colorStream.FramePixelDataLength];
                        this.depthPoints = new DepthImagePoint[colorStream.FrameWidth * colorStream.FrameHeight];
                        this.ColorImageElement.Source = this._ColorImageBitmap;
                    }));

                    this.DepthImageElement.Dispatcher.BeginInvoke(new Action(() =>
                    {
                        //this._DepthImageBitmap = new WriteableBitmap(depthStream.FrameWidth, depthStream.FrameHeight, 96, 96, PixelFormats.Gray16, null);
                        this._DepthImageBitmap = new WriteableBitmap(depthStream.FrameWidth, depthStream.FrameHeight, 96, 96, PixelFormats.Bgr32, null);
                        this._DepthImageBitmapRect = new Int32Rect(0, 0, depthStream.FrameWidth, depthStream.FrameHeight);
                        this._DepthImageStride = depthStream.FrameWidth * depthStream.FrameBytesPerPixel;
                        this._DepthColoredImageStride = depthStream.FrameWidth * colorStream.FrameBytesPerPixel;
                        this.depthPixels = new DepthImagePixel[depthStream.FramePixelDataLength];
                        this.RawDepthData = new short[depthStream.FramePixelDataLength];
                        this.colorPoints = new ColorImagePoint[depthStream.FrameWidth * depthStream.FrameHeight];
                        this.DepthImageElement.Source = this._DepthImageBitmap;
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
                     //Retrieves and handles depth data
                    DepthImageFrame Depthframe = this._Kinect.DepthStream.OpenNextFrame(100);//The 100 milliseconds timeout duration passed to the OpenNextFrame call is fairly arbitrary. A well-chosen timeout ensures the application continues to operate smoothly even if a frame or two are skipped. You will also want your application to maintain as close to 30 frames per second as possible.(Pag 42 kinectprogramming)
                            
                            if (Depthframe == null)
                            {
                                return;
                            }

                            this.LastdepthFrame = Depthframe;
                            this.currentDepthImageFormat = Depthframe.Format;

                            
                            Depthframe.CopyDepthImagePixelDataTo(this.depthPixels);

          
                            //Depthframe.CopyPixelDataTo(this.RawDepthData);
                            
                           
                            byte[] DepthColoredPixels = GenerateColoredBytes(Depthframe);

                            
                            this.DepthImageElement.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                //this._DepthImageBitmap.WritePixels(this._DepthImageBitmapRect, this.RawDepthData, this._DepthImageStride, 0);// serve per la depth map a 16 livelli di grigio
                                this._DepthImageBitmap.WritePixels(this._DepthImageBitmapRect, DepthColoredPixels, this._DepthColoredImageStride, 0);// serve per la depth map a 32 livelli di grigio
                            }));

                            this.textBlock3.Dispatcher.BeginInvoke(new Action(() =>
                            {
                               this.textBlock3.Text = string.Format("DepthFrameNumber(DFN): {0} ", Depthframe.FrameNumber);// forse serve un dispatcher
                            }));

                            //  Retrieves and handles RGB data
                            ColorImageFrame Colorframe = this._Kinect.ColorStream.OpenNextFrame(100);

                            if (Colorframe == null)
                            {
                                return;
                            }

                            this.LastcolorFrame = Colorframe;

                          
                            this.currentColorImageFormat = Colorframe.Format;

                            
                            Colorframe.CopyPixelDataTo(this._ColorImagePixelData);
                            
                            this.textBlock1.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                textBlock1.Text = string.Format("ColorFrameNumber (CFN): {0} ", Colorframe.FrameNumber); 
                            }));
                           
                           
                            CoordinateMapper mapper = _Kinect.CoordinateMapper;
                            
                            mapper.MapColorFrameToDepthFrame(this.currentColorImageFormat, this.currentDepthImageFormat, this.depthPixels, this.depthPoints);//show depth at a color point:metodo che riempie la matrice depth points
                            mapper.MapDepthFrameToColorFrame(this.currentDepthImageFormat, this.depthPixels, this.currentColorImageFormat, this.colorPoints);
                          

                            // Paint Distance on the color image if the checkbox is checked
                            this.checkBox1.Dispatcher.BeginInvoke(
                                                                  new Action(() =>
                            {
                                if (checkBox1.IsChecked == true)
                                {
                                    PaintDistance();
                                }
                            }));

                            this.ColorImageElement.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                this._ColorImageBitmap.WritePixels(this._ColorImageBitmapRect, this._ColorImagePixelData, this._ColorImageStride, 0);
                            }));

                            
                            this.checkBox2.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                if (checkBox2.IsChecked == true)
                                {
                                    ComputeDistance();
                                }
                            }));
                            
                            
                            this.textBlock4.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                      textBlock4.Text = string.Format("Elevation angle = {0}°", this._Kinect.ElevationAngle);
                            }));

                            
                            Accelerometro = new Vector4();
                            Accelerometro =this._Kinect.AccelerometerGetCurrentReading();
                            this.textBlock8.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                textBlock8.Text = string.Format("Dati accelerometro:\nx = {0} g\ny = {1} g\nz = {2} g", Math.Round(Accelerometro.X,2), Math.Round(Accelerometro.Y,2), Math.Round(Accelerometro.Z,2));
                            }));

                            this.checkBox3.Dispatcher.BeginInvoke(new Action(() =>
                            {
                                if (checkBox3.IsChecked == true)
                                {
                                    AcquisisciAccelerometro();
                                }
                            }));

         }
       }

        
        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            // get the raw data from the kinect with the depth for every pixel: old method
            //short[] RawDepthData = new short[depthFrame.PixelDataLength];
            //depthFrame.CopyPixelDataTo(RawDepthData);//dati da depthFrame in rawDepthData

            // Get the min and max reliable depth for the current frame
            int minDepth = depthFrame.MinDepth;
            int maxDepth = depthFrame.MaxDepth;

            // use depthFrame to create the image to display on screen
            // depthFrame contains color information for all pixels in image
            // Heigth*Width*4 (Red,Green,Blue,Empty byte)
            byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];

            // Bgr32 Blue, Green, Red, Empty byte
            // Bgra32 Blue, Green, Red, Transparency
            // You must set transparency for Bgra as.Net defaults a byte to 0 =fully transparent

            // hardcoded locations to Blue, Green, Red (BGR) index positions (for empty we don't write anything)
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            //loop through all distances
            for (int depthIndex = 0, colorIndex = 0; depthIndex < this.depthPixels.Length && colorIndex < pixels.Length; depthIndex++, colorIndex = colorIndex + 4)
            {
                // get the player (requires skeleton tracking enabled for values)
                //int player = RawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;
                int player = this.depthPixels[depthIndex].PlayerIndex;

                // get the depth values 
                //int depth = RawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                int depth = this.depthPixels[depthIndex].Depth;

                // depth < 0.9 metres = 900 mm
                if (depth <= 900)
                {
                    // we are very close: blu
                    pixels[colorIndex + BlueIndex] = 255;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;
                }

                // 0.9 metres = 900 mm < depth < 2 metres = 2000 mm
                else if (depth >= 900 && depth <= 2000)
                {
                    // we are a bit further away:green
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 255;
                    pixels[colorIndex + RedIndex] = 0;
                }

                // depth > 2 metres = 2000 mm
                else if (depth >= 2000)
                {
                    // we are the farthest
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 255;
                }

                // equal coloring for monochromatic histogram
                byte intensity = CalculateIntensityFromDepthData(depth, minDepth, maxDepth);
                pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;//*/

            }

            return pixels;

        }

        public static byte CalculateIntensityFromDepthData(int distance, int minDepthdistance, int maxDepthdistance)
        {
            return (byte)(255 - (255 * Math.Max(distance - minDepthdistance, 0) / (gray)));
        }//*/
        
        //Method that controls the Augmented Reality
        private void PaintDistance()
        {
            Parallel.For(0, this.LastcolorFrame.Width * this.LastcolorFrame.Height, ((int colorPixelIndex) =>
            {
               

                DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
                int depthInMM = depthPoint.Depth;
                int y = depthPoint.Y; 
                int x = depthPoint.X;

                //It calculates the pixel dimension in mm and the point coordinates
                //Axis x (pixel) ---> Axis X(mm)
                // (x = width/2 --> X=0)
                const double ALFA = 58.5;
                double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; 
                double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // coordinata X in mm


                //Axis y (pixel) ---> Axis Y(mm)
                // y = heigth/2 --> Y=0          
                const double BETA = 45.6; 
                double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //dimensione del pixel lungo y
                double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // coordinata Y in mm

                double a = Math.Abs(X) % 250; 

                int b = ((int)Math.Truncate(Math.Abs(X) / 250.0)) % 2; 

                if (a > 125 && a < 175) 
                {
                    int ByteIndex = colorPixelIndex * this.LastcolorFrame.BytesPerPixel;

                    if (b == 0) 
                    {
                        this._ColorImagePixelData[ByteIndex] = 100;
                        this._ColorImagePixelData[ByteIndex + 1] = 0;
                        this._ColorImagePixelData[ByteIndex + 2] = 0;
                    }
                    else
                    {
                        this._ColorImagePixelData[ByteIndex] = 0;
                        this._ColorImagePixelData[ByteIndex + 1] = 0;
                        this._ColorImagePixelData[ByteIndex + 2] = 100;
                    }
                }
            }));

        }
         
        private void ColorImageElement_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            contcol++;
            Point p = e.GetPosition(ColorImageElement);
            int colorPixelIndex = (int)(p.X + ((int)p.Y * this.LastcolorFrame.Width));
            
            DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
            int depthInMM = depthPoint.Depth;
            int y = depthPoint.Y;
            int x = depthPoint.X;

            //It calculates the pixel dimension in mm and the point coordinates
            //Axis x (pixel) ---> Axis X(mm)
            // (x = width/2 --> X=0)
            const double ALFA = 58.5; 
            double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension in mm
            double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; //X in mm
            
            //Axis y (pixel) ---> Axis Y(mm)
            // y = heigth/2 --> Y=0            
            const double BETA = 45.6; // angolo di vista verticale nominale della DEPTH CAMERA
            double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //dimensione del pixel lungo y
            double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // coordinata Y in mm 

            textBlock5.Text = string.Format("Coordinate dei pixel dell'immagine RGB\nFrame number n°{8}:\npunto cliccato n°{0} \n RGB: x = {1}\n         y = {2} \nDEPTH: x ={3}\n             y = {4}  \n d={5} mm \nX={6} mm \nY={7} mm ", contcol, p.X, p.Y, depthPoint.X, depthPoint.Y, depthPoint.Depth, Math.Round(X, 3), Math.Round(Y, 3), this.LastcolorFrame.FrameNumber);
            this.sw3.WriteLine("{0} {1}  {2}  {3}  {4}  {5}  {6}  {7}  {8}  {9}  {10}",this.LastcolorFrame.FrameNumber, this._Kinect.ElevationAngle, contcol, p.X, p.Y, depthPoint.Depth, X, Y, Accelerometro.X, Accelerometro.Y, Accelerometro.Z);

          
            el[contcol] = new Ellipse();
            el[contcol].Stroke = System.Windows.Media.Brushes.Black;
            el[contcol].Fill = System.Windows.Media.Brushes.Red;
            el[contcol].Width = 8;
            el[contcol].Height = 8;
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

           
            el3[contcol] = new Ellipse();
            el3[contcol].Stroke = System.Windows.Media.Brushes.Black;
            el3[contcol].Fill = System.Windows.Media.Brushes.Yellow;
            el3[contcol].Width = 8;
            el3[contcol].Height = 8;
            el3[contcol].VerticalAlignment = VerticalAlignment.Top;
            el3[contcol].HorizontalAlignment = HorizontalAlignment.Left;
            el3[contcol].Margin = new Thickness(depthPoint.X, depthPoint.Y, 0, 0);
            el3[contcol].MouseRightButtonDown += new MouseButtonEventHandler(el3_MouseRightButtonDown);
            el3[contcol].MouseRightButtonUp += new MouseButtonEventHandler(el3_MouseRightButtonUp);

           
            TextBlock txt3 = new TextBlock();
            txt3.Text = string.Format("{0}", contcol);
            txt3.FontSize = 11;
            txt3.Foreground = Brushes.Yellow;
            txt3.Margin = new Thickness(depthPoint.X + 1, depthPoint.Y + 6, 0, 0);

       
            canvas1.Children.Add(el[contcol]);
            canvas1.Children.Add(txt[contcol]);
            canvas2.Children.Add(el3[contcol]);
            canvas2.Children.Add(txt3);
        }

    
        private void el_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.codePopup.IsOpen = false;
            codePopup.IsOpen = false;
        }

        
        private void el_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Ellipse current = sender as Ellipse;

            int colorPixelIndex = (int)(current.Margin.Left + ((int)current.Margin.Top * this.LastcolorFrame.Width));

            DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
            int depthInMM = depthPoint.Depth;
            int y = depthPoint.Y;
            int x = depthPoint.X;

            //It calculates the pixel dimension in mm and the point coordinates
            //Axis x (pixel) ---> Axis X(mm)
            // (x = width/2 --> X=0)
            const double ALFA = 58.5;
            double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension in mm
            double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x;


            //Axis y (pixel) ---> Axis Y(mm)
            // y = heigth/2 --> Y=0               
            const double BETA = 45.6; 
            double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height;//pixel dimension in mm
            double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // coordinata Y in mm 

            
            this.codePopup = new Popup();
            this.codePopup.IsOpen = true;
            this.codePopup.PlacementTarget = current;
            TextBlock popupText = new TextBlock();
            popupText.Background = Brushes.LightBlue;
            popupText.Foreground = Brushes.Blue;
            popupText.Text = string.Format("DFN n°{6}: x={1} y={2} X={3}mm Y={4}mm d={5}mm", contcol, current.Margin.Left, current.Margin.Top, Math.Round(X, 2), Math.Round(Y, 2), depthInMM, this.LastdepthFrame.FrameNumber);
            this.codePopup.Child = popupText;
        }

       
        private void el3_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            //It retrieves the ellipses data
            Ellipse current = sender as Ellipse;
            int pixelIndex = (int)(current.Margin.Left + ((int)current.Margin.Top * this.LastdepthFrame.Width));
            int y = pixelIndex / this.LastdepthFrame.Width;
            int x = pixelIndex - y * this.LastdepthFrame.Width;
            int depth = this.depthPixels[pixelIndex].Depth;// distanza estesa: introdotta con l'SDK 1.6: arriva oltre i 4m
            
            const double ALFA = 58.5;
            double dp_x = (2 * depth * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension in mm
            double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; //  X in mm 
            const double BETA = 45.6; 
            double dp_y = (2 * depth * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //pixel dimension in mm
            double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // Y in mm 
          
            this.codePopup3 = new Popup();
            this.codePopup3.IsOpen = true;
            this.codePopup3.PlacementTarget = current;
            TextBlock popupText = new TextBlock();
            popupText.Background = Brushes.LightBlue;
            popupText.Foreground = Brushes.Blue;
            popupText.Text = string.Format("DFN n°{6}: x={1} y={2} X={3}mm Y={4}mm d={5}mm", contdepth, current.Margin.Left, current.Margin.Top, Math.Round(X, 2), Math.Round(Y, 2), depth, this.LastcolorFrame.FrameNumber);
            this.codePopup3.Child = popupText;
        }

        
        private void el3_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.codePopup3.IsOpen = false;
            codePopup3.IsOpen = false;
        }

        private void DepthImageElement_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            contdepth++;
            Point p = e.GetPosition(DepthImageElement);
            int pixelIndex = (int)(p.X + ((int)p.Y * this.LastdepthFrame.Width));
            int depth = this.depthPixels[pixelIndex].Depth;
            const double ALFA = 58.5; 
            double dp_x = (2 * depth * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; 
            double X = (p.X - (this.LastdepthFrame.Width / 2)) * dp_x;
            const double BETA = 45.6;
            double dp_y = (2 * depth * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; 
            double Y = (-p.Y + (this.LastdepthFrame.Height / 2)) * dp_y; 
            ColorImagePoint colorPoint = this.colorPoints[pixelIndex];
            textBlock2.Text = string.Format("Frame number n°{8}:\npunto cliccato n°{0} \n x={1}   y={2}  d={3} mm \n X={4} mm \n Y={5} mm\n colorPoint x={6} \n colorPoint y={7}", contdepth, p.X, p.Y, depth, Math.Round(X, 3), Math.Round(Y, 3), colorPoint.X, colorPoint.Y, this.LastdepthFrame.FrameNumber);
            this.sw2.WriteLine("{0}  {1}  {2}  {3}  {4}  {5}  {6}  {7}  {8}  {9}  {10} {11}   {12}", this.LastdepthFrame.FrameNumber, this._Kinect.ElevationAngle, contdepth, p.X, p.Y, X, Y, depth, colorPoint.X, colorPoint.Y, Accelerometro.X, Accelerometro.Y,Accelerometro.Z);

         
            el2[contdepth] = new Ellipse();
            el2[contdepth].Stroke = System.Windows.Media.Brushes.Black;
            el2[contdepth].Fill = System.Windows.Media.Brushes.YellowGreen;
            el2[contdepth].Width = 8;
            el2[contdepth].Height = 8;
            el2[contdepth].Margin = new Thickness(p.X, p.Y, 0, 0);
            el2[contdepth].MouseRightButtonDown += new MouseButtonEventHandler(el2_MouseRightButtonDown);
            el2[contdepth].MouseRightButtonUp += new MouseButtonEventHandler(el2_MouseRightButtonUp);

            
            TextBlock txt = new TextBlock();
            txt.Text = string.Format("{0}", contdepth);
            txt.FontSize = 11;
            txt.Foreground = Brushes.YellowGreen;
            txt.Margin = new Thickness(p.X + 1, p.Y + 6, 0, 0);

        
            canvas2.Children.Add(el2[contdepth]);
            canvas2.Children.Add(txt);
        }

      
        private void el2_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            Ellipse current = sender as Ellipse;

            short[] MouseDepthData = new short[this.LastdepthFrame.PixelDataLength];
            this.LastdepthFrame.CopyPixelDataTo(MouseDepthData);

            int pixelIndex = (int)(current.Margin.Left + ((int)current.Margin.Top * this.LastdepthFrame.Width));
            int y = pixelIndex / this.LastdepthFrame.Width;
            int x = pixelIndex - y * this.LastdepthFrame.Width;
            int d = MouseDepthData[pixelIndex] >> 3;
            int depth = this.depthPixels[pixelIndex].Depth;

            const double ALFA = 58.5; 
            double dp_x = (2 * depth * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; 
            double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; 
            const double BETA = 45.6; 
            double dp_y = (2 * depth * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; 
            double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; 

            this.codePopup2 = new Popup();
            this.codePopup2.IsOpen = true;
            this.codePopup2.PlacementTarget = current;
            TextBlock popupText = new TextBlock();
            popupText.Background = Brushes.LightBlue;
            popupText.Foreground = Brushes.Blue;
            popupText.Text = string.Format("DFN n°{6}: x={1} y={2} X={3}mm Y={4}mm d_SDK1.6={5}mm d_old={7}mm", contdepth, current.Margin.Left, current.Margin.Top, Math.Round(X, 2), Math.Round(Y, 2), depth, this.LastcolorFrame.FrameNumber, d);
            this.codePopup2.Child = popupText;
        }

        
        private void el2_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            this.codePopup2.IsOpen = false;
            codePopup2.IsOpen = false;
        }

       
        private void Window_Closing(object sender, CancelEventArgs e)
        {
            if (this._Kinect == null)
                {
                    this._Kinect.ColorStream.Disable();
                    this._Kinect.DepthStream.Disable();
                    this._Kinect.SkeletonStream.Disable();
                    this._Kinect.Stop();
                    this._Kinect.AudioSource.Stop();
                }
            sw.Close();
            sw2.Close();
            sw3.Close();
            //sw4.Close();
            sw5.Close();
            sw6.Close();
            sw1.Close();
            sw7.Close();
            sw8.Close();
            sw9.Close();
        }

        
        private void ColorImageElement_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            Point p = e.GetPosition(ColorImageElement);
            
            TransformGroup transformgroup = (TransformGroup)ColorImageElement.RenderTransform;
            ScaleTransform transform = (ScaleTransform)transformgroup.Children[0];
           
            transform.CenterX = p.X;
            transform.CenterY = p.Y;

            double zoom = e.Delta > 0 ? .2 : -.2;
            transform.ScaleX += zoom;
            transform.ScaleY += zoom;
        }

        
        private void ColorImageElement_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            ColorImageElement.CaptureMouse();
            var tt = (TranslateTransform)((TransformGroup)ColorImageElement.RenderTransform).Children.First(tr => tr is TranslateTransform);
            start = e.GetPosition(Border);
            origin = new Point(tt.X, tt.Y);
        }

       
        private void ColorImageElement_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            ColorImageElement.ReleaseMouseCapture(); 
        }

       
        private void ColorImageElement_MouseMove(object sender, MouseEventArgs e)
        {
            if (!floatingTip.IsOpen) { floatingTip.IsOpen = true; }

            Point currentPos = e.GetPosition(ColorImageElement);

            // The + 10 part is so your mouse pointer doesn't overlap.
            floatingTip.HorizontalOffset = currentPos.X + 10;
            floatingTip.VerticalOffset = currentPos.Y;
            int colorPixelIndex = (int)(currentPos.X + ((int)currentPos.Y * this.LastcolorFrame.Width));

            DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
            int depthInMM = depthPoint.Depth;
            int y = depthPoint.Y;
            int x = depthPoint.X;

            //It calculates the pixel dimension in mm and the point coordinates
            //Axis x (pixel) ---> Axis X(mm)
            // (x = width/2 --> X=0)
            const double ALFA = 58.5; 
            double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension in mm
            double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // X in mm 
           
            //Asse y (pixel) ---> Asse Y(mm)
            // y = heigth/2 --> Y=0           
            const double BETA = 45.6; 
            double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //pixel dimension in mm
            double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; //Y in mm 

            
            textBlock6.Text = string.Format("x={0}   y={1} \n d={2} mm \n X={3} mm \n Y={4} mm ", Math.Round(currentPos.X, 2), Math.Round(currentPos.Y, 2), depthPoint.Depth, Math.Round(X, 3), Math.Round(Y, 3));// è meglio arrotondare anzichè mettere int

            
            if (!ColorImageElement.IsMouseCaptured) return;
            var tt = (TranslateTransform)((TransformGroup)ColorImageElement.RenderTransform).Children.First(tr => tr is TranslateTransform);
            Vector v = start - e.GetPosition(Border);
            tt.X = origin.X - v.X;
            tt.Y = origin.Y - v.Y;
        }

        
        private void floatingTip_Opened(object sender, EventArgs e)
        {
           
        }

        
        private void ColorImageElement_MouseLeave(object sender, MouseEventArgs e)
        {
            floatingTip.IsOpen = false;
        }

        
        private void DepthImageElement_MouseMove(object sender, MouseEventArgs e)
        {
            if (!floatingTip2.IsOpen) { floatingTip2.IsOpen = true; }

            Point currentPos = e.GetPosition(DepthImageElement);

            // The + 10 part is so your mouse pointer doesn't overlap.
            floatingTip2.HorizontalOffset = currentPos.X + 10;
            floatingTip2.VerticalOffset = currentPos.Y;

            int pixelIndex = (int)(currentPos.X + ((int)currentPos.Y * this.LastdepthFrame.Width)); 
            int dist = this.depthPixels[pixelIndex].Depth;

            const double ALFA = 58.5; 
            double dp_x = (2 * dist * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; 
            double X = (currentPos.X - (this.LastdepthFrame.Width / 2)) * dp_x; 
            const double BETA = 45.6; 
            double dp_y = (2 * dist * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; 
            double Y = (-currentPos.Y + (this.LastdepthFrame.Height / 2)) * dp_y; 
            textBlock7.Text = string.Format("x={0}   y={1} \n d={5} mm\n X={3} mm \n Y={4} mm ", (int)(currentPos.X), (int)(currentPos.Y), X, Math.Round(X, 2), Math.Round(Y, 2), dist); 
        }

        
        private void floatingTip2_Opened(object sender, EventArgs e)
        {
           
        }

        
        private void DepthImageElement_MouseLeave(object sender, MouseEventArgs e)
        {
            floatingTip2.IsOpen = false;
        }

        
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            
            this.sw.WriteLine("Frame number:{0}; elevation angle: {1}°;accelerometro: x={2}g  y={3}g  z={4}g ", this.LastdepthFrame.FrameNumber, this._Kinect.ElevationAngle,this.Accelerometro.X,this.Accelerometro.Y,this.Accelerometro.Z);
            this.sw.WriteLine("x   y          X             Y               d");
          
            for (int depthpixelindex = 0; depthpixelindex < this.LastdepthFrame.Width * this.LastdepthFrame.Height; depthpixelindex++)
            {
                int y = depthpixelindex / this.LastdepthFrame.Width;
                int x = depthpixelindex - y * this.LastdepthFrame.Width;

                int dist = this.depthPixels[depthpixelindex].Depth;

                // Get the min and max reliable depth for the current frame
                int min_d = LastdepthFrame.MinDepth;
                int max_d = LastdepthFrame.MaxDepth;


               //It calculates the pixel dimension in mm and the point coordinates
               //Axis x (pixel) ---> Axis X(mm)
               // (x = width/2 --> X=0)
                const double ALFA = 58.5; 
                double dp_x = (2 * dist * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; 
                double X = (x - (this.LastdepthFrame.Width / 2)) * dp_x; 
                
                // y = heigth/2 --> Y=0         
                const double BETA = 45.6; 
                double dp_y = (2 * dist * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; 
                double Y = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; 

                this.sw.WriteLine("{0}  {1}  {2}  {3}  {4}", x, y, X, Y, dist);
            }
        }

        //Calcola la distanza tra i punti cliccati (verdi) sull'immagine di profondità
        private void button2_Click(object sender, RoutedEventArgs e)
        {
            cliccatadepth++;
           
            double[] X = new double[50];
            double[] Y = new double[50];
            int[] Z = new int[50];
            int[] DFN = new int[50];

            this.sw5.WriteLine("Cliccata n°{0}\n", cliccatadepth);
            this.sw5.WriteLine("DFN n°{0}", this.LastdepthFrame.FrameNumber);
            this.sw5.WriteLine("Elevation Angle: {0}°", this._Kinect.ElevationAngle);
            this.sw5.WriteLine("Accelerometro: x={0} g; y={1} g; z ={2}", this.Accelerometro.X, this.Accelerometro.Y, this.Accelerometro.Z);

            //This cicle for ascribes ist own distance to every ellipse, and, as a consequence, the X and Y coordinates expressed in mm 
            for (int i = 1; i <= contdepth; i++)
            {
                DFN[i] = this.LastdepthFrame.FrameNumber;
                int pixelIndex = (int)(el2[i].Margin.Left + ((int)el2[i].Margin.Top * this.LastdepthFrame.Width));
                int y = pixelIndex / this.LastdepthFrame.Width;
                int x = pixelIndex - y * this.LastdepthFrame.Width;
                int depth = this.depthPixels[pixelIndex].Depth;// distanza estesa: introdotta con l'SDK 1.6: arriva oltre i 4m

                const double ALFA = 58.5; // angolo di vista orizzontale nominale della DEPTH CAMERA: const equivale al define in c++
                double dp_x = (2 * depth * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //dimensione del pixel lungo x
                double XMM = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // coordinata X in mm 
                const double BETA = 45.6; // angolo di vista verticale nominale della DEPTH CAMERA
                double dp_y = (2 * depth * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //dimensione del pixel lungo y
                double YMM = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // coordinata Y in mm 

                X[i] = XMM;
                Y[i] = YMM;
                Z[i] = depth;
            }

            // It calculates the distance between all the green ellipses
            for (int i = 1; i <= contdepth; i++)
            {
                for (int j = i + 1; j <= contdepth; j++)// j=i+1 per evitare la ripetizione delle distanze simmetriche
                {
                    double dist = Math.Sqrt((X[j] - X[i]) * (X[j] - X[i]) + (Y[j] - Y[i]) * (Y[j] - Y[i]) + (Z[j] - Z[i]) * (Z[j] - Z[i]));
                    MessageBox.Show(String.Format("Cliccata n°{11}\n\npunto cliccato n°{0} (DFN n°{9}) \n X{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\npunto cliccato n°{4} (DFN n°{10})\n X{4}={5}mm Y{4}={6}mm Z{4}={7}mm \n\nd{0},{4}={8}mm ", i, Math.Round(X[i], 2), Math.Round(Y[i], 2), Z[i], j, Math.Round(X[j], 2), Math.Round(Y[j], 2), Z[j], Math.Round(dist, 2), DFN[i], DFN[j], cliccatadepth));
                    this.sw5.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm d{0}_{4}={8}mm ", i, X[i], Y[i], Z[i], j, X[j], Y[j], Z[j], dist, DFN[i], DFN[j]);
                }
            }
        }

        
        private void button3_Click(object sender, RoutedEventArgs e)
        {
            cliccatagialla++;
            
            double[] X = new double[50];
            double[] Y = new double[50];
            int[] Z = new int[50];
            int[] DFN = new int[50];

            this.sw6.WriteLine("Cliccata n°{0}\n", cliccatagialla);
            this.sw6.WriteLine("DFN n°{0}, Elevation angle: {1}°", this.LastdepthFrame.FrameNumber, this._Kinect.ElevationAngle);
            this.sw6.WriteLine("Accelerometro: x={0} g; y={1} g; z ={2}", this.Accelerometro.X, this.Accelerometro.Y, this.Accelerometro.Z);

            //This cicle for ascribes ist own distance to every ellipse, and, as a consequence, the X and Y coordinates expressed in mm 
            for (int i = 1; i <= contcol; i++)
            {
                DFN[i] = this.LastdepthFrame.FrameNumber;
                int pixelIndex = (int)(el3[i].Margin.Left + ((int)el3[i].Margin.Top * this.LastdepthFrame.Width));

                int y = pixelIndex / this.LastdepthFrame.Width;
                int x = pixelIndex - y * this.LastdepthFrame.Width;
                int depth = this.depthPixels[pixelIndex].Depth;// distanza estesa: introdotta con l'SDK 1.6: arriva oltre i 4m.

                const double ALFA = 58.5;
                double dp_x = (2 * depth * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension along X axis
                double XMM = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // X in mm 
                const double BETA = 45.6;
                double dp_y = (2 * depth * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //pixel dimension along Y axis
                double YMM = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; // Y in mm 

                X[i] = XMM;
                Y[i] = YMM;
                Z[i] = depth;
            }

            //// Esegue il calcolo della distanza tra tulle le varie ellissi(tuttte le possibili combinazioni)
            //for (int i = 1; i <= contcol; i++)
            //{
            //    for (int j = i + 1; j <= contcol; j++)// j=i+1 per evitare la ripetizione delle distanze simmetriche
            //    {
            //        double dist = Math.Sqrt((X[j] - X[i]) * (X[j] - X[i]) + (Y[j] - Y[i]) * (Y[j] - Y[i]) + (Z[j] - Z[i]) * (Z[j] - Z[i]));
            //        MessageBox.Show(String.Format("Cliccata n°{11}\n\npunto cliccato n°{0} (DFN n°{9}) \n X{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\npunto cliccato n°{4} (DFN n°{10})\n X{4}={5}mm Y{4}={6}mm Z{4}={7}mm \n\nd{0},{4}={8}mm ", i, Math.Round(X[i], 2), Math.Round(Y[i], 2), Z[i], j, Math.Round(X[j], 2), Math.Round(Y[j], 2), Z[j], Math.Round(dist, 2), DFN[i], DFN[j], cliccatagialla));
            //        this.sw6.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm d{0}_{4}={8}mm ", i, X[i], Y[i], Z[i], j, X[j], Y[j], Z[j], dist, DFN[i], DFN[j]);
            //    }
            //}

            //Horizontal distances
            for (int j = 0; j <= 2; j++)
            {
                for (int i = 1; i <= 3; i++)
                {
                    double dist = Math.Sqrt((X[4 * j + i + 1] - X[4 * j + i]) * (X[4 * j + i + 1] - X[4 * j + i]) + (Y[4 * j + i + 1] - Y[4 * j + i]) * (Y[4 * j + i + 1] - Y[4 * j + i]) + (Z[4 * j + i + 1] - Z[4 * j + i]) * (Z[4 * j + i + 1] - Z[4 * j + i]));
                    this.sw6.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm  d{4}_{0}={8}mm d{4}_{0}={11}cm", 4 * j + i + 1, X[4 * j + i + 1], Y[4 * j + i + 1], Z[4 * j + i + 1], 4 * j + i, X[4 * j + i], Y[4 * j + i], Z[4 * j + i], dist, DFN[4 * j + i + 1], DFN[4 * j + i], dist / 10);
                    MessageBox.Show(String.Format("punto cliccato n°{0} (DFN n°{9}) \nX{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\npunto cliccato n°{4} (DFN n°{10}) \nX{4}={5}mm Y{4}={6}mm Z{4}={7}mm\n\n d{4}_{0}={8}mm\n d{4}_{0}={11}cm", 4 * j + i + 1, X[4 * j + i + 1], Y[4 * j + i + 1], Z[4 * j + i + 1], 4 * j + i, X[4 * j + i], Y[4 * j + i], Z[4 * j + i], dist, DFN[4 * j + i + 1], DFN[4 * j + i], Math.Round(dist / 10, 2)));
                }
            }

            // Vertical distances
            for (int j = 0; j <= 1; j++)
            {
                for (int i = 1; i <= 4; i++)
                {
                    double dist = Math.Sqrt((X[4 * j + i + 4] - X[4 * j + i]) * (X[4 * j + i + 4] - X[4 * j + i]) + (Y[4 * j + i + 4] - Y[4 * j + i]) * (Y[4 * j + i + 4] - Y[4 * j + i]) + (Z[4 * j + i + 4] - Z[4 * j + i]) * (Z[4 * j + i + 4] - Z[4 * j + i]));           
                    this.sw6.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm d{4}_{0}={8}mm d{4}_{0}={11}cm", 4 * j + i + 4, X[4 * j + i + 4], Y[4 * j + i + 4], Z[4 * j + i + 4], 4 * j + i, X[4 * j + i], Y[4 * j + i], Z[4 * j + i], dist, DFN[4 * j + i + 4], DFN[4 * j + i], Math.Round(dist / 10, 2));
                    MessageBox.Show(String.Format("punto cliccato n°{0} (DFN n°{9}) \nX{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\npunto cliccato n°{4} (DFN n°{10}) \nX{4}={5}mm Y{4}={6}mm Z{4}={7}mm\n\n d{4}_{0}={8}mm\n d{4}_{0}={11}cm", 4 * j + i + 4, X[4 * j + i + 4], Y[4 * j + i + 4], Z[4 * j + i + 4], 4 * j + i, X[4 * j + i], Y[4 * j + i], Z[4 * j + i], dist, DFN[4 * j + i + 4], DFN[4 * j + i], Math.Round(dist / 10, 2)));
                }
            }

            //Diagonals
            for (int j = 0; j <= 1; j++)
            {
                for (int i = 1; i <= 3; i++)
                {
                    double dist_diagonali_1 = Math.Sqrt((X[4 * j + i + 5] - X[4 * j + i]) * (X[4 * j + i + 5] - X[4 * j + i]) + (Y[4 * j + i + 5] - Y[4 * j + i]) * (Y[4 * j + i + 5] - Y[4 * j + i]) + (Z[4 * j + i + 5] - Z[4 * j + i]) * (Z[4 * j + i + 5] - Z[4 * j + i]));
                    this.sw6.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm d{4}_{0}={8}mm d{4}_{0}={11}cm", 4 * j + i + 5, X[4 * j + i + 5], Y[4 * j + i + 5], Z[4 * j + i + 5], 4 * j + i, X[4 * j + i], Y[4 * j + i], Z[4 * j + i], dist_diagonali_1, DFN[4 * j + i + 5], DFN[4 * j + i], Math.Round(dist_diagonali_1 / 10, 2));
                    double dist_diagonali_2 = Math.Sqrt((X[4 * j + i + 1] - X[4 * j + i + 1 + 3]) * (X[4 * j + i + 1] - X[4 * j + i + 1 + 3]) + (Y[4 * j + i + 1] - Y[4 * j + i + 1 + 3]) * (Y[4 * j + i + 1] - Y[4 * j + i + 1 + 3]) + (Z[4 * j + i + 1] - Z[4 * j + i]) * (Z[4 * j + i + 1] - Z[4 * j + i + 1 + 3]));
                    this.sw6.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm d{4}_{0}={8}mm d{4}_{0}={11}cm", 4 * j + i + 1, X[4 * j + i + 1], Y[4 * j + i + 1], Z[4 * j + i + 1], 4 * j + i + 1 + 3, X[4 * j + i + 1 + 3], Y[4 * j + i + 1 + 3], Z[4 * j + i + 1 + 3], dist_diagonali_2, DFN[4 * j + i + 1], DFN[4 * j + i + 1 + 3], Math.Round(dist_diagonali_2 / 10, 2));
                    MessageBox.Show(String.Format("punto cliccato n°{0} (DFN n°{9}) \nX{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\npunto cliccato n°{4} (DFN n°{10}) \nX{4}={5}mm Y{4}={6}mm Z{4}={7}mm\n\n d{4}_{0}={8}mm\n d{4}_{0}={11}cm", 4 * j + i + 5, X[4 * j + i + 5], Y[4 * j + i + 5], Z[4 * j + i + 5], 4 * j + i, X[4 * j + i], Y[4 * j + i], Z[4 * j + i], dist_diagonali_1, DFN[4 * j + i + 5], DFN[4 * j + i], Math.Round(dist_diagonali_1 / 10, 2)));
                    MessageBox.Show(String.Format("punto cliccato n°{0} (DFN n°{9}) \nX{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\npunto cliccato n°{4} (DFN n°{10}) \nX{4}={5}mm Y{4}={6}mm Z{4}={7}mm\n\n d{4}_{0}={8}mm\n d{4}_{0}={11}cm", 4 * j + i + 1, X[4 * j + i + 1], Y[4 * j + i + 1], Z[4 * j + i + 1], 4 * j + i + 1 + 3, X[4 * j + i + 1 + 3], Y[4 * j + i + 1 + 3], Z[4 * j + i + 1 + 3], dist_diagonali_2, DFN[4 * j + i + 1], DFN[4 * j + i + 1 + 3], Math.Round(dist_diagonali_2 / 10, 2)));
                }
            }

            ////External diagonals :  Rad(13)*34.5=124.39cm
            //double dist_1_12 = Math.Sqrt((X[1] - X[12]) * (X[1] - X[12]) + (Y[1] - Y[12]) * (Y[1] - Y[12]) + (Z[1] - Z[12]) * (Z[1] - Z[12]));
            //double dist_4_9 = Math.Sqrt((X[4] - X[9]) * (X[4] - X[9]) + (Y[4] - Y[9]) * (Y[4] - Y[9]) + (Z[4] - Z[9]) * (Z[4] - Z[9]));
            //this.sw6.WriteLine("{0:#####.0000} {1:#####.0000} ", dist_1_12, dist_4_9);
        }

        //Delete the ellipses
        private void button4_Click(object sender, RoutedEventArgs e)
        {
            //collimazione++;
            for (int i = 1; i <= contcol; i++)
            {
                canvas1.Children.Remove(el[i]);
                canvas1.Children.Remove(txt[i]);
            }
            //this.sw4.WriteLine("Ellisssi cambiate: nuova collimazione");
            this.sw1.WriteLine("Ellisssi cambiate: nuova collimazione");
            canvas2.Children.Clear();
            contcol = 0;
            contdepth = 0;
            acquisizione = 0;
        }
        
        //Changes gray levels of the depth map
        private void button5_Click(object sender, RoutedEventArgs e)
        {
            gray = Convert.ToInt32(textBox1.Text);
        }
        
        //Changes the elevation angle of the Kinect
        private void slider1_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
  
            if (_Kinect != null && _Kinect.IsRunning)
            {
                _Kinect.ElevationAngle = (int)slider1.Value;
            }
            System.Threading.Thread.Sleep(new TimeSpan(hours: 0, minutes: 0, seconds: 1));
        }

      
        private void ComputeDistance()
        {
            if (this.LastdepthFrame.FrameNumber - CDFN > 19)//la differenza tra il frame iniziale e il frame attuale
            {
                CDFN = this.LastdepthFrame.FrameNumber;

                acquisizione++;
                double[] X = new double[colonne_griglia_calibrazione*righe_griglia_calibrazione+1];
                double[] Y = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
                double[] Z = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];

                int[] DFN = new int[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
                
                
                StringBuilder sb = new StringBuilder();
                sb.Append(acquisizione);
                sb.Append("\t"); //Add tabulation
                sb.Append(CDFN);
                sb.Append("\t");
                sb.Append(this._Kinect.ElevationAngle);
                sb.Append("\t");
                sb.Append(Math.Round(this.Accelerometro.X, 3));
                sb.Append("\t");
                sb.Append(Math.Round(this.Accelerometro.Y, 3));
                sb.Append("\t");
                sb.Append(Math.Round(this.Accelerometro.Z, 3));
                sb.Append("\t");


                //this.sw4.Write("{0:####.0} {1} {2} ", acquisizione, CDFN, this._Kinect.ElevationAngle);
                //this.sw4.Write(sb.ToString());
                this.sw1.Write(sb.ToString());

                //This cicle for ascribes ist own distance to every ellipse, and, as a consequence, the X and Y coordinates expressed in mm 
                for (int i = 1; i <= contcol; i++)
                {
                    DFN[i] = this.LastdepthFrame.FrameNumber;
                    int colorPixelIndex = (int)(el[i].Margin.Left + ((int)el[i].Margin.Top * this.LastcolorFrame.Width));

                    DepthImagePoint depthPoint = this.depthPoints[colorPixelIndex];
                    int depthInMM = depthPoint.Depth;
                    int y = depthPoint.Y;
                    int x = depthPoint.X;

                    //It calculates the pixel dimension in mm and the coordinates of the selected points

                    //Axis x (pixel) ---> Axis X(mm)
                    // (x = width/2 --> X=0)
                    const double ALFA = 58.5; 
                    double dp_x = (2 * depthInMM * Math.Tan(ALFA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Width; //pixel dimension along X axis
                    double XMM = (x - (this.LastdepthFrame.Width / 2)) * dp_x; // X in mm (origin in the center of depth image)
                  

                    //Axis y (pixel) ---> Axis Y(mm)
                    // y = heigth/2 --> Y=0     
                    const double BETA = 45.6; // angolo di vista verticale nominale della DEPTH CAMERA
                    double dp_y = (2 * depthInMM * Math.Tan(BETA * (Math.PI / 180) / 2)) / this.LastdepthFrame.Height; //pixel dimension along Y axis
                    double YMM = (-y + (this.LastdepthFrame.Height / 2)) * dp_y; //  Y in mm (origin in the center of depth image)
                    X[i] = XMM;
                    Y[i] = YMM;
                    Z[i] = (double) depthInMM;
                    this.sw1.Write("{0:#####.0000}    {1:#####.0000}    {2:#####.0000}    ", XMM, YMM, depthInMM);
                }

                this.sw1.WriteLine();

                //Per ogni acquisizione, un'osservazione del valore delle 3 coordinate del punto specifico della griglia è aggiunta al campione statistico
                punti_griglia_bisX.Add(X);
                punti_griglia_bisY.Add(Y);
                punti_griglia_bisZ.Add(Z);

                //// It calculates all the distances between all the ellipses
                //for (int i = 1; i <= contcol; i++)
                //{
                //    for (int j = i + 1; j <= contcol; j++)// j=i+1 per evitare la ripetizione delle distanze simmetriche
                //    {
                //        double dist = Math.Sqrt((X[j] - X[i]) * (X[j] - X[i]) + (Y[j] - Y[i]) * (Y[j] - Y[i]) + (Z[j] - Z[i]) * (Z[j] - Z[i]));
                //        //MessageBox.Show(String.Format("Cliccata n°{12}\n\nAcquisizione n°{11}\n\n punto cliccato n°{0} (DFN n°{9}) \n X{0}={1}mm Y{0}={2}mm Z{0}={3}mm \n\n punto cliccato n°{4} (DFN n°{10})\n X{4}={5}mm Y{4}={6}mm Z{4}={7}mm \n\n d{0},{4}={8}mm ", i, Math.Round(X[i], 2), Math.Round(Y[i], 2), Z[i], j, Math.Round(X[j], 2), Math.Round(Y[j], 2), Z[j], Math.Round(dist, 2), DFN[i], DFN[j], acquisizione, cliccata));
                //        this.sw4.WriteLine("punto cliccato n°{0} (DFN n°{9}) X{0}={1}mm Y{0}={2}mm Z{0}={3}mm punto cliccato n°{4} (DFN n°{10}) X{4}={5}mm Y{4}={6}mm Z{4}={7}mm d{0}_{4}={8}mm d{0}_{4}={11}cm", i, X[i], Y[i], Z[i], j, X[j], Y[j], Z[j], dist, DFN[i], DFN[j], Math.Round(dist / 10, 2));
                //    }
                //}

            }

            if (acquisizione == 10)// Stops the acquisition to 100 frames for single collimation
            {
                collimazione++;
                checkBox2.IsChecked = false;

                this.sw8.Write("{0}      ", collimazione);
               // for (int i = 1; i <= contcol; i++)
                for (int i = 1; i <= colonne_griglia_calibrazione * righe_griglia_calibrazione; i++)
                {  //Columna 1 --> Point 1;  Column 2 --> Point 2;  Column 3 --> Point 3 etc.
                     X_Mediane[i]=punti_griglia_bisX.Column(i).Median;
                     Y_Mediane[i] = punti_griglia_bisY.Column(i).Median;
                     Z_Mediane[i] = punti_griglia_bisZ.Column(i).Median;
                     this.sw8.Write("{0}      {1}      {2}      ",X_Mediane[i], Y_Mediane[i], Z_Mediane[i]);
                }
                this.sw8.WriteLine();

                //For a single collimation, a median is added to the median sample
                MedianePuntiGrigliaX.Add(X_Mediane);
                MedianePuntiGrigliaY.Add(Y_Mediane);
                MedianePuntiGrigliaZ.Add(Z_Mediane);

                punti_griglia_bisX.Clear();
                punti_griglia_bisY.Clear();
                punti_griglia_bisZ.Clear();

                
                for (int i = 1; i <= contcol; i++)
                {
                    canvas1.Children.Remove(el[i]);
                    canvas1.Children.Remove(txt[i]);
                }
                canvas2.Children.Clear();
                
           
                this.sw1.WriteLine();
        
                contcol = 0;
                contdepth = 0;
                acquisizione = 0;
                textBlock9.Text = string.Format("N° collimazioni acquisite: {0}", collimazione);
            }
        }

        
        private void AcquisisciAccelerometro()
        {
            if (this.LastdepthFrame.FrameNumber - CDFN1 > 0)
            {
                CDFN1 = this.LastdepthFrame.FrameNumber;
                this.sw7.WriteLine("{0}    {1}    {2}     {3}     {4}    ", CDFN1, this._Kinect.ElevationAngle, Math.Round(this.Accelerometro.X, 3), Math.Round(this.Accelerometro.Y, 3), Math.Round(this.Accelerometro.Z, 3));
            }
        }

        //Calculate the mean of the medians
        private void button1_Click(object sender, RoutedEventArgs e)
        {
            
                this.sw9.Write("{0}      ", collimazione);//numero di collimazioni mediate


                for (int i = 1; i <= righe_griglia_calibrazione * colonne_griglia_calibrazione; i++)
                {   
                    X_Kinect[i-1] = MedianePuntiGrigliaX.Column(i).Mean;
                    Y_Kinect[i-1] = MedianePuntiGrigliaY.Column(i).Mean;
                    Z_Kinect[i-1] = MedianePuntiGrigliaZ.Column(i).Mean;
                    
                    this.sw9.Write("{0:#####.0000}      {1:#####.0000}      {2:#####.0000}      ", X_Kinect[i-1], Y_Kinect[i-1], Z_Kinect[i-1]);
                }

                this.sw9.WriteLine();

                calibrazione(X_Kinect, Y_Kinect, Z_Kinect);
        }

        //Method that calculates the calibration parameters
        void calibrazione(double[] X_kinect, double[] Y_kinect, double[] Z_kinect)
        {
            //Grid coordinates
            double[] X_Ref_B = new double[12] { -517.5, -172.5, 172.5, 517.5, -517.5, -172.5, 172.5, 517.5, -517.5, -172.5, 172.5, 517.5 };// coordinate X dei punti della griglia di riferimento (origine posta al centro della griglia stessa)
            double[] Y_Ref_B = new double[12] { 345.0, 345.0, 345.0, 345.0, 0.0, 0.0, 0.0, 0.0, -345.0, -345.0, -345.0, -345.0 };// coordinate Y dei punti della griglia di riferimento (origine posta al centro della griglia stessa)
            double[] Z_Ref_B = new double[12];

            Sample XKINECT = new Sample(X_kinect);
            Sample YKINECT = new Sample(Y_kinect);
            Sample ZKINECT = new Sample(Z_kinect);

            // barycenter of Kinect coordinates
            double X_Bar = XKINECT.Mean;
            double Y_Bar = YKINECT.Mean;
            double Z_Bar = ZKINECT.Mean;

            //Kinect barycentric coordinates contaneirs
            double[] X_Kinect_Bar = new double[12];
            double[] Y_Kinect_Bar = new double[12];
            double[] Z_Kinect_Bar = new double[12];

            //Kinect barycentric coordinates 
            for (int i = 0; i < 12; i++)
            {
                X_Kinect_Bar[i] = X_kinect[i] - X_Bar;
                Y_Kinect_Bar[i] = Y_kinect[i] - Y_Bar;
                Z_Kinect_Bar[i] = Z_kinect[i] - Z_Bar;
            }

            //Modello rotazione + 2 parametri di scala Yoss=AX+b  oppure B=Yoss-b=AX  X=B*A^-1
           
            RectangularMatrix A = new RectangularMatrix(36, 5); 
            ColumnVector B = new ColumnVector(36);
            ColumnVector bb = new ColumnVector(36);
            ColumnVector Yoss = new ColumnVector(36);
            ColumnVector X = new ColumnVector(5);
            SquareMatrix X1 = new SquareMatrix(5);
            ColumnVector X2 = new ColumnVector(5);

            for (int i = 0; i < 12; i++)
            {
                
                Yoss[3 * i] = X_Ref_B[i];
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
            this.sw9.Write("{0:#####.0000}      {1:#####.0000}      {2:#####.0000}      {3:#####.0000}      {4:#####.0000}      ", a, b, c, lambdaX, lambdaY);
            this.sw9.WriteLine();
        }

        //It changes the grid parameters(rows,columns) and so it also changes the dimensions of statistic sample
        private void button6_Click(object sender, RoutedEventArgs e)
        {
            collimazione = 0;
            righe_griglia_calibrazione = Convert.ToInt32(Righe.Text);
            colonne_griglia_calibrazione =Convert.ToInt32(Colonne.Text);
            punti_griglia_bisX = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
            punti_griglia_bisY = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
            punti_griglia_bisZ = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
            X_Mediane = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
            Y_Mediane = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
            Z_Mediane = new double[colonne_griglia_calibrazione * righe_griglia_calibrazione + 1];
            MedianePuntiGrigliaX = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
            MedianePuntiGrigliaY = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);
            MedianePuntiGrigliaZ = new MultivariateSample(righe_griglia_calibrazione * colonne_griglia_calibrazione + 1);

            X_Kinect = new double[righe_griglia_calibrazione * colonne_griglia_calibrazione];
            Y_Kinect = new double[righe_griglia_calibrazione * colonne_griglia_calibrazione];
            Z_Kinect = new double[righe_griglia_calibrazione * colonne_griglia_calibrazione];
            this.checkBox2.IsEnabled = true;
        }
    }
}
