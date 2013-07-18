Kinect-Assessment-Tool
======================

The Kinect tool is a measurement tool that displays in real time on the PC screen the depth and RGB data captured by the Kinect, respectively as a gray depth map at 640x480 resolution, in the left side of the window, and RGB image at 1280x960 resolution, in the right side. 

The depth and the RGB images have been aligned using the CoordinateMapper method: in this way the depth data can be retrieved at every RGB pixel which is inside the overlapping region of the two images. So, clicking with the mouse left button on the RGB image, two circles are drawn at the same time: a red one on the RGB image and the equivalent yellow one on the depth image. Instead, clicking directly on the depth image, a green circle is drawn only on this image. At the same time, the circle identification number is also drawn in a textblock under its own circle, assigned in chronological order of selection. The circles point out the selected points: clicking on them with the mouse right button, a popup appears showing the relative coordinate of the point at the given depth frame number. 

The Kinect tool allows also to calculate the parameter of a Kinect calibration model. 
To calculate the calibration parameters (columns, rows,  and step), the user must first select the calibration grid parameters. 
Once theses have been selected, the user must press the “Acquisisci griglia” button and then he can start the calibration procedure by selecting the grid points on the RGB image. 
Once the points have been selected, data capture begins pushing the red checkbox;  for a single collimation the application retrieves  the coordinates (expressed in mm) of the selected points every tot frame(for example every two frames) and then it calculates the median values among all the frames captured (for example the user can choose to acquire 100 frames for single collimation), in order to get the central value among all the variations of the depth values streamed by the Kinect, filtering out the effect of possible outliers. When the checkbox check disappears, the user can reselect the points for another collimation.
When the selected number of collimations have been acquired,  by pushing the “media delle mediane”  button,  the user  can finally calculate the calibration parameters. 
The program presents also some additional features: the display of the depth data in a textblock next to the mouse pointer icon, the control of the Kinect's elevation angle, the display of the accelerometer data and an augmented reality (AR) sub - application.
The first shows the depth value of the pixel pointed by the mouse on the depth or on the RGB image in real time. The second allows to increase and/or to decrease the Kinect's elevation angle, in order to frame better the object to measure. The third shows the values of the three accelerometer coordinates in a textblock under the depth image and, checking the accelerometer checkbox, it allows them to be saved in a text file. The latter draws red and blue stripes on the RGB image in real time, checking the blue AR checkbox: they are perpendicular to the X axis, therefore they are vertical if the X axis is horizontal and so on.
In the 2 windows version, the application allows an automatic collimation of the grid points, thanks to the use of Emgu library. In this way the user must only select the grid parameters and then press the get grid button:  the application then automatically starts to calculate the calibration parameters, without the need to press any other button. The 2 windows application shows only the RGB image at 1280x960 resolution.
The two applications run on both Windows 7 and Windows 8 operative systems.
The applications can be build using Visual Studio 2010 or Visual Studio 2012.

DEPENDENCIES
Metanumerics 
http://metanumerics.codeplex.com/releases/view/93895

Emgu 
http://sourceforge.net/projects/emgucv/files/?source=navbar
Note: to avoid the cvInvoke exception the output path must be in the emgu installation folder 
(for example: C:\Emgu\emgucv-windows-universal-gpu 2.4.9.1847\bin))
