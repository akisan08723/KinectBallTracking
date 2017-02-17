using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Simulation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor _sensor = null;
        private ColorFrameReader _colorReader = null;
        private BodyFrameReader _bodyReader = null;
        private IList<Body> _bodies = null;
        private int _width = 0;
        private int _height = 0;
        private byte[] _pixels = null;
        private WriteableBitmap _bitmap = null;

        private int flag = 0;
        private int ball_flag = 0;
        private int goupflag = 0;
        private int stopflag = 0;

        private float startX = 0;
        private float startY = 0;
        private float pixel_per_cm = 0;
        private float[] po = { 0, 0 };

        private float starttime = 0;
        private float curenttime = 0;
        private float time = 0;

        private float swingStartPosition = 0;
        private float swingEndPosition = 0;
        private float swingDistence = 0;
        private float swingTime = 0;
        private float swingStartTime = 0;
        private float swingEndTime = 0;

        public MainWindow()
        {
            InitializeComponent();
            _sensor = KinectSensor.GetDefault();

            if(_sensor != null)
            {
                _sensor.Open();

                _width = _sensor.ColorFrameSource.FrameDescription.Width;
                _height = _sensor.ColorFrameSource.FrameDescription.Height;

                _colorReader = _sensor.ColorFrameSource.OpenReader();
                _colorReader.FrameArrived += ColorReader_FrameArrived;

                _bodyReader = _sensor.BodyFrameSource.OpenReader();    
                _bodyReader.FrameArrived += BodyReader_FrameArrived;

                _pixels = new byte[_width * _height * 4];
                _bitmap = new WriteableBitmap(_width, _height, 96.0, 96.0, PixelFormats.Bgra32, null);

                _bodies = new Body[_sensor.BodyFrameSource.BodyCount];

                camera.Source = _bitmap;
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if(_colorReader != null)
            {
                _colorReader.Dispose();
            }

            if (_bodyReader != null)
            {
                _bodyReader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        private void ColorReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    frame.CopyConvertedFrameDataToArray(_pixels, ColorImageFormat.Bgra);

                    _bitmap.Lock();
                    Marshal.Copy(_pixels, 0, _bitmap.BackBuffer, _pixels.Length);
                    _bitmap.AddDirtyRect(new Int32Rect(0, 0, _width, _height));
                    _bitmap.Unlock();
                }
            }
        }

        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            HandState rightHandState;
            HandState leftHandState;

            using(var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    frame.GetAndRefreshBodyData(_bodies);

                    Body body = _bodies.Where(b => b.IsTracked).FirstOrDefault();

                    if(body != null)
                    {
                        Joint handRight = body.Joints[JointType.HandRight];
                        Joint handLeft = body.Joints[JointType.HandLeft];
                        Joint elbowRight = body.Joints[JointType.ElbowRight];

                        if (handRight.TrackingState != TrackingState.NotTracked)
                        {
                            CameraSpacePoint handRightPosition = handRight.Position;
                            CameraSpacePoint handLeftPosition = handLeft.Position;
                            CameraSpacePoint elbowRightPosition = elbowRight.Position;
                            ColorSpacePoint handRightPoint = _sensor.CoordinateMapper.MapCameraPointToColorSpace(handRightPosition);
                            ColorSpacePoint handLeftPoint = _sensor.CoordinateMapper.MapCameraPointToColorSpace(handLeftPosition);

                            float mapRightHandx = handRightPoint.X;
                            float mapRightHandy = handRightPoint.Y;
                            float mapLeftHandx = handLeftPoint.X;
                            float mapLeftHandy = handLeftPoint.Y;
                            float realRightHandx = handRightPosition.X;
                            float realRightHandy = handRightPosition.Y;
                            float realLeftHandx = handLeftPosition.X;
                            float realLeftHandy = handLeftPosition.Y;
                            float realRightElbowy = elbowRightPosition.Y;
                            rightHandState = body.HandRightState;
                            leftHandState = body.HandLeftState;

                            if (!float.IsInfinity(mapLeftHandx) && !float.IsInfinity(mapLeftHandy) && !float.IsInfinity(mapRightHandx) && !float.IsInfinity(mapRightHandy))
                            {
                                if (flag == 0 && rightHandState == HandState.Closed && leftHandState == HandState.Closed)
                                {
                                    float pixel = mapRightHandx - mapLeftHandx;
                                    float cm = (realRightHandx - realLeftHandx) * 100;
                                    pixel_per_cm = pixel / cm;
                                    //changing a ball scale. "myBall" is in MainWindow.xaml 
                                    myBall.Height = pixel;
                                    myBall.Width = pixel;

                                    flag = 1;
                                }

                                if(realLeftHandy<-0.1&&realRightHandy<realRightElbowy&& ball_flag==0&&rightHandState==HandState.Closed&& leftHandState == HandState.Closed)
                                {
                                    ball_flag = 1;
                                    swingStartTime = DateTime.Now.Millisecond / 1000f + DateTime.Now.Second;
                                    swingStartPosition = realLeftHandy * 100;
                                }

                                else if (realLeftHandy > 0.1 && realRightHandy > realRightElbowy && ball_flag == 1 && rightHandState == HandState.Closed && leftHandState == HandState.Closed)
                                {
                                    ball_flag = 2;
                                    starttime = DateTime.Now.Millisecond / 1000f + DateTime.Now.Second + DateTime.Now.Minute * 60;

                                    swingEndTime = DateTime.Now.Millisecond / 100f + DateTime.Now.Second;
                                    swingEndPosition = realLeftHandy * 100;
                                    swingTime = swingEndTime - swingStartTime;
                                    swingDistence = swingEndPosition - swingStartPosition;
                                }
                                else if (ball_flag == 2)
                                {
                                    curenttime = DateTime.Now.Millisecond / 1000f + DateTime.Now.Second + DateTime.Now.Minute * 60;

                                    //simulation
                                    float swing = swingDistence / swingTime;
                                    if (stopflag != 1)
                                    {
                                        time = curenttime - starttime;
                                        Ball_Data(swing, time);
                                        goupflag = 1;
                                    }
                                    //Printing on TextBlock in MainWindow.xaml
                                    Hand_data.Text = "swing(Initial velocity):" + (int)swing + " velocity(V_0+V):" + po[0];

                                    Canvas.SetLeft(myBall, startX);
                                    Canvas.SetTop(myBall, startY - po[1] * pixel_per_cm);//"po[1]" is ball's moving distance. changing cm to pixel by "pixel_per_cm".

                                    if ((startY - po[1] * pixel_per_cm) > 1000)
                                    {
                                        Stop_reset();
                                    }

                                }
                                if (goupflag == 0)
                                {
                                    float hands_distance = mapRightHandx - mapLeftHandx;
                                    //changing a Ball position
                                    Canvas.SetLeft(myBall, mapLeftHandx + (hands_distance / 2) - (myBall.Width / 2));
                                    Canvas.SetTop(myBall, mapLeftHandy - myBall.Height / 2);

                                    //startX,Y is start point of ball to start the simulation
                                    startX = mapLeftHandx + (hands_distance / 2) - ((float)myBall.Width / 2.0f);
                                    startY = mapLeftHandy - ((float)myBall.Height / 2.0f);

                                }
                                //Printing on TextBlock in MainWindow.xaml
                                Ball_data.Text = "Flag: " + ball_flag + " Y:" + (int)(startY - po[1] * pixel_per_cm) + " Time:" + time;

                            }


                        }

                    }
                }
            }
        }

        private void Ball_Data(float swing, float time)
        {
            float g = 9.8f;
            float v = swing - (g * time);
            float Y = (swing * time) - ((g * (time * time)) / 2);
            po[0] = v;
            po[1] = Y;
        }

        private void Stop_reset()
        {
            stopflag = 1;
        }
    }
}
