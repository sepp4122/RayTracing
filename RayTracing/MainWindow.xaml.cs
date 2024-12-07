using Microsoft.VisualBasic;
using System;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace RayTracing
{
    public partial class MainWindow : Window
    {
        private WriteableBitmap _bitmap;
        private RayTracingEngine rayTracingEngine;
        public MainWindow()
        {
            InitializeComponent();
            _bitmap = new WriteableBitmap(
                Constants.resolution,
                Constants.resolution,
                100, 
                100, 
                PixelFormats.Bgra32,
                null
            );
            Display.Source = _bitmap;
            Constants.console = new Consoles(this);
            rayTracingEngine = new RayTracingEngine(this);
            InitializeInputHandlers();
            this.Cursor = Cursors.None;
        }
        public void DrawPixels(RayTracing.Color[,] colorArray)
        {
            int width = colorArray.GetLength(0);
            int height = colorArray.GetLength(1);
            _bitmap.Lock();
            unsafe
            {
                IntPtr backBuffer = _bitmap.BackBuffer;
                int stride = _bitmap.BackBufferStride;
                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        int pixelOffset = y * stride + x * 4; // 4 bytes per pixel (ARGB)
                        RayTracing.Color color = colorArray[x, y];
                        *((int*)(backBuffer + pixelOffset)) = (color.a << 24) | (color.b << 16) | (color.g << 8) | color.r;
                    }
                }
            }
            _bitmap.AddDirtyRect(new Int32Rect(0, 0, width, height));
            _bitmap.Unlock();
        }
        public void InitializeInputHandlers()
        {
            this.KeyDown += new KeyEventHandler(rayTracingEngine.HandleKeyPress);
            this.MouseMove += new MouseEventHandler(rayTracingEngine.HandleMouseInput);
        }
        private void Image_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Constants.console.DisableInput();
        }
        private void ConsoleClicked(object sender, MouseButtonEventArgs e)
        {
            Constants.console.EnableInput();
        }
        public void OnConsoleKeyInput(object senderm, KeyEventArgs e)
        {
            if (e.Key == Key.Enter) Constants.console.ParseInput();
        }
    }
}