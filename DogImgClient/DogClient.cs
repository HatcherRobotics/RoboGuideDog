using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace DogImgClient
{
    class DogClient
    {
        Bitmap bitmap;
        Bitmap depthmap;
        public const string DogServer = "192.168.11.110";
        public const int DogPort = 34343;

        TcpClient client;

        public void Connect()
        {
            client = new TcpClient();
            client.Connect(DogServer, DogPort);
        }

        public VisualDetectedResult[] ReadVisualResult()
        {
            var netStream = client.GetStream();
            var reader = new BinaryReader(netStream);
            var writer = new BinaryWriter(netStream);

            writer.Write((byte)3);

            var list = new List<VisualDetectedResult>();
            while(true)
            {
                int num = reader.ReadInt32();
                if (num == -1)
                    break;
                float depth = reader.ReadSingle();
                int x = reader.ReadInt32();
                int y = reader.ReadInt32();
                int x2 = reader.ReadInt32();
                int y2 = reader.ReadInt32();
                list.Add(new VisualDetectedResult { num = num, depth = depth, x = x, y = y, x2 = x2, y2 = y2 });
            }
            return list.ToArray();
        }

        public Bitmap[] ReadFrame()
        {
            var netStream = client.GetStream();
            var reader = new BinaryReader(netStream);
            var writer = new BinaryWriter(netStream);

            writer.Write((byte)1);

            var height = reader.ReadInt32();
            var width = reader.ReadInt32();
            var bytePerPixel = reader.ReadInt32();
            var dataSize = reader.ReadInt32();

            var totalData = new byte[dataSize];
            var readedSize = 0;
            while (readedSize < dataSize)
                readedSize += reader.Read(totalData, readedSize, dataSize - readedSize);

            if (bitmap == null)
                bitmap = new Bitmap(width, height, PixelFormat.Format24bppRgb);

            var bitData = bitmap.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            Marshal.Copy(totalData, 0, bitData.Scan0, dataSize);

            bitmap.UnlockBits(bitData);

            var depth_height = reader.ReadInt32();
            var depth_width = reader.ReadInt32();
            var depth_bytePerPixel = reader.ReadInt32();
            var depth_dataSize = reader.ReadInt32();

            var depth_totalData = new byte[depth_dataSize];
            var depth_readedSize = 0;
            while (depth_readedSize < depth_dataSize)
                depth_readedSize += reader.Read(depth_totalData, depth_readedSize, depth_dataSize - depth_readedSize);

            if (depthmap == null)
                depthmap = new Bitmap(depth_width, depth_height);

            for (var x = 0; x < depth_width; ++x)
            for (var y = 0; y < depth_height; ++y)
            {
                var l = depth_totalData[(x + y * depth_width) * depth_bytePerPixel + 1];
                var h = depth_totalData[(x + y * depth_width) * depth_bytePerPixel + 0];
                    //var depth = (l | h * byte.MaxValue) / 256;
                var depth = l * 10;
                if (depth > 255)
                    depth = 255;
                depthmap.SetPixel(x, y, Color.FromArgb(255, depth, depth, depth));
            }

            return new[] { bitmap, depthmap };
        }

        public void SendCmd(HighCmd cmd)
        {
            var netStream = client.GetStream();
            var writer = new BinaryWriter(netStream);

            writer.Write((byte)2);
            writer.Write(cmd.mode);
            writer.Write(cmd.forwardSpeed);
            writer.Write(cmd.sideSpeed);
            writer.Write(cmd.rotateSpeed);
            writer.Write(cmd.bodyHeight);
            writer.Write(cmd.footRaiseHeight);
            writer.Write(cmd.yaw);
            writer.Write(cmd.pitch);
            writer.Write(cmd.roll);

            writer.Flush();
        }
    }
}
