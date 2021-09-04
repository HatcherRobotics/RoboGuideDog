using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DogImgClient
{
    struct VisualDetectedResult
    {
        public int num;
        public float depth;
        public int x;
        public int y;
        public int x2;
        public int y2;
    }
}
