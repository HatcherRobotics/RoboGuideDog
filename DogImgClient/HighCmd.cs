using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DogImgClient
{
    class HighCmd
    {
		public char mode = '\0';					  // 0:idle, default stand      1:forced stand     2:walk continuously
		public float forwardSpeed = 0;                // speed of move forward or backward, scale: -1~1
		public float sideSpeed = 0;                   // speed of move left or right, scale: -1~1
		public float rotateSpeed = 0;                 // speed of spin left or right, scale: -1~1
		public float bodyHeight = 0;                  // body height, scale: -1~1
		public float footRaiseHeight = 0;             // foot up height while walking (unavailable now)
		public float yaw = 0;                         // unit: radian, scale: -1~1
		public float pitch = 0;                       // unit: radian, scale: -1~1
		public float roll = 0;                        // unit: radian, scale: -1~1
	}
}
