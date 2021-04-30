using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GuideDogBrain
{
    [Serializable]
    public class RoutePlan
    {
        [Serializable]
        public struct Position
        {
            public double lng;
            public double lat;
        }

        [Serializable]
        public struct Result
        {
            public Position origin;
            public Position destination;

            public List<Route> routes;
        }

        [Serializable]
        public struct Route
        {
            public int distance;
            public int duration;

            public List<Step> steps;
        }

        [Serializable]
        public struct Step
        {
            public int direction;
            public int distance;
            public int duration;
            public string instruction;
            public Position start_location;
            public Position end_location;
            public string path;
        }

        public int status;
        public string message;
        public int type;

        public Result result;
    }
}
