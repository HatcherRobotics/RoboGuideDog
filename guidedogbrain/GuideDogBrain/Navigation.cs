using System;
using System.Collections.Generic;
using System.Data;
using System.Device.Location;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Speech.Synthesis;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using Newtonsoft.Json;

namespace GuideDogBrain
{
    public class Navigation
    {
        private SpeechSynthesizer speech = new SpeechSynthesizer();

        public enum Result
        {
            Ok, Error
        }

        public const string AK = "ExTXUwoSCWP3u8w5bEKdDDOg7Ght39Cj";

        private readonly GeoCoordinateWatcher _watcher = new GeoCoordinateWatcher(GeoPositionAccuracy.High);

        public GeoCoordinate Position => Wgs842Gcj02(_watcher.Position.Location.Longitude, _watcher.Position.Location.Latitude);
        public RoutePlan RoutePlan { get; private set; }
        private int _currentStep;
        public float speed = 0;

        public void Init()
        {
            if (!_watcher.TryStart(false, TimeSpan.FromSeconds(15)))
                MessageBox.Show("未能启动定位服务");

            while(_watcher.Status != GeoPositionStatus.Ready)
                Thread.Sleep(10);

            speech.Rate = 1;
            speech.SelectVoice("Microsoft Huihui Desktop");
            speech.Volume = 100;
            speech.Speak("导航系统初始化结束");
        }

        public void Update()
        {
            if (RoutePlan == null)
                return;

            if (_currentStep >= RoutePlan.result.routes[0].steps.Count)
                return;

            /*var instruction = RoutePlan.result.routes[0].steps[_currentStep].instruction;
            instruction = instruction.Replace("</b>", " ").Replace("<b>", " ");
            speech.Speak(instruction);*/

            ++_currentStep;
        }

        public async Task<Result> StartNavigation(GeoCoordinate destination)
        {
            return await StartNavigation(Position, destination);
        }

        private async Task StartNavigationInner(GeoCoordinate origin, GeoCoordinate destination)
        {
            RoutePlan = null;

            var offsetOrigin = origin;
            var offsetDist = destination;

            var getUrl =
                "http://api.map.baidu.com/directionlite/v1/walking?" +
                $"origin={offsetOrigin.Latitude},{offsetOrigin.Longitude}&" +
                $"destination={offsetDist.Latitude},{offsetDist.Longitude}&" +
                "coord_type=gcj02&" +
                "ret_coordtype=gcj02&" +
                $"ak={AK}";

            HttpResponseMessage routePlanResponse;
            using (var httpClient = new HttpClient())
                routePlanResponse = httpClient.GetAsync(getUrl).Result;

            var routePlanJson = await routePlanResponse.Content.ReadAsStringAsync();
            RoutePlan = JsonConvert.DeserializeObject<RoutePlan>(routePlanJson);

            if (RoutePlan == null)
                throw new NoNullAllowedException();

            _currentStep = 0;
        }

        public static GeoCoordinate Gcj022Wgs84(double lng, double lat)
        {
            var ee = 0.00669342162296594323;
            var a = 6378245.0;

            if (lng < 72.004 || lng > 137.8347)
                return new GeoCoordinate(lat, lng);
            if (lat < 0.8293 || lat > 55.8271)
                return new GeoCoordinate(lat, lng);

            var dlat = transformlat(lng - 105.0, lat - 35.0);
            var dlng = transformlng(lng - 105.0, lat - 35.0);
            var radlat = lat / 180.0 * Math.PI;
            var magic = Math.Sin(radlat);
            magic = 1 - ee * magic * magic;
            var sqrtmagic = Math.Sqrt(magic);
            dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * Math.PI);
            dlng = (dlng * 180.0) / (a / sqrtmagic * Math.Cos(radlat) * Math.PI);
            var mglat = lat + dlat;
            var mglng = lng + dlng;

            return new GeoCoordinate(lat * 2 - mglat, lng * 2 - mglng);
        }

        public static GeoCoordinate Wgs842Gcj02(double lng, double lat)
        {
            var ee = 0.00669342162296594323;
            var a = 6378245.0;

            if (lng < 72.004 || lng > 137.8347)
                return new GeoCoordinate(lat, lng);
            if (lat < 0.8293 || lat > 55.8271)
                return new GeoCoordinate(lat, lng);

            var dlat = transformlat(lng - 105.0, lat - 35.0);
            var dlng = transformlng(lng - 105.0, lat - 35.0);
            var radlat = lat / 180.0 * Math.PI;
            var magic = Math.Sin(radlat);
            magic = 1 - ee * magic * magic;
            var sqrtmagic = Math.Sqrt(magic);
            dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * Math.PI);
            dlng = (dlng * 180.0) / (a / sqrtmagic * Math.Cos(radlat) * Math.PI);
            var mglat = lat + dlat;
            var mglng = lng + dlng;

            return new GeoCoordinate(mglat, mglng);
        }

        private static double transformlat(double lng, double lat)
        {
            double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * Math.Sqrt(Math.Abs(lng));
            ret += (20.0 * Math.Sin(6.0 * lng * Math.PI) + 20.0 * Math.Sin(2.0 * lng * Math.PI)) * 2.0 / 3.0;
            ret += (20.0 * Math.Sin(lat * Math.PI) + 40.0 * Math.Sin(lat / 3.0 * Math.PI)) * 2.0 / 3.0;
            ret += (160.0 * Math.Sin(lat / 12.0 * Math.PI) + 320 * Math.Sin(lat * Math.PI / 30.0)) * 2.0 / 3.0;
            return ret;
        }


        private static double transformlng(double lng, double lat)
        {
            double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * Math.Sqrt(Math.Abs(lng));
            ret += (20.0 * Math.Sin(6.0 * lng * Math.PI) + 20.0 * Math.Sin(2.0 * lng * Math.PI)) * 2.0 / 3.0;
            ret += (20.0 * Math.Sin(lng * Math.PI) + 40.0 * Math.Sin(lng / 3.0 * Math.PI)) * 2.0 / 3.0;
            ret += (150.0 * Math.Sin(lng / 12.0 * Math.PI) + 300.0 * Math.Sin(lng / 30.0 * Math.PI)) * 2.0 / 3.0;
            return ret;
        }

        public async Task<Result> StartNavigation(GeoCoordinate origin, GeoCoordinate destination)
        {
            try
            {
                await StartNavigationInner(origin, destination);
            }
            catch
            {
                return Result.Error;
            }

            return Result.Ok;
        }
    }
}
