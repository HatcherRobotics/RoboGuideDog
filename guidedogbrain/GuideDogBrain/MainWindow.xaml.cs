using System;
using System.Collections.Generic;
using System.Device.Location;
using System.Linq;
using System.Runtime.Remoting.Channels;
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
using System.Windows.Threading;
using Microsoft.Maps.MapControl.WPF;

namespace GuideDogBrain
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        private Navigation _navigation = new Navigation();
        private Pushpin _selectedPushpin;
        private List<Pushpin> _path = new List<Pushpin>();

        public MainWindow()
        {
            InitializeComponent();
            _navigation.Init();

            var dispatcherTimer = new DispatcherTimer();
            dispatcherTimer.Tick += (sender, e) => _navigation.Update();
            dispatcherTimer.Interval = TimeSpan.FromMilliseconds(100);

            dispatcherTimer.Start();

            var center = _navigation.Position;
            Map.ZoomLevel = 16;
            Map.Center = new Location(center.Latitude, center.Longitude);

            OriginTextBox.Text = center.Longitude + "," + center.Latitude;
            AddPushpin(center);
        }

        private Pushpin AddPushpin(GeoCoordinate loc)
        {
            var pushpin = new Pushpin();
            pushpin.Location = new Location(loc.Latitude, loc.Longitude);
            Map.Children.Add(pushpin);

            return pushpin;
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedPushpin == null)
                return;

            var dest = new GeoCoordinate(_selectedPushpin.Location.Latitude, _selectedPushpin.Location.Longitude);

            _navigation.StartNavigation(dest).Wait();

            foreach (var pushpin in _path)
                Map.Children.Remove(pushpin);
            ResultTextBox.Text = "";

            foreach (var step in _navigation.RoutePlan.result.routes[0].steps)
            {
                var paths = step.path.Split(';');

                foreach (var path in paths)
                {
                    var pos = path.Split(',');
                    double.TryParse(pos[0], out var lon); 
                    double.TryParse(pos[1], out var lat);

                    _path.Add(AddPushpin(new GeoCoordinate(lat, lon)));
                    ResultTextBox.Text += path + "\r";
                }
                //_path.Add(AddPushpin(new GeoCoordinate(step.start_location.lat, step.start_location.lng)));
                //_path.Add(AddPushpin(new GeoCoordinate(step.end_location.lat, step.end_location.lng)));
            }
        }

        private void Map_MouseClick(object sender, MouseEventArgs e)
        {
            if (_selectedPushpin != null)
                Map.Children.Remove(_selectedPushpin);

            var clickPoint = e.GetPosition(sender as IInputElement);
            var mapPos = Map.ViewportPointToLocation(clickPoint);
            _selectedPushpin = AddPushpin(new GeoCoordinate(mapPos.Latitude, mapPos.Longitude));

            DestTextBox.Text = mapPos.Longitude + "," + mapPos.Latitude;
        }
    }
}
