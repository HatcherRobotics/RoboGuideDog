using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Speech.Recognition;
using System.Globalization;
using System.Speech.Synthesis;

namespace DogImgClient
{
    public partial class MainForm : Form
    {
        private static string[] CLASS =
        {
            "", "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
            "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
            "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
            "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
            "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
            "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
        };

        private Task msgTask = null;

        private SpeechRecognitionEngine recognizer;
        private SpeechSynthesizer speech;

        private DictationGrammar dictationGrammar;

        public string prefex = "导盲犬";

        private Tuple<string, int>[] keywordlist = 
        {
            new Tuple<string, int>("停下" ,0),
            new Tuple<string, int>("趴下" ,0),
            new Tuple<string, int>("前进" ,1),
            new Tuple<string, int>("后退" ,1),
            new Tuple<string, int>("左转" ,1),
            new Tuple<string, int>("右转" ,1),
            new Tuple<string, int>("慢点" ,2),
            new Tuple<string, int>("快点" ,2),
            new Tuple<string, int>("起来" ,3)
        };

        DogClient dogClient = new DogClient();

        HighCmd cmd = new HighCmd();

        public float speed = 0.3f;

        public int movingState;

        public int stopDelay;
        public float currentmessageLevel;

        public bool isRelax = false;

        public bool isLock = false;



        public MainForm()
        {
            InitializeComponent();
            speech = new SpeechSynthesizer();
            speech.SpeakAsync("主人 我已经准备好引领你前进了");
            speech.SpeakCompleted += (arg, obj) => currentmessageLevel = 0;
            speech.Rate = 5;
        }

        public void SetImg(Bitmap[] bitmap)
        {
            pictureBox1.Image = bitmap[0];
            pictureBox2.Image = bitmap[1];
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            dogClient.Connect();
            SRecognition();
            recognizer.RecognizeAsync(RecognizeMode.Multiple);
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            var imgs = dogClient.ReadFrame();
            SetImg(imgs);
        }

        private void cmdTextChanged(object sender, EventArgs e)
        {
            int.TryParse(modeTextBox.Text, out var mode);
            cmd.mode = (char)mode;
            float.TryParse(forwardSpeedTextBox.Text, out cmd.forwardSpeed);
            float.TryParse(sideSpeedTextBox.Text, out cmd.sideSpeed);
            float.TryParse(rotateSpeedTextBox.Text, out cmd.rotateSpeed);
            float.TryParse(bodyHeightTextBox.Text, out cmd.bodyHeight);
            float.TryParse(footRaiseHeightTextBox.Text, out cmd.footRaiseHeight);
            float.TryParse(yawTextBox.Text, out cmd.yaw);
            float.TryParse(pitchTextBox.Text, out cmd.pitch);
            float.TryParse(rollTextBox.Text, out cmd.roll);

            dogClient.SendCmd(cmd);
        }

        public void SRecognition() //创建关键词语列表  
        {
            CultureInfo myCIintl = new CultureInfo("zh-CN");
            foreach (RecognizerInfo config in SpeechRecognitionEngine.InstalledRecognizers())//获取所有语音引擎  
            {
                Console.WriteLine(config.Culture.EnglishName);
                if (config.Culture.Equals(myCIintl))
                {
                    recognizer = new SpeechRecognitionEngine(config);
                    break;
                }//选择识别引擎
            }
            if (recognizer != null)
            {
                InitializeSpeechRecognitionEngine();//初始化语音识别引擎  
                dictationGrammar = new DictationGrammar();
            }
            else
            {
                MessageBox.Show("创建语音识别失败");
            }
        }

        void InitializeSpeechRecognitionEngine()
        {
            // Create and load a dictation grammar.
            //recognizer.LoadGrammar(new DictationGrammar());

            // Configure input to the speech recognizer.
            recognizer.SetInputToDefaultAudioDevice();

            // Modify the initial silence time-out value.
            recognizer.InitialSilenceTimeout = TimeSpan.FromSeconds(5);

            foreach (var pair in keywordlist)
            {
                GrammarBuilder GB = new GrammarBuilder();
                GB.Append(prefex);
                GB.Append(pair.Item1);
                Grammar G = new Grammar(GB);
                Console.WriteLine(G.RuleName);
                G.Priority = pair.Item2;
                G.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(G_SpeechRecognized);
                recognizer.LoadGrammar(G);
            }
        }

        void G_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            if (isLock)
                return;
            stopDelay = -1;
            Console.WriteLine(e.Result.Text);
            switch (e.Result.Text.Substring(prefex.Length))
            {
                case "快点":
                    if (!isRelax)
                        SpeedUp();
                    break;
                case "慢点":
                    if (!isRelax)
                        SpeedDown();
                    break;
                case "前进":
                    if (!isRelax)
                        Forward();
                    break;
                case "后退":
                    if (!isRelax)
                        Backword();
                    break;
                case "停下":
                    if (!isRelax)
                        Stop();
                    break;
                case "左转":
                    if (!isRelax)
                        RotateLeft();
                    break;
                case "右转":
                    if (!isRelax)
                        RotateRight();
                    break;
                case "趴下":
                    SetRelax(true);
                    break;
                case "起来":
                    SetRelax(false);
                    break;
            }
        }

        void SetRelax(bool relax)
        {
            isRelax = relax;
            HighCmd command = new HighCmd();
            command.mode = (char)1;
            command.bodyHeight = isRelax ? -1 : 0;
            dogClient.SendCmd(command);
        }
        void RotateLeft()
        {
            movingState = 0;
            HighCmd command = new HighCmd();
            command.mode = (char)2;
            command.rotateSpeed = 0.3f;
            dogClient.SendCmd(command);
            stopDelay = 10;
        }

        void RotateRight()
        {
            movingState = 0;
            HighCmd command = new HighCmd();
            command.mode = (char)2;
            command.rotateSpeed = -0.3f;
            dogClient.SendCmd(command);
            stopDelay = 10;
        }

        void SpeedUp()
        {
            speed += 0.05f;
            if (speed > 1)
                speed = 0;
        }
        void SpeedDown()
        {
            speed -= 0.05f;
            if (speed < 0.1f)
                speed = 0.1f;

            if (movingState == 1)
                Forward();
            else if (movingState == -1)
                Backword();
        }

        void Forward()
        {
            movingState = 1;
            HighCmd command = new HighCmd();
            command.mode = (char)2;
            command.forwardSpeed = speed;
            dogClient.SendCmd(command);
        }

        void Backword()
        {
            movingState = -1;
            HighCmd command = new HighCmd();
            command.mode = (char)2;
            command.forwardSpeed = -speed;
            dogClient.SendCmd(command);
        }

        void Stop()
        {
            movingState = 0;
            HighCmd command = new HighCmd();
            command.mode = (char)1;
            dogClient.SendCmd(command);
        }

        private void stopTimer_Tick(object sender, EventArgs e)
        {
            --stopDelay;

            if (stopDelay == 0)
                Stop();
        }

        private void button2_Click(object sender, EventArgs e)
        {
            Stop();
        }

        private void button3_Click(object sender, EventArgs e)
        {
            var visualResult = dogClient.ReadVisualResult();

            foreach (var result in visualResult)
            {

            }
        }

        private void safityTestTimer_Tick(object sender, EventArgs e)
        {
            if (msgTask != null)
                return;

            msgTask = new Task(() =>
            {
                var visualResult = dogClient.ReadVisualResult();

                foreach (var result in visualResult)
                {
                    if (result.depth > 5)
                        continue;
                    if (result.depth == 0)
                        continue;

                    var centerX = (result.x + result.x2) / 2.0f;

                    if (Math.Abs(centerX) > 350)
                        continue;

                    switch (result.num)
                    {
                        case 1:
                            if (currentmessageLevel - 0.1f >= result.depth * 10)
                            {
                                currentmessageLevel = (int)result.depth * 10;
                            }
                            speech.Speak($"有行人出现在您前方{result.depth}米 主人");
                            break;
                    }
                }

                msgTask = null;
            });
            msgTask.Start();
        }

        private void button4_Click(object sender, EventArgs e)
        {
            isLock = !isLock;
            Stop();
        }
        private void button5_Click_1(object sender, EventArgs e)
        {
            SetImg(dogClient.ReadFrame());
        }
    }
}
