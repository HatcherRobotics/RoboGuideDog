#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <netinet/in.h>
#include <unistd.h>
#include <memory.h>
#include <opencv2/opencv.hpp>

#define IMG_SERVER_PORT 34343
#define MAX_PACK_SIZE 512000

using namespace UNITREE_LEGGED_SDK;

extern void init_loop_server();
extern HighCmd udpCommand;
extern int isCmdSetFinished;
extern int isStopped;

rs2::pipeline p;

cv::dnn::experimental_dnn_34_v21::Net tfNet;

const char* CLASS[] =
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

void send_vision_command(int client_fd)
{
	auto frame = p.wait_for_frames();
	auto color = frame.get_color_frame();
	auto depth = frame.get_depth_frame();
	auto width = color.get_width();
	auto height = color.get_height();
	auto depth_width = depth.get_width();
	auto depth_height = depth.get_height();
	cv::Mat image(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

	tfNet.setInput(cv::dnn::blobFromImage(image));
	cv::Mat mat2 = tfNet.forward();
	cv::Mat mat3(mat2.size[2], mat2.size[3], 5, mat2.ptr(0), 0L);
	for (int i = 0; i < mat3.rows; i++)
	{
		if (!(mat3.at<float>(i, 2) < 0.5f))
		{
			int num = (int)mat3.at<float>(i, 1);
			if (num != 1 && num != 2)
				continue;
			int x = (int)(mat3.at<float>(i, 3) * (float)image.cols);
			int y = (int)(mat3.at<float>(i, 4) * (float)image.rows);
			int x2 = (int)(mat3.at<float>(i, 5) * (float)image.cols);
			int y2 = (int)(mat3.at<float>(i, 6) * (float)image.rows);

			auto center_x = (x + x2) / 2.0f;
			auto center_y = (y + y2) / 2.0f;

			auto center_x_scaled = center_x / width * depth_width;
			auto center_y_scaled = center_y / height * depth_height;

			x -= width / 2;
			y -= height / 2;
			x2 -= width / 2;
			y2 -= height / 2;

			send(client_fd, &num, sizeof(num), 0);
			auto center_depth = depth.get_distance((int)center_x_scaled, (int)center_y_scaled);
			send(client_fd, &center_depth, sizeof(center_depth), 0);
			send(client_fd, &x, sizeof(x), 0);
			send(client_fd, &y, sizeof(y), 0);
			send(client_fd, &x2, sizeof(x2), 0);
			send(client_fd, &y2, sizeof(y2), 0);
		}
	}
	int finishNum = -1;
	send(client_fd, &finishNum, sizeof(finishNum), 0);
}

void recv_control_command(int client_fd)
{
	std::cout << "Recv control command" << std::endl;

	if (isCmdSetFinished)
		isCmdSetFinished = 0;

	recv(client_fd, &udpCommand.mode, sizeof(udpCommand.mode), 0);
	recv(client_fd, &udpCommand.forwardSpeed, sizeof(udpCommand.forwardSpeed), 0);
	recv(client_fd, &udpCommand.sideSpeed, sizeof(udpCommand.sideSpeed), 0);
	recv(client_fd, &udpCommand.rotateSpeed, sizeof(udpCommand.rotateSpeed), 0);
	recv(client_fd, &udpCommand.bodyHeight, sizeof(udpCommand.bodyHeight), 0);
	recv(client_fd, &udpCommand.footRaiseHeight, sizeof(udpCommand.footRaiseHeight), 0);
	recv(client_fd, &udpCommand.yaw, sizeof(udpCommand.yaw), 0);
	recv(client_fd, &udpCommand.pitch, sizeof(udpCommand.pitch), 0);
	recv(client_fd, &udpCommand.roll, sizeof(udpCommand.roll), 0);

	isCmdSetFinished = 1;
}

void send_img_command(int client_fd, rs2::pipeline& p)
{
	std::cout << "Reading frame" << std::endl;

	rs2::frameset frames = p.wait_for_frames();
	rs2::depth_frame depth = frames.get_depth_frame();
	rs2::video_frame color = frames.get_color_frame();

	std::cout << "Sending data" << std::endl;

	// send color frame
	int height = color.get_height();
	int width = color.get_width();

	send(client_fd, &height, sizeof(height), 0);
	send(client_fd, &width, sizeof(width), 0);

	int bytes_per_pixel = color.get_bytes_per_pixel();
	send(client_fd, &bytes_per_pixel, sizeof(bytes_per_pixel), 0);

	int dataSize = color.get_data_size();
	send(client_fd, &dataSize, sizeof(dataSize), 0);

	int sentData = 0;

	auto colorData = color.get_data();

	while (sentData < dataSize)
		sentData += send(client_fd, colorData + sentData, dataSize - sentData, 0);

	// send depth frame
	int depth_height = depth.get_height();
	int depth_width = depth.get_width();

	send(client_fd, &depth_height, sizeof(height), 0);
	send(client_fd, &depth_width, sizeof(width), 0);

	int depth_bytes_per_pixel = depth.get_bytes_per_pixel();
	send(client_fd, &depth_bytes_per_pixel, sizeof(bytes_per_pixel), 0);

	int depth_dataSize = depth.get_data_size();
	send(client_fd, &depth_dataSize, sizeof(dataSize), 0);

	int depth_sentData = 0;

	auto depthData = depth.get_data();

	while (depth_sentData < depth_dataSize)
		depth_sentData += send(client_fd, depthData + depth_sentData, depth_dataSize - depth_sentData, 0);

	fflush(stdout);
}

int main()
{
	std::cout << "Starting" << std::endl;

	std::cout << "Init opencv" << std::endl;

	tfNet = cv::dnn::readNetFromTensorflow("frozen_inference_graph.pb", "graph.pbtxt");

	std::cout << "Init control server" << std::endl;
	init_loop_server();

	fflush(stdout);

	// Init pipeline
	p.start();

	// Init UDP
	auto server_fd = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in server_addr;
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(IMG_SERVER_PORT);

	if (bind(server_fd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0)
	{
		std::cout << "Failed to bind" << std::endl;
		return -1;
	}

	if (listen(server_fd, 10) == -1)
	{
		std::cout << "Failed to listen" << std::endl;
		return -1;
	}

	int client_fd = -1;

	std::cout << "All down" << std::endl;

	// main loop
	while (true)
	{
		if (client_fd == -1)
		{
			std::cout << "Waiting connecting" << std::endl;
			
			isStopped = 1;
			client_fd = accept(server_fd, 0, 0);

			if (client_fd == -1)
			{
				std::cout << "Failed to accept" << std::endl;
				continue;
			}

			std::cout << "Connected" << std::endl;
			isStopped = 0;
		}

		char cmdId;
		int dataLen = recv(client_fd, &cmdId, 1, 0);

		if (dataLen < 1)
		{
			std::cout << "Failed to recv" << std::endl;
			client_fd = -1;
			continue;
		}

		if (dataLen != 1)
		{
			std::cout << "Command not correct" << std::endl;
			continue;
		}

		if (cmdId == 1)
			send_img_command(client_fd, p);
		else if (cmdId == 2)
			recv_control_command(client_fd);
		else if (cmdId == 3)
			send_vision_command(client_fd);
	}
}