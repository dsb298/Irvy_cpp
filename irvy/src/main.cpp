#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// #include <pigpio.h>

#include "tcp_server.hpp"

using boost::asio::ip::tcp;
using namespace boost::asio;
using namespace std::chrono;
using namespace std;
using namespace cv;

void stream(TcpServer&);
// void control();

// void initBot();
// void forward(int);
// void reverse(int);
// void left(int);
// void right(int);
// void stop();

const int port = 4789;

int main()
{
    // initBot();

    // Set the alarm to go off in 5 seconds
    TcpServer tcpCam(port);
    thread stream_thread(stream, ref(tcpCam));
    // thread control_thread(control);

    stream_thread.join();
    // control_thread.join();

    return 0;
}

void stream(TcpServer& tcpCam)
{

    vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
    // boost::system::error_code error;
    // vector<int> frameVecSize(1);
    size_t frameSize[1];
    string resp = "n";
    vector<uchar> frameJpg;
    VideoCapture vid(0);
    Mat frame = Mat(480, 640, CV_8UC3);
    Mat grayframe;

    // send video frame by frame
    cout << "Video stream starting." << endl;
    int cntr = 0;
    int run = 1;
    while(run){

        auto start = high_resolution_clock::now();

        // Take picture
        vid >> frame;

        // cvtColor(img, grayframe, 6);

        // Encode picture to jpeg
        cv::imencode(".jpg", frame, frameJpg, params);
        // cv::imencode(".jpg", grayframe, imgVec, params);

        // Send jpg
        tcpCam.send_frame(frameJpg);

        auto end = high_resolution_clock::now();
        auto elapsed = end - start;
        auto frame_time = duration_cast<milliseconds>(elapsed).count();
        cntr++;
        cout << "Frame: " << cntr << endl;
        cout << "Elapsed Time: " << frame_time << endl;
    }
    vid.release();
}

// void control()
// {
//     // // Create a server endpoint
//     io_context service;
//     ip::tcp::endpoint endpoint(ip::tcp::v4(), port+1);

//     // // Create a socket and bind it to the endpoint
//     ip::tcp::acceptor acceptor(service, endpoint);

//     cout << "Waiting for Commander..." << endl;

//     // Wait for a client to connect
//     ip::tcp::socket socket(service);
//     acceptor.accept(socket);

//     cout << "Connected to Commander!" << endl;

//     char command[1];

//     int run = 1;
//     while(run){
//         read(socket, buffer(command, sizeof(char)));
//         if (command[0]=='q') {
//             run = 0;
//             stop();
//         } else if (command[0]==' '){
//             stop();
//         } else if (command[0]=='w'){
//             forward(255);
//         } else if (command[0]=='s'){
//             reverse(255);
//         } else if (command[0]=='a'){
//             left(255);
//         } else if (command[0]=='d'){
//             right(255);
//         } else if (command[0]=='c'){
//             // will take pic
//             continue;
//         }
//     }
// }

// void initBot()
// {
//     if(gpioInitialise())
//         cout << "Motors ready." << endl;
//     else
//         cout << "Motors failed." << endl;

//     gpioSetMode(12, PI_OUTPUT); // PWM0
//     gpioSetMode(13, PI_OUTPUT); // PWM1
//     gpioSetMode(18, PI_OUTPUT); // PWM0
//     gpioSetMode(19, PI_OUTPUT); // PWM1
// }

// void forward(int s)
// {
//     gpioPWM(12, 0);
//     gpioPWM(13, s);
//     gpioPWM(18, s);
//     gpioPWM(19, 0);
// }

// void reverse(int s)
// {
//     gpioPWM(12, s);
//     gpioPWM(13, 0);
//     gpioPWM(18, 0);
//     gpioPWM(19, s);
// }

// void left(int s)
// {
//     gpioPWM(12, s);
//     gpioPWM(13, s);
//     gpioPWM(18, 0);
//     gpioPWM(19, 0);
// }

// void right(int s)
// {
//     gpioPWM(12, 0);
//     gpioPWM(13, 0);
//     gpioPWM(18, s);
//     gpioPWM(19, s);
// }

// void stop()
// {
//     gpioPWM(12, 0);
//     gpioPWM(13, 0);
//     gpioPWM(18, 0);
//     gpioPWM(19, 0);
// }