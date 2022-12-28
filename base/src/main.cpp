#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

// #include <ncurses.h>
#include <unistd.h>

#include "tcp_client.hpp"

using boost::asio::ip::tcp;
using namespace std::chrono;
using namespace std;
using namespace cv;

static void visualize(Mat& input, int frame, Mat& faces, double fps, int thickness);
void display(TcpClient&);
void commander();

// IP address of raspberry pi
// const string ip_addr = "192.168.1.162";
const string ip_addr = "127.0.0.1";
const int port = 4789;

int main()
{
    int result = nice(1);
    if (result == -1) {
        std::cerr << "Error setting priority" << std::endl;
        return 1;
    }

    TcpClient tcpCam(ip_addr, port);

    thread display_thread(display, ref(tcpCam));
    // thread command_thread(commander);

    display_thread.join();
    // command_thread.join();

    return 0;
}

void display(TcpClient& tcpCam)
{
    // FaceDetector -----------------------------------------------

    // Face detection model: https://github.com/opencv/opencv_zoo/tree/master/models/face_detection_yunet
    String fd_modelPath = "/home/devli/Documents/models/face_detection_yunet_2022mar.onnx";

    // Face recognition model: https://github.com/opencv/opencv_zoo/tree/master/models/face_recognition_sface
    String fr_modelPath = "/home/devli/Documents/models/face_recognition_sface_2021dec.onnx";

    // Threshold must be 0-1. scoreThreshold > score means a face is recognized
    float scoreThreshold = .9;

    // Suppress bounding boxes of iou >= nms_threshold -- idk what this means
    float nmsThreshold = .3;

    // Keep top_k bounding boxes before NMS -- idk what this means
    int topK = 5000;

    // Scale to resize video frames
    float scale = 1.0;

    // No clue
    double cosine_similar_thresh = 0.363;
    double l2norm_similar_thresh = 1.128;

    // Initialize FaceDetectorYN
    Ptr<FaceDetectorYN> detector = FaceDetectorYN::create(fd_modelPath, "", Size(320, 320), scoreThreshold, nmsThreshold, topK);

    int frameWidth = int(640 * scale);
    int frameHeight = int(480 * scale);

    detector->setInputSize(Size(frameWidth, frameHeight));

    Mat faces;
    TickMeter tm;

    // -------------------------------------------------

    // Set up the video window and text data
    namedWindow("Irvy's View", cv::WINDOW_AUTOSIZE);
    string fps_text = "FPS: ";
    const int fps = 100;
    int frame_time = 1;
    uint16_t frameSize[1];
    int cur_fps;

    // vector<uchar> frameJpg;

    // Loop to get and display frames
    int cntr = 0;
    int run = 1;
    int nFrame = 0;
    while(run){
        auto start = high_resolution_clock::now();

        // Get frame size and create frameArr buffer of this size
        // read(socket, buffer(frameSize, 4));
        tcpCam.rcv(frameSize, 2);

        auto end = high_resolution_clock::now();
        
        // boost::asio::read(tcpCam.socket, boost::asio::buffer(frameSize, 2));
        uchar frameJpg[frameSize[0]];
        // boost::asio::read(tcpCam.socket, boost::asio::buffer(frameJpg, frameSize[0]));
        tcpCam.rcv(frameJpg, frameSize[0]);

        // cout << frameSize[0] << endl;
        // cout.write(frameSize,5);

        // uchar frameJpg[frameSize[0]];

        // Get entire image data compressed as jpeg, decode into cv::Mat
        // read(socket, buffer((frameArr), frameSize[0]));
        // tcpCam.rcv(frameArr, frameSize[0]);
        Mat frame = imdecode(Mat(1, frameSize[0], CV_8UC1, frameJpg), -1);

        // // Display image along with fps
        // tm.start();
        // detector->detect(frame, faces);
        // tm.stop();

        // Mat result = frame.clone();
        // // Draw results on the input image
        // visualize(result, nFrame, faces, tm.getFPS(), 2);

        imshow("Irvy's view", frame);//result);
        int key = waitKey(1);
        if(key == 27){
            destroyAllWindows();
            break;
        }

        ++nFrame;

        // auto end = high_resolution_clock::now();
        auto elapsed = end - start;
        frame_time = duration_cast<milliseconds>(elapsed).count();
        cntr++;
        cout << "Frame: " << cntr << endl;
        cout << "Elapsed Time: " << frame_time << endl;
    }
}

// void commander()
// {
//     cout << "Connecting to Control..." << endl;

//     // Set up the Boost ASIO IO service
//     boost::asio::io_context io_context;

//     // Create a TCP socket
//     tcp::socket socket(io_context);

//     // Set up a deadline timer to retry the connection if it fails
//     boost::asio::deadline_timer timer(io_context);

//     // Set the timeout for the connection attempt
//     timer.expires_from_now(boost::posix_time::seconds(5));

//     // Set up a flag to track the connection status
//     bool connected = false;

//     // Loop until the connection is established or the timer expires
//     while (!connected && timer.expires_from_now() > boost::posix_time::seconds(0)) {
//         try {
//             // Resolve the server address and port
//             tcp::resolver resolver(io_context);
//             auto endpoints = resolver.resolve(ip_addr, to_string(port+1));

//             // Attempt to connect to the server
//             boost::asio::connect(socket, endpoints);

//             // If the connection was successful, set the flag and break out of the loop
//             connected = true;
//             break;
//         } catch (const boost::system::system_error& e) {
//             // If the connection failed, print an error message and wait for the timer to expire
//             std::cerr << "Error connecting to server: " << e.what() << std::endl;
//             timer.wait();
//         }
//     }

//     // Check the connection status
//     if (connected) {
//         std::cout << "Connected to Control!" << std::endl;
//     } else {
//         std::cerr << "Timed out waiting for server." << std::endl;
//     }

//     // Init ncurses
//     initscr();
//     cbreak();
//     noecho();
//     char command[1];

//     // Loop to get and send commands
//     int run = 1;
//     while(run){
//         command[0] = getch();
//         write(socket, buffer(command, sizeof(char)));
//         if(command[0] == 'q')
//             run = 0;
//     }
//     endwin();
// }

static void visualize(Mat& input, int frame, Mat& faces, double fps, int thickness)
{
    std::string fpsString = cv::format("FPS : %.2f", (float)fps);
    for (int i = 0; i < faces.rows; i++) {
        // Draw bounding box
        rectangle(input, Rect2i(int(faces.at<float>(i, 0)), int(faces.at<float>(i, 1)), int(faces.at<float>(i, 2)), int(faces.at<float>(i, 3))), Scalar(0, 255, 0), thickness);
    }
    putText(input, fpsString, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
}