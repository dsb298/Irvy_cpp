#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

#include <ncurses.h>
#include <unistd.h>

void display();
void commander();

using boost::asio::ip::tcp;
using namespace boost::asio;
using namespace std::chrono;
using namespace std;
using namespace cv;

const string ip_addr = "192.168.1.162";
const int port = 4789;

int main()
{
    int result = nice(1);
    if (result == -1) {
        std::cerr << "Error setting priority" << std::endl;
        return 1;
    }

    thread display_thread(display);
    thread command_thread(commander);

    display_thread.join();
    command_thread.join();

    return 0;
}

void display()
{
    cout << "Connecting to Display..." << endl;

    // Create a client endpoint
    io_context service;
    ip::tcp::endpoint endpoint(ip::address::from_string(ip_addr), port);

    // Connect to the server
    ip::tcp::socket socket(service);
    socket.connect(endpoint);

    cout << "Connected to Display!" << endl;

    // Set up the video window and text data
    namedWindow("Irvy's View", cv::WINDOW_AUTOSIZE);
    string fps_text = "FPS: ";
    const int fps = 100;
    int frame_time = 1;
    int frameSize[1];
    int cur_fps;

    // Loop to get and display frames
    int run = 1;
    while(run){
        auto start = high_resolution_clock::now();

        // Get frame size and create frameArr buffer of this size
        read(socket, buffer(frameSize, 4));
        uchar frameArr[frameSize[0]];

        // Get entire image data compressed as jpeg, decode into cv::Mat
        read(socket, buffer((frameArr), frameSize[0]));
        Mat frame = imdecode(Mat(1, frameSize[0], CV_8UC1, frameArr), -1);

        // Display image along with fps
        if(frame_time!=0){
            cur_fps = 1000/frame_time;
            fps_text = "FPS: " + to_string(cur_fps);
        }
        putText(frame, fps_text, Point(10, 20), FONT_HERSHEY_SIMPLEX, .6, Scalar(0, 255, 255), 2);
        imshow("Irvy's View", frame);
        char c = waitKey(1000/fps);
        if(c == 27){
            destroyAllWindows();
            run = 0;
        }
        auto end = high_resolution_clock::now();
        auto elapsed = end - start;
        frame_time = duration_cast<milliseconds>(elapsed).count();
    }
}

void commander()
{
    cout << "Connecting to Control..." << endl;

    // Set up the Boost ASIO IO service
    boost::asio::io_context io_context;

    // Create a TCP socket
    tcp::socket socket(io_context);

    // Set up a deadline timer to retry the connection if it fails
    boost::asio::deadline_timer timer(io_context);

    // Set the timeout for the connection attempt
    timer.expires_from_now(boost::posix_time::seconds(5));

    // Set up a flag to track the connection status
    bool connected = false;

    // Loop until the connection is established or the timer expires
    while (!connected && timer.expires_from_now() > boost::posix_time::seconds(0)) {
        try {
            // Resolve the server address and port
            tcp::resolver resolver(io_context);
            auto endpoints = resolver.resolve(ip_addr, to_string(port));

            // Attempt to connect to the server
            boost::asio::connect(socket, endpoints);

            // If the connection was successful, set the flag and break out of the loop
            connected = true;
            break;
        } catch (const boost::system::system_error& e) {
            // If the connection failed, print an error message and wait for the timer to expire
            std::cerr << "Error connecting to server: " << e.what() << std::endl;
            timer.wait();
        }
    }

    // Check the connection status
    if (connected) {
        std::cout << "Connected to Control!" << std::endl;
    } else {
        std::cerr << "Timed out waiting for server." << std::endl;
    }

    // Init ncurses
    initscr();
    cbreak();
    noecho();
    char command[1];

    // Loop to get and send commands
    int run = 1;
    while(run){
        command[0] = getch();
        write(socket, buffer(command, sizeof(char)));
        if(command[0] == 'q')
            run = 0;
    }
    endwin();
}