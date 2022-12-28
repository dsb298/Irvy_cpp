#ifndef _TCP_CLIENT_
#define _TCP_CLIENT_

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <vector>

using boost::asio::ip::tcp;

class TcpClient
{
public:
    TcpClient(std::string ip, size_t port) : resolver(io_context), socket(io_context) {
        endpoints = resolver.resolve(ip, std::to_string(port));
        boost::asio::connect(socket, endpoints);
    }

    template<class T>
    void send(T msg){
        boost::asio::write(socket, boost::asio::buffer(msg));
    }

    inline void rcv_frame(std::vector<uchar> frameJpg){
        boost::asio::read(this->socket, boost::asio::buffer(frameSize, 2));
        boost::asio::read(this->socket, boost::asio::buffer(frameJpg, frameSize[0]));
        std::cout << frameSize[0] << std::endl;
    }

    template<class T>
    inline void rcv(T buf, size_t nBytes){
        boost::asio::read(this->socket, boost::asio::buffer(buf, nBytes));
    }

// private:
    boost::asio::io_context io_context;
    tcp::resolver resolver;
    tcp::resolver::results_type endpoints;
    tcp::socket socket;
    uint16_t frameSize[1];
};

#endif