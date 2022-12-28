#ifndef _TCP_SERVER_
#define _TCP_SERVER_

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <vector>

using boost::asio::ip::tcp;

class TcpServer
{
public:
    TcpServer(size_t port) : acceptor(context, tcp::endpoint(tcp::v4(), port)), socket(context){
        acceptor.accept(socket);
    }

    // void send_frame(uint16_t* frameJpg){
    //     // Send frame size first, then send frame
    //     boost::asio::write(socket, boost::asio::buffer(frameJpg.size()));
    //     // std::cout << frameJpg.size() << std::endl;
    //     // std::cout << std::to_string(frameJpg.size()) << std::endl;
    //     // boost::asio::write(socket, boost::asio::buffer(frameJpg, sizeof(frameJpg)));
    // }

    // Send frame size first, then send frame
    inline void send_frame(std::vector<uchar> frameJpg){
        frameSize[0] = frameJpg.size();
        boost::asio::write(this->socket, boost::asio::buffer(frameSize));
        boost::asio::write(this->socket, boost::asio::buffer(frameJpg));
    }

    template<class T>
    void rcv(T buf, size_t nBytes){
        boost::asio::read(this->socket, boost::asio::buffer(buf, nBytes));
    }

private:
    boost::asio::io_context context;
    tcp::acceptor acceptor;
    tcp::socket socket;
    uint16_t frameSize[1];
};

#endif