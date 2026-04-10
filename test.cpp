/*
 * Copyright (c) 2025 Nazarov Vsevolod
 * This code is licensed under the MIT License. See LICENSE.md for details.
 */

#include "protocol.h"

#include <signal.h>
#include <iostream>
#include <thread>

bool running{true};

void signalHandler(int sig)
{
    std::cout << "Got signal " << sig << std::endl;
    running = false;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    protocol::SerialSettings s;
    s.device = "/dev/ttyAMA0";
    s.recv_timeout_desisec = 100;
    s.control_pin = 34;
    protocol::Protocol p(s, 3, false);
    p.poll([](const std::string& msg, protocol::msg_type_type type) -> bool
    {
        switch(type)
        {
        case protocol::MSG_READY:
            std::cout << "Server ready" << std::endl;
            break;
        case protocol::MSG_DATA:
            std::cout << "Got data " << msg.size() << std::endl;
            break;
        case protocol::MSG_DONE:
            std::cout << "Server done" << std::endl;
            break;
        case protocol::MSG_PING:
            std::cout << "Got ping" << std::endl;
            break;
        default:
            std::cerr << "Got unexpected message type " << type << std::endl;
            return false;
        }
        return running;
    },[](protocol::Protocol::PollError et, const std::string& err) -> bool
    {
        switch(et)
        {
        case protocol::Protocol::PollError::EmptyError:
            std::cerr << "empty error: ";
            break;
        case protocol::Protocol::PollError::MsgError:
            std::cerr << "message error: ";
            break;
        case protocol::Protocol::PollError::MutipartError:
            std::cerr << "multipart error: ";
            break;
        case protocol::Protocol::PollError::ReadError:
            std::cerr << "read error: ";
            break;
        case protocol::Protocol::PollError::RetryError:
            std::cerr << "retry error: ";
            break;
        case protocol::Protocol::PollError::WriteError:
            std::cerr << "write error: ";
            break;
        }
        std::cerr << err << std::endl;
        return running;
    });
    while(running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    p.stop();
    return 0;
}
