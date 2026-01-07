/*
 * Copyright (c) 2025 Nazarov Vsevolod
 * This code is licensed under the MIT License. See LICENSE.md for details.
 */

#include "protocol.h"

#include <signal.h>
#include <iostream>

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
        case protocol::MSG_ERROR:
            std::cerr << "Got error: " << msg << std::endl;
            break;
        default:
            std::cerr << "Got unexpected message type " << type << std::endl;
            return false;
        }
        return true;
    });
    while(running)
        usleep(100000);
    p.stop();
    return 0;
}
