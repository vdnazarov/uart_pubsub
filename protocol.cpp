// Copyright (c) 2025 Nazarov Vsevolod
// This code is licensed under the MIT License. See LICENSE.md for details.

#include "protocol.h"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <queue>
#include <future>
#include <mutex>
#include <atomic>
#include <memory>
#include <cstring>
#include <assert.h>
#include <thread>
#include <wiringPi.h>


namespace gpio {

class GpioOutPin
{
    const int num;
public:
    GpioOutPin(int n)
        : num(n)
    {}

    bool open()
    {
        if(wiringPiSetupGpio() == -1)
            return false;
        pinMode(num, OUTPUT);
        return true;
    }

    bool setVaue(bool v)
    {
        pinMode(num, v ? HIGH : LOW);
        return true;
    }
};

}

namespace protocol {

using msg_edge = uint8_t;
static const constexpr msg_edge START_BYTE = 0x7e;
static const constexpr msg_edge STOP_BYTE = 0x7f;
using raw_data = std::vector<uint8_t>;

class uart_err: public std::runtime_error
{
public:
    uart_err(const std::string& err)
        : std::runtime_error("UART error: " + err)
    {}
};

class Serial
{
    int serial_port;
    int send_to;
    gpio::GpioOutPin ctrl;
public:
    Serial(const SerialSettings& settings)
        : send_to(settings.send_timeout_msec),
          ctrl(settings.control_pin)
    {
        if(!ctrl.open())
            throw uart_err("Failed to open cotrol pin");
        serial_port = open(settings.device.data(), O_RDWR | O_NOCTTY | O_SYNC);
        if(serial_port < 0)
            throw uart_err("failed to open device " + settings.device);
        struct termios tty;
        tcgetattr(serial_port, &tty);

        // speed
        cfsetospeed(&tty, settings.speed);
        cfsetispeed(&tty, settings.speed);

        // raw non canonical mode
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
        tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        tty.c_cflag |= (CS8 | CREAD | CLOCAL); // Read and write with ognoring of a control line and 8 bits per byte
        tty.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);
        tty.c_oflag &= ~(OPOST | ONLCR); // prevent spesioan interruption for output bytes


        // recv timeout
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = settings.recv_timeout_desisec; // 1 secod = 10

        tcsetattr(serial_port, TCSANOW, &tty);
        ctrl.setVaue(false);
    }

    ~Serial() noexcept(false)
    {
        if(close(serial_port) < 0)
            throw uart_err("Fauled to close UART");
    }

    bool lockWrite()
    {
        try
        {
            return ctrl.setVaue(true);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
    }
    bool unlockWrite()
    {
        try
        {
            tcdrain(serial_port);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return ctrl.setVaue(false);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
    }
    bool whaitRead()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return true;
    }

    bool whaitWrite()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        return true;
    }

    bool writeData(const raw_data& data)
    {
        auto offset = 0;
        std::cout << "Sending " << data.size() << std::endl;
        while(offset < data.size())
        {
            tcdrain(serial_port);
            if(send_to > 0)
            {
                fd_set write_fds;
                struct timeval timeout;
                FD_ZERO(&write_fds);
                FD_SET(serial_port, &write_fds);
                timeout.tv_sec = 0;
                timeout.tv_usec = send_to*10e6;
                bool done{false};
                while(!done)
                    switch (select(serial_port+1, NULL, &write_fds, NULL, &timeout))
                    {
                    case EINTR:
                        continue;
                    case 0:
                    {
                        std::cerr << "Write timeout" << std::endl;
                        return false;
                    }
                    default:
                    {
                        done = true;
                        break;
                    }
                    }

            }
            auto res = write(serial_port, data.data()+offset, data.size()-offset);
            if(res < 0)
                return false;
            offset += res;
            std::cout << offset << ":" << data.size() << std::endl;
        }
        return tcdrain(serial_port) == 0;
    }

    raw_data readData()
    {
        raw_data out;
        raw_data buffer(1024*9);
        int n{0};
        do
        {
            n = read(serial_port, buffer.data(), buffer.size());
            if(n > 0)
                for(int i=0; i<n; ++i)
                    out.push_back(buffer[i]);
            else
                throw uart_err("Failed to read from UART");
        }
        while(buffer[n-1] != STOP_BYTE);
        std::cout << "Got " << out.size() << std::endl;
        return out;
    }
};

class SerialLock
{
    Serial* s;
public:
    SerialLock(Serial* s)
        : s(s)
    {
        if(!s->lockWrite())
            throw uart_err("Failed to lock UART for writing");
    }

    ~SerialLock() noexcept(false)
    {
        if(!s->unlockWrite())
            throw uart_err("Failed to unlock UART for writing");
    }
};

class ProtocolPrivate
{
public:
    std::queue<std::string> frames_queue;
    using msg_size_type = uint16_t;
    using checksumm_type = uint32_t;

    std::unique_ptr<Serial> serial;
    SerialSettings ssettings;

    std::queue<std::vector<raw_data>> cache;
    std::mutex mutex;

    static uint32_t crc32(const std::vector<uint8_t>& data)
    {
        uint32_t crc = 0xFFFFFFFF;
        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0xEDB88320;
                else
                    crc >>= 1;
            }
        }
        return ~crc;
    }

    std::vector<raw_data> formMsg(msg_type_type type)
    {
        return formMsg({""}, type);
    }

    std::vector<raw_data> formMsg(const std::vector<std::string>& payloads, msg_type_type type)
    {
        std::vector<raw_data> list;
        msg_part_type of = payloads.size();
        for(int i=0; i<payloads.size(); ++i)
        {
            raw_data payload(payloads[i].size());
            memcpy(payload.data(), payloads[i].data(), payload.size());
            raw_data res;
            auto total = sizeof(type);
            if(type == MSG_DATA)
                total += sizeof(msg_size_type) + sizeof(msg_part_type)*2 + payload.size();
            res.resize(total);
            size_t offset{0};

            memcpy(res.data()+offset, &type, sizeof(type));
            offset += sizeof(type);

            if(type == MSG_DATA)
            {
                msg_size_type size = payload.size();
                memcpy(res.data()+offset, &size, sizeof(size));
                offset += sizeof(size);

                msg_part_type part = i+1;
                memcpy(res.data()+offset, &part, sizeof(part));
                offset += sizeof(part);

                memcpy(res.data()+offset, &of, sizeof(of));
                offset += sizeof(of);

                memcpy(res.data()+offset, payload.data(), payload.size());
                offset += payload.size();
            }

            auto ch = crc32(res);
            raw_data chv(sizeof(ch));
            memcpy(chv.data(), &ch, sizeof(ch));
            res.insert(res.begin(), START_BYTE);
            for(auto v: chv)
                res.push_back(v);
            res.push_back(STOP_BYTE);
            list.push_back(res);
        }
        return list;
    }

    static std::string unwrapMsg(bool& ok, raw_data& msg, msg_type_type& type, msg_part_type& part, msg_part_type& of)
    {
        ok = false;
        size_t offset{0};
        msg_edge start_b;
        memcpy(&start_b, msg.data(), sizeof(start_b));
        if(start_b != START_BYTE)
            return "Bad start byte";
        offset += sizeof(start_b);

        memcpy(&type, msg.data()+offset, sizeof(type));

        switch (type)
        {
        case MSG_DATA:
            std::cout << "data";
            break;
        case MSG_ACK:
            std::cout << "act";
            break;
        case MSG_NACK:
            std::cout << "nack";
            break;
        case MSG_MNACK:
            std::cout << "mnack";
            break;
        case MSG_READY:
            std::cout << "ready";
            break;
        case MSG_DONE:
            std::cout << "done";
            break;
        case MSG_PING:
            std::cout << "ping";
            break;
        default:
            return "unknown message";
        }
        offset += sizeof(type);


        std::string payload;
        if(type == MSG_DATA)
        {
            if(offset+sizeof(msg_part_type)*2+sizeof(checksumm_type)+sizeof(STOP_BYTE) > msg.size())
            {
                std::cout << std::endl;
                return "bad length";
            }

            msg_size_type len;
            memcpy(&len, msg.data()+offset, sizeof(len));
            offset += sizeof(len);

            if(len+offset+sizeof(msg_part_type)*2+sizeof(checksumm_type)+sizeof(STOP_BYTE) > msg.size())
            {
                std::cout << std::endl;
                return "bad length";
            }

            memcpy(&part, msg.data()+offset, sizeof(part));
            offset += sizeof(part);

            memcpy(&of, msg.data()+offset, sizeof(of));
            offset += sizeof(of);
            std::cout << " " << part << " of " << of << std::endl;

            payload.resize(len);
            memcpy(payload.data(), msg.data()+offset, len);
            offset += len;
        }
        else
        {
            if(offset+sizeof(checksumm_type)+sizeof(STOP_BYTE) > msg.size())
                return "bad length";
            part = 1;
            of = 1;
        }

        checksumm_type ch;
        memcpy(&ch, msg.data()+offset, sizeof(ch));
        raw_data cspl(offset-sizeof(START_BYTE));
        memcpy(cspl.data(), msg.data()+sizeof(START_BYTE), offset-sizeof(START_BYTE));

        auto rchs = crc32(cspl);

        if(ch != rchs)
        {
            std::cout << ch << " " << rchs << std::endl;
            return "checksumm failed for " + payload+"|"+std::to_string(payload.size());
        }
        offset += sizeof(ch);

        memcpy(&start_b, msg.data()+offset, sizeof(start_b));
        if(start_b != STOP_BYTE)
            return "Bad end byte";
        offset += sizeof(start_b);

        ok = true;
        msg.clear();
        return payload;
    }


    void sendMessage(msg_type_type type)
    {
        sendMessage({""}, type);
    }

    void sendMessage(const std::vector<std::string>& payloads, msg_type_type type)
    {
        if(!running)
            return;
        std::cout << "Sending ";
        switch (type)
        {
        case MSG_DATA:
            std::cout << "data";
            break;
        case MSG_ACK:
            std::cout << "act";
            break;
        case MSG_NACK:
            std::cout << "nack";
            break;
        case MSG_MNACK:
            std::cout << "mnack";
            break;
        case MSG_READY:
            std::cout << "ready";
            break;
        case MSG_DONE:
            std::cout << "done";
            break;
        case MSG_PING:
            std::cout << "ping";
            break;
        default:
            std::cout << "unknown";
            break;
        }
        std::cout << std::endl;
        std::vector<raw_data> mp_q;
        for(const auto& msg: formMsg(payloads, type))
            mp_q.push_back(msg);
        std::lock_guard<std::mutex> l(mutex);
        cache.push(mp_q);
    }

    bool fullSend(const std::vector<raw_data>& mp_q)
    {
        int bad_count{0};
        bool mp_failed;
        while(bad_count < retry_cout)
        {
            for(const auto& msg: mp_q)
                if(!fullSend(msg, mp_failed))
                {
                    if(!mp_failed)
                        return false;
                    break;
                }
            if(!mp_failed)
                return true;
        }
        return false;
    }

    bool fullSend(const raw_data& message, bool& mpart_failed)
    {
        mpart_failed = false;
        assert(serial);
        int bad_count{0}, nack_count{0};
        serial.reset(nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        serial.reset(new Serial(ssettings));
        while(bad_count < retry_cout)
        {
            if(!serial->whaitWrite())
                return false;
            {
                SerialLock l(serial.get());
                if(!serial->writeData(message))
                    return false;
            }
            if(!serial->whaitRead())
                return false;
            int i{0};
            raw_data reply;
            for(; i<retry_cout; ++i)
                try
                {
                    reply = serial->readData();
                    break;
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << std::endl;
                }
                catch(...)
                {
                    return false;
                }
            if(reply.empty())
            {
                ++bad_count;
                continue;
            }
            bool ok;
            msg_type_type rep_type;
            msg_part_type part, of;
            unwrapMsg(ok, reply, rep_type, part, of);
            if(part != of && part != 1)
            {
                std::cerr << "bad parts" << std::endl;
                ++bad_count;
            }
            else
            {
                if(ok && rep_type == MSG_ACK)
                    return true;
                if(ok)
                {
                    if(rep_type != MSG_NACK && rep_type != MSG_MNACK)
                    {
                        ++bad_count;
                        std::cerr << "Got unexpected messgae type, countinuing to resend anyway" << std::endl;
                    }
                    if(rep_type == MSG_MNACK)
                    {
                        mpart_failed = true;
                        return false;
                    }
                    ++nack_count;
                }
                else
                    ++bad_count;
            }

            if(nack_count >= retry_cout)
            {
                serial.reset(nullptr);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                serial.reset(new Serial(ssettings));
                nack_count = 0;
            }
        }
        return false;
    }

    bool fullRecv(std::string& message, msg_type_type& type, Protocol::ClientErrorCallback error_callback, bool& aborted)
    {
        message.clear();
        msg_part_type part{0}, last_part{0}, of{1};
        msg_type_type last_type;
        while(part < of)
        {
            std::string section;
            bool ok{false};
            while(!ok)
            {
                if(!serial->whaitRead())
                {
                    aborted = error_callback(Protocol::EmptyError, "no data");
                    return false;
                }
                int i{0};
                raw_data recv;
                for(;i < retry_cout; ++i)
                    try
                    {
                        recv = serial->readData();
                        break;
                    }
                    catch(const std::exception& e)
                    {
                        aborted = !error_callback(Protocol::ReadError, e.what());
                        if(aborted)
                            return false;
                    }
                    catch(...)
                    {
                        aborted = !error_callback(Protocol::ReadError, "unknown error");
                        return false;
                    }

                if(i >= retry_cout)
                {
                    aborted = !error_callback(Protocol::RetryError, "Failed to read from UART");
                    return false;
                }

                section = unwrapMsg(ok, recv, type, part, of);
                if(!serial->whaitWrite())
                {
                    aborted = !error_callback(Protocol::WriteError, "failed to initialise reply");
                    return false;
                }
                SerialLock l(serial.get());
                if(!ok)
                {
                    aborted = !error_callback(Protocol::MsgError, section);
                    if(aborted)
                        return false;
                    std::cerr << "Sending NACK: " << serial->writeData(formMsg(MSG_NACK).front()) << std::endl;
                }
                else
                {
                    bool mp_failed{false};
                    if(last_part+1 != part)
                    {
                        aborted = !error_callback(Protocol::MutipartError, "part of multipart message is missing");
                        if(aborted)
                            return false;
                        mp_failed = true;
                    }
                    if((last_part > 0 && last_type != type) || part > of)
                    {
                        aborted = !error_callback(Protocol::MutipartError, "part types missmacth or last parts are missing");
                        if(aborted)
                            return false;
                        mp_failed = true;
                    }
                    int i{0};
                    for(;i<retry_cout*10; ++i)
                        if(serial->writeData(formMsg(mp_failed ? MSG_MNACK : MSG_ACK).front()))
                            break;
                    if(i >= retry_cout*10)
                    {
                        aborted = !error_callback(Protocol::RetryError, "failed to reply");
                        if(!aborted)
                            return false;
                        std::cout << "Failed to reply" << std::endl;
                    }
                    if(mp_failed)
                        return false;
                }
            }

            last_type = type;
            last_part = part;
            message.append(section);
            std::cout << "part recv end" << std::endl;
        }
        return true;
    }

    std::future<void> worker;
    std::atomic_bool running{true};
    const int retry_cout;
    bool started{false};
    Protocol::ClientPollAction on_recv_actor;
    Protocol::ClientErrorCallback on_error_act;

    class ProtocolError: public std::runtime_error
    {
    public:
        ProtocolError(const std::string& err)
            : std::runtime_error("Protocol error: " + err)
        {}
    };

    ProtocolPrivate(const SerialSettings& settings, int retry_count)
        : ssettings(settings),
          retry_cout(retry_count)
    {}
};

Protocol::Protocol(const SerialSettings& settings, int retry_count, bool server)
    : p(new ProtocolPrivate(settings, retry_count))
{
    p->serial.reset(new Serial(settings));
    if(server)
    {
        p->worker = std::async(std::launch::async, [this]()
        {
            size_t queue_size{1};
            while(p->running || queue_size > 0)
            {
                std::vector<raw_data> mp_q;
                {
                    std::lock_guard<std::mutex> l(p->mutex);
                    if(!p->cache.empty())
                    {
                        if(p->cache.front().empty())
                        {
                            std::cout << "protocol queue: done" << std::endl;
                            p->running = false;
                            return;
                        }
                        mp_q = p->cache.front();
                        p->cache.pop();
                    }
                    else
                        mp_q = p->formMsg(MSG_PING);
                    queue_size = p->cache.size();
                }
                auto start = std::chrono::high_resolution_clock::now();
                if(!p->fullSend(mp_q))
                {
                    p->running = false;
                    std::cerr << "Protocol failed" << std::endl;
                    return;
                }
                else if(queue_size > 0)
                    std::cout << "protocol queue: " << queue_size << std::endl;

                int t_offset = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
                static const constexpr int sleep_time = 200;
                if(p->running && queue_size <= 0)
                    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(sleep_time - t_offset, 100)));
            }
        });
    }
}

Protocol::~Protocol() noexcept(false)
{
    if(p->started)
    {
        stop();
        usleep(10000);
    }
    p->worker.wait();
    delete p;
}

void Protocol::poll(ClientPollAction action, ClientErrorCallback error_cb)
{
    p->on_recv_actor = action;
    p->on_error_act = error_cb;
    p->worker = std::async(std::launch::async, [this]()
    {
        std::string msg;
        msg_type_type type;
        while(p->running)
        {
            bool aborted;
            if(!p->fullRecv(msg, type, p->on_error_act, aborted))
            {
                if(aborted)
                {
                    p->running = false;
                    return;
                }
                p->serial.reset(nullptr);
                p->serial.reset(new Serial(p->ssettings));
                continue;
            }
            if(!p->on_recv_actor(msg, type))
            {
                p->running = false;
                return;
            }
        }
    });
}

bool Protocol::polling()
{
    return p->running;
}

void Protocol::start()
{
    if(p->started)
        throw ProtocolPrivate::ProtocolError("attempt to ready multiple times");
    p->sendMessage(MSG_READY);
    p->started = true;
}

void Protocol::sendPayload(const std::string& message)
{
    if(!p->started)
        throw ProtocolPrivate::ProtocolError("attempt to send messgae before ready");
    std::vector<std::string> msgs;
    for(size_t i=0; i<message.size(); i+=p->ssettings.msg_part_max_size)
        msgs.push_back(message.substr(i, p->ssettings.msg_part_max_size));
    p->sendMessage(msgs, MSG_DATA);
}

void Protocol::stop()
{
    if(!p->started)
    {
        p->running = false;
        return;
    }
    p->sendMessage(MSG_DONE);
    std::lock_guard<std::mutex> l(p->mutex);
    p->cache.push({});
    p->started = false;
}

void Protocol::whaitAllSend()
{
    bool queue_empty{false};
    while(p->running && !queue_empty)
    {
        {
            std::lock_guard<std::mutex> l(p->mutex);
            queue_empty = p->cache.empty();
        }
        if(!queue_empty)
            std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool Protocol::status()
{
    return p->running;
}

}
