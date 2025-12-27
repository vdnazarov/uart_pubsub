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

namespace protocol {

using msg_edge = uint8_t;
static const constexpr msg_edge START_BYTE = 0x7e;
static const constexpr msg_edge STOP_BYTE = 0x7f;

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
public:
    Serial(const SerialSettings& settings)
        : send_to(settings.send_timeout_msec)
    {
        serial_port = open(settings.device.data(), O_RDWR | O_NOCTTY);
        if(serial_port < 0)
            throw uart_err("failed to open device " + settings.device);
        struct termios tty;
        tcgetattr(serial_port, &tty);
        cfsetospeed(&tty, settings.speed);
        cfsetispeed(&tty, settings.speed);
        tty.c_cflag &= ~PARENB; // No chetnost
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag |= CREAD | CLOCAL; // Read and write with ognoring of a control line
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = settings.recv_timeout_desisec; // 1 secod = 10

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_oflag &= ~OPOST; // prevent spesioan interruption for output bytes
        tty.c_oflag &= ~ONLCR; // Prevent convertion of newline ot carrige return

        tcsetattr(serial_port, TCSANOW, &tty);
    }

    ~Serial() noexcept(false)
    {
        if(close(serial_port) < 0)
            throw uart_err("Fauled to close UART");
    }

    bool lockWrite() { return true; }
    bool unlockWrite() { return true; }
    bool whaitRead()
    {
        usleep(100000);
        return true;
    }

    bool writeData(const std::string& data)
    {
        auto offset = 0;
        std::cout << "Sending " << data.size() << std::endl;
        while(offset < data.size())
        {
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
        }
        return true;
    }

    std::string readData()
    {
        std::string out;
        char buffer[1024*4];
        int n{0};
        do
        {
            n = read(serial_port, buffer, sizeof(buffer));
            if(n > 0)
                out.append(buffer, n);
            else
                throw uart_err("Failed to read from UART");
        }
        while(buffer[n-1] != *reinterpret_cast<const char*>(&STOP_BYTE));
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
    using msg_num_type = uint16_t;
    using checksumm_type = uint16_t;

    std::unique_ptr<Serial> serial;
    SerialSettings ssettings;

    msg_num_type msg_cout{0};
    std::queue<std::string> cache;
    std::mutex mutex;

    static checksumm_type checksumm(const std::string& data)
    {
        checksumm_type crc = 0xffff;
        for(auto byte: data)
        {
            crc ^= (uint8_t)byte << 8;
            for(int i=0; i<8; ++i)
                if(crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
        }
        return crc;
    }

    std::string formMsg(const std::string& payload, msg_type_type type = MSG_DATA)
    {
        std::string res;
        res.resize(sizeof(msg_size_type) + sizeof(msg_cout) + sizeof(type) + payload.size());
        size_t offset{0};

        msg_size_type size = payload.size();
        memcpy(res.data()+offset, &size, sizeof(size));
        offset += sizeof(size);

        memcpy(res.data()+offset, &msg_cout, sizeof(msg_cout));
        offset += sizeof(msg_cout);
        ++msg_cout;

        memcpy(res.data()+offset, &type, sizeof(type));
        offset += sizeof(type);

        memcpy(res.data()+offset, payload.data(), payload.size());
        offset += payload.size();

        auto ch = checksumm(res);
        res.insert(0, reinterpret_cast<const char*>(&START_BYTE), sizeof(START_BYTE));
        res.append(reinterpret_cast<const char*>(&ch), sizeof(ch));
        res.append(reinterpret_cast<const char*>(&STOP_BYTE), sizeof(STOP_BYTE));
        return res;
    }

    static std::string unwrapMsg(bool& ok, std::string& msg, msg_type_type& type, msg_num_type& seq)
    {
        ok = false;
        size_t offset{0};
        msg_edge start_b;
        memcpy(&start_b, msg.data(), sizeof(start_b));
        if(start_b != START_BYTE)
            return "Bad start byte";
        offset += sizeof(start_b);

        msg_size_type len;
        memcpy(&len, msg.data()+offset, sizeof(len));
        offset += sizeof(len);

        memcpy(&seq, msg.data()+offset, sizeof(seq));
        offset += sizeof(seq);

        memcpy(&type, msg.data()+offset, sizeof(type));

        switch (type)
        {
        case MSG_ACK:
        case MSG_DATA:
        case MSG_DONE:
        case MSG_ERROR:
        case MSG_NACK:
        case MSG_READY:
            break;
        default:
            return "unknown messgae type";
        }

        offset += sizeof(type);

        if(len+offset+sizeof(checksumm_type)+sizeof(STOP_BYTE) > msg.size())
            return "bad length";

        std::string payload(msg.data()+offset, len);
        offset += payload.size();

        checksumm_type ch;
        memcpy(&ch, msg.data()+offset, sizeof(ch));
        if(ch != checksumm({msg.data()+sizeof(START_BYTE), offset-sizeof(START_BYTE)}))
            return "checksumm failed";
        offset += sizeof(ch);

        memcpy(&start_b, msg.data()+offset, sizeof(start_b));
        if(start_b != STOP_BYTE)
            return "Bad end byte";
        offset += sizeof(start_b);

        ok = true;
        if(offset == msg.size())
            msg.clear();
        else
            msg.erase(0, offset);
        return payload;
    }

    void sendMessage(const std::string& payload, msg_type_type type = MSG_DATA)
    {
        if(!running)
            return;
        std::lock_guard<std::mutex> l(mutex);
        cache.push(formMsg(payload, type));
    }

    bool fullSend(const std::string& message)
    {
        assert(serial);
        int bad_count{0}, nack_count{0};
        while(bad_count < retry_cout)
        {
            {
                SerialLock l(serial.get());
                if(!serial->writeData(message))
                    return false;
            }
            if(!serial->whaitRead())
                return false;
            int i{0};
            std::string reply;
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
            msg_num_type num;
            unwrapMsg(ok, reply, rep_type, num);
            if(ok && rep_type == MSG_ACK)
                return true;
            if(ok)
            {
                if(rep_type != MSG_NACK)
                {
                    ++bad_count;
                    std::cerr << "Got unexpected messgae type, countinuing to resend anyway" << std::endl;
                }
                ++nack_count;
            }
            else
                ++bad_count;

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

    bool fullRecv(std::string& message, msg_type_type& type, msg_num_type& seq)
    {
        bool ok{false};
        while(!ok)
        {
            if(!serial->whaitRead())
                return false;
            int i{0};
            for(;i < retry_cout; ++i)
                try
                {
                    message = serial->readData();
                    break;
                }
                catch(const std::exception& e)
                {
                    continue;
                }
                catch(...)
                {
                    return false;
                }

            if(i >= retry_cout)
                return false;

            message = unwrapMsg(ok, message, type, seq);
            SerialLock l(serial.get());
            if(ok)
            {
                int i{0};
                for(;i<retry_cout; ++i)
                    if(serial->writeData(formMsg("", MSG_ACK)))
                        break;
                if(i >= retry_cout)
                    std::cout << "Failed to reply" << std::endl;
            }
            else
                serial->writeData(formMsg("", MSG_NACK));
        }
        return true;
    }

    std::future<void> worker;
    std::atomic_bool running{true};
    const int retry_cout;
    std::string last_error;
    bool started{false};
    bool server;
    Protocol::ClientPollAction on_recv_actor;

    class ProtocolError: public std::runtime_error
    {
    public:
        ProtocolError(const std::string& err)
            : std::runtime_error("Protocol error: " + err)
        {}
    };

    ProtocolPrivate(const SerialSettings& settings, int retry_count, bool server)
        : ssettings(settings),
          retry_cout(retry_count),
          server(server)
    {}
};

Protocol::Protocol(const SerialSettings& settings, int retry_count, bool server)
    : p(new ProtocolPrivate(settings, retry_count, server))
{
    p->serial.reset(new Serial(settings));
    if(server)
    {
        p->worker = std::async(std::launch::async, [this]()
        {
            bool queue_empty{true};
            while(p->running || !queue_empty)
            {
                {
                    std::lock_guard<std::mutex> l(p->mutex);
                    if(!p->cache.empty())
                    {
                        if(p->cache.front().empty())
                        {
                            p->running = false;
                            return;
                        }
                        if(!p->fullSend(p->cache.front()))
                        {
                            p->last_error = "Protocol failed";
                            p->running = false;
                            return;
                        }
                        p->cache.pop();
                    }
                    queue_empty = p->cache.empty();
                }
                if(p->running && queue_empty)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    if(!p->last_error.empty())
        std::cerr << p->last_error << std::endl;
    delete p;
}

void Protocol::poll(ClientPollAction action)
{
    p->on_recv_actor = action;
    p->worker = std::async(std::launch::async, [this]()
    {
        std::string msg;
        msg_type_type type;
        ProtocolPrivate::msg_num_type num;
        while(p->running)
        {
            auto last = num;
            if(!p->fullRecv(msg, type, num))
            {
                p->serial.reset(nullptr);
                p->serial.reset(new Serial(p->ssettings));
                continue;
            }
            if(last != num)
                p->on_recv_actor(msg, type);
        }
    });
}

void Protocol::start()
{
    if(p->started)
        throw ProtocolPrivate::ProtocolError("attempt to ready multiple times");
    p->sendMessage("", MSG_READY);
    p->started = true;
}

void Protocol::sendPayload(const std::string& message, bool is_error)
{
    if(!p->started)
        throw ProtocolPrivate::ProtocolError("attempt to send messgae before ready");
    p->sendMessage(message, is_error ? MSG_ERROR : MSG_DATA);
}

void Protocol::stop()
{
    if(p->server)
    {
        p->running = false;
        return;
    }
    if(!p->started)
        throw ProtocolPrivate::ProtocolError("attempt to done defore ready");
    p->sendMessage("", MSG_DONE);
    std::lock_guard<std::mutex> l(p->mutex);
    p->cache.push("");
    p->started = false;
}

bool Protocol::status()
{
    return p->running;
}

std::string Protocol::lastError()
{
    std::lock_guard<std::mutex> l(p->mutex);
    return p->last_error;
}

}
