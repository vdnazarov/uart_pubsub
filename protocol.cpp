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
#include <fstream>

namespace gpio
{

class GpioOutPin
{
    std::string number;
    bool opened{false};
    bool valid{false};

    std::string pinDir() const
    {
        return "/sys/class/gpio/gpio" + number;
    }

    std::string valueFile() const
    {
        return pinDir() + "/value";
    }

    bool unexport()
    {
        if(!valid)
            return true;
        opened = false;
        std::ofstream f("/sys/class/gpio/unexport", std::ios::out);
        if(!f.is_open())
            return false;

        if(!f.write(number.data(), number.size()))
        {
            f.close();
            return false;
        }
        f.close();
        return true;
    }
public:
    GpioOutPin(unsigned int number)
        : number(std::to_string(number))
    {
        opened = true;
        valid = number > 0;
        close();
    }

    ~GpioOutPin()
    {
        close();
    }

    bool open()
    {
        if(!valid)
            return true;
        std::ofstream f("/sys/class/gpio/export", std::ios::out);
        if(!f.is_open())
            return false;

        if(!f.write(number.data(), number.size()))
            return false;

        f.close();

        f.open(pinDir()+ "/direction", std::ios::out);
        if(!f.is_open())
        {
            unexport();
            return false;
        }
        if(!f.write("out", 3))
        {
            f.close();
            unexport();
            return false;
        }

        f.close();
        opened = true;
        return true;
    }

    bool close()
    {
        if(!valid)
            return true;
        if(!opened)
            return true;
        std::ofstream f(pinDir() + "/direction", std::ios::out);
        if(!f.is_open())
        {
            unexport();
            return false;
        }
        if(!f.write("in" , 2))
        {
            f.close();
            unexport();
            return false;
        }
        return unexport();
    }

    bool setVaue(bool v)
    {
        if(!valid)
            return true;
        if(!opened)
            return false;
        std::ofstream f(valueFile(), std::ios::out);
        if(!f.is_open())
            return false;
        auto sv = std::to_string(v ? 1 : 0);
        if(!f.write(sv.data(), sv.size()))
        {
            f.close();
            return false;
        }
        f.close();
        return true;
    }
};

}

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
        cfsetospeed(&tty, settings.speed);
        cfsetispeed(&tty, settings.speed);
        tty.c_cflag &= ~PARENB; // No chetnost
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag |= (CREAD | CLOCAL); // Read and write with ognoring of a control line
        tty.c_cflag &= ~CRTSCTS; // No flow control
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = settings.recv_timeout_desisec; // 1 secod = 10

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no xon/xoff

        tty.c_oflag &= ~OPOST; // prevent spesioan interruption for output bytes
        tty.c_oflag &= ~ONLCR; // Prevent convertion of newline ot carrige return

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
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return true;
    }

    bool writeData(const std::string& data)
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
        }
        return tcdrain(serial_port) == 0;
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

        std::string payload;
        payload.resize(len);
        memcpy(payload.data(), msg.data()+offset, len);
        offset += len;

        checksumm_type ch;
        memcpy(&ch, msg.data()+offset, sizeof(ch));
        std::string cspl;
        cspl.resize(offset-sizeof(START_BYTE));
        memcpy(cspl.data(), msg.data()+sizeof(START_BYTE), offset-sizeof(START_BYTE));

        if(ch != checksumm(cspl))
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
        case MSG_ERROR:
            std::cout << "error";
            break;
        case MSG_READY:
            std::cout << "ready";
            break;
        case MSG_DONE:
            std::cout << "done";
            break;
        default:
            std::cout << "unknown";
            break;
        }
        std::cout << std::endl;
        std::lock_guard<std::mutex> l(mutex);
        cache.push(formMsg(payload, type));
    }

    bool fullSend(const std::string& message)
    {
        assert(serial);
        int bad_count{0}, nack_count{0};
        serial.reset(nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        serial.reset(new Serial(ssettings));
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
                    //std::cerr << e.what() << std::endl;
                }
                catch(...)
                {
                    return false;
                }

            if(i >= retry_cout)
                return false;

            message = unwrapMsg(ok, message, type, seq);
            if(!serial->whaitWrite())
                return false;
            SerialLock l(serial.get());
            if(!ok)
            {
                std::cerr << message << std::endl;
                std::cerr << "Sending NACK: " << serial->writeData(formMsg("", MSG_NACK)) << std::endl;
            }
            else
            {
                int i{0};
                for(;i<retry_cout*10; ++i)
                    if(serial->writeData(formMsg("", MSG_ACK)))
                        break;
                if(i >= retry_cout*10)
                {
                    std::cout << "Failed to reply" << std::endl;
                }
            }
        }
        return true;
    }

    std::future<void> worker;
    std::atomic_bool running{true};
    const int retry_cout;
    std::string last_error;
    bool started{false};
    Protocol::ClientPollAction on_recv_actor;

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
    if(!p->started)
    {
        p->running = false;
        return;
    }
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
