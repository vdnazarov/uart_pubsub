// Copyright (c) 2025 Nazarov Vsevolod
// This code is licensed under the MIT License. See LICENSE.md for details.

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>
#include <termios.h>
#include <functional>
#include <cstdint>

/*!
 * Namespace for all protocol UART PUB SUB protocol
 * classes
 *
 * Protocol implenets simple PUB SUB pattern with
 * comnfirmation of acceptance.
 *
 * Server (PUB) sends mesages and client (SUB) receives them
 */
namespace protocol {


/*!
 * \brief Serial port settings structure
 */
struct SerialSettings
{
    /// \brief Serial port device name
    std::string device;

    /// \brief Serial port bound rate
    speed_t speed{B115200};

    /// \brief Message send timeout, msec
    int send_timeout_msec{50};

    /// \brief Message recv timeout, decsec (1/10 of a second)
    int recv_timeout_desisec{20};

    /// \brief RS485 control GPIO pin
    int control_pin{34};

    /// \brief Max multipart payload size in bytes
    size_t msg_part_max_size = 10240;
};

using msg_type_type = uint8_t;
using msg_part_type = uint16_t;

/// \brief Protocol start message
static const constexpr msg_type_type MSG_READY = 0x01;

/// \brief Message type for good message
static const constexpr msg_type_type MSG_ACK = 0x02;

/// \brief Message type for good message
static const constexpr msg_type_type MSG_NACK = 0x03;

/// \brief Message multipart payload
static const constexpr msg_type_type MSG_DATA = 0x04;

/// \brief Protocol finish message
static const constexpr msg_type_type MSG_DONE = 0x05;

/// \brief Ping message
static const constexpr msg_type_type MSG_PING = 0x07;

/// \brief Multipart corruption message
static const constexpr msg_type_type MSG_MNACK = 0x08;

class ProtocolPrivate;

/*!
 * \brief Protocol implementation
 *
 * All function of this class are not thread safe
 */
class Protocol
{
    ProtocolPrivate* p;
public:

    /*!
     * \brief Recv error types
     */
    enum PollError
    {
        EmptyError,    /*!< Failed to start reading */
        ReadError,     /*!< Failed to read/recv */
        RetryError,    /*!< Retries exhausted */
        WriteError,    /*!< Failed to write reply */
        MsgError,      /*!< Message corrupted */
        MutipartError  /*!< Multipart message corrupted (missing parts) */
    };

    /// \brief Function type to be called on client side when message is received
    using ClientPollAction = std::function<bool(const std::string&, msg_type_type)>;

    /// \brief Function type to be called on client side error occurred
    using ClientErrorCallback = std::function<bool(PollError,const std::string&)>;

    /*!
     * \brief Constructor of full initialisation
     * \param settings Serial port settings
     * \param retry_count Retry count for failed recv
     * \param server If true object will do server (PUB) side job, client (SUB) otherwise
     */
    Protocol(const SerialSettings& settings, int retry_count = 3, bool server = true);
    ~Protocol() noexcept(false);

    /*!
     * \brief Client side start function
     * \param action Will be called when message is reseived
     * \param error_cd Will be called error occured
     * \warning Will work only if \b server = false
     *
     * Non blocking. Destructor will block until #stop() is called or until recv is aborted.
     * If \p action or \b error_cd will return false - recv will abort
     */
    void poll(ClientPollAction action, ClientErrorCallback error_cb);

    /*!
     * \brief Returns \b true if polling is active
     *
     * If startted in server mode - result is undefined
     */
    bool polling();

    /*!
     * \brief Initialise server side
     * \warning Will work only if \b server = true
     *
     * Sends MSG_READY. Cannot be called again before #stop is called
     */
    void start();

    /*!
     * \brief Sends regular message
     * \param message Message data
     * \param is_error If true function will send MSG_ERROR, MSG_DATA otherwise
     * \warning Will work only if \b server = true
     */
    void sendPayload(const std::string& message);

    /*!
     * \brief Stops server and client
     */
    void stop();

    /*!
     * \brief Blocks until all pending messages are send or until protocol is aborted
     */
    void whaitAllSend();

    /// \brief If \p false protocol failed
    bool status();
};

}

#endif // PROTOCOL_H
