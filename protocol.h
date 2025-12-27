/*
 * Copyright (c) 2025 Nazarov Vsevolod
 * This code is licensed under the MIT License. See LICENSE.md for details.
 */

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
    speed_t speed{B38400};

    /// \brief Message send timeout, msec
    int send_timeout_msec{50};

    /// \brief Message recv timeout, decsec (1/10 of a second)
    int recv_timeout_desisec{10};
};

using msg_type_type = uint8_t;
static const constexpr msg_type_type MSG_READY = 0x01;
static const constexpr msg_type_type MSG_ACK = 0x02;
static const constexpr msg_type_type MSG_NACK = 0x03;
static const constexpr msg_type_type MSG_DATA = 0x04;
static const constexpr msg_type_type MSG_DONE = 0x05;
static const constexpr msg_type_type MSG_ERROR = 0x06;

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

    /// \brief Function type to be called on client side when message is received
    using ClientPollAction = std::function<bool(const std::string&, msg_type_type)>;

    /*!
     * \brief Constructor of full initialisation
     * \param settings Serial port settings
     * \param retry_count Retry count for failed recv
     * \param server If true object will do server (PUB) side job, client (SUB) otherwise
     */
    Protocol(const SerialSettings& settings, int retry_count = 3, bool server = true);

    /// Can thorow runtime error on failure to close serial device
    ~Protocol() noexcept(false);

    /*!
     * \brief Client side start function
     * \param action Will be called when message is reseived
     * \warning Will work only if \b server = false
     *
     * Non blocking. Destructor will block until #stop() is called
     */
    void poll(ClientPollAction action);

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
    void sendPayload(const std::string& message, bool is_error = false);

    /*!
     * \brief Stops server and client
     */
    void stop();

    /// \brief If true protocol failed
    bool status();

    /// \brief Last protocol failure message. Empty if protocol did not fail
    std::string lastError();
};

}

#endif // PROTOCOL_H
