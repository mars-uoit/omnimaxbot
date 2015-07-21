/*!
 * \file ax2550/ax2550.h
 * \author  William Woodall <wjwwood@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 William Woodall
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides an interface to the ax2550 motor controller from Roboteq
 */

#ifndef AX2550_AX2550_H
#define AX2550_AX2550_H

#include <string>
#include <sstream>
#include <serial/serial.h>
// #define SERIAL_LISTENER_DEBUG 1
#include <serial/utils/serial_listener.h>

// Handy macro for throwing exceptions
#define AX2550_THROW(exceptionClass,message) throw \
  exceptionClass(__FILE__,__LINE__,(message))

namespace ax2550 {

  /*!
   * This defines a function prototype that can receive logging
   * messages of various levels.
   *
   * \see ax2550::AX2550::info, ax2550::AX2550::debug
   */
  typedef boost::function<void(const std::string&)> LoggingCallback;

  /*!
   * This defines a function prototype that can called when a watchdog
   * timeout is received.
   *
   * \see ax2550::AX2550::watch_dog_callback
   */
  typedef boost::function<void()> WatchDogCallback;

  /*!
   * This class provides an interface for driving and querying an ax2550
   */
  class AX2550 {
  public:
    AX2550 (std::string port = "");
    ~AX2550 ();

    void connect (std::string port = "");

    void disconnect ();

    bool isConnected ();

    bool issueCommand (const std::string &command, std::string &fail_why);

    void move (double speed, double direction);

    void queryEncoders (long &encoder1, long &encoder2, bool relative = false);

    WatchDogCallback watch_dog_callback;
    LoggingCallback debug;
    LoggingCallback info;
    LoggingCallback warn;
  private:
    // Serial stuff
    std::string port_;
    serial::Serial * serial_port_;
    serial::utils::SerialListener serial_listener_;
    // Filter stuff
    void setupFilters_ ();
    serial::utils::BufferedFilterPtr encoders_filt_;
    void watchDogCallback_ (const std::string&);
    serial::utils::FilterPtr watch_dog_filt_;
    serial::utils::BufferedFilterPtr ack_nak_filt_;
    // serial::utils::BufferedFilterPtr rc_msg_filt_;
    // State stuff
    void sync_ ();
    bool connected_;
    bool synced_;
    boost::mutex mc_mutex;
  };

  /*!
   * This exception occurs during connection.
   */
  class ConnectionException : public std::exception {
    std::string file_;
    int line_;
    const char* e_what_;
  public:
    ConnectionException (std::string file, int line, const char * description)
    : file_(file), line_(line), e_what_ (description) {}
    virtual ~ConnectionException() throw() {}

    virtual const char* what () const throw () {
      std::stringstream ss;
      ss << "Connection Exception: " << e_what_;
      ss << ", file " << file_ << ", line " << line_ << ".";
      return ss.str ().c_str ();
    }
  };

  /*!
   * This exception occurs during synchronization with the motor controller.
   */
  class SynchronizationException : public std::exception {
    std::string file_;
    int line_;
    const char* e_what_;
  public:
    SynchronizationException (std::string file, int line,
                              const char * description)
    : file_(file), line_(line), e_what_ (description) {}
    virtual ~SynchronizationException() throw() {}

    virtual const char* what () const throw () {
      std::stringstream ss;
      ss << "Synchronization Exception: " << e_what_;
      ss << ", file " << file_ << ", line " << line_ << ".";
      return ss.str ().c_str ();
    }
  };

  /*!
   * This exception occurs during move and encoder query.
   */
  class CommandFailedException : public std::exception {
    std::string file_;
    int line_;
    const char* e_what_;
  public:
    CommandFailedException (std::string file, int line,
                              const char * description)
    : file_(file), line_(line), e_what_ (description) {}
    virtual ~CommandFailedException() throw() {}

    virtual const char* what () const throw () {
      std::stringstream ss;
      ss << "Command Failed Exception: " << e_what_;
      ss << ", file " << file_ << ", line " << line_ << ".";
      return ss.str ().c_str ();
    }
  };

} // namespace ax2550

#endif
