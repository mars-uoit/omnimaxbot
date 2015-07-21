#include "ax2550/ax2550.h"

#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdarg>

#include <boost/algorithm/string.hpp>

using namespace ax2550;

using std::string;
using std::stringstream;
using serial::Serial;
using serial::utils::SerialListener;
using serial::utils::BufferedFilterPtr;
using serial::utils::TokenPtr;

inline void defaultInfo(const string &msg) {
  std::cout << "AX2550 Info: " << msg << std::endl;
}

// Tokenizes on carriage return or W
inline void tokenizer(const std::string &data,
                      std::vector<TokenPtr> &tokens)
{
  // Find the number of W's
  size_t number_of_Ws =
    (size_t) std::count(data.begin(), data.end(), 'W');
  // Create tokens for each of the acks
  for(size_t i = 0; i < number_of_Ws; ++i) {
    tokens.push_back(TokenPtr( new std::string("W") ));
  }
  // Split on W
  typedef std::vector<std::string> find_vector_type;
  find_vector_type t;
  boost::split(t, data, boost::is_any_of("W"));
  // Rejoin without the W's
  string new_data = "";
  for (find_vector_type::iterator it = t.begin(); it != t.end(); ++it) {
    new_data.append((*it));
  }
  // Split on \r or \n
  find_vector_type t2;
  boost::split(t2, new_data, boost::is_any_of("\r\n"));
  for (find_vector_type::iterator it = t2.begin(); it != t2.end(); ++it) {
    tokens.push_back(TokenPtr( new std::string(*it) ));
  }
}

AX2550::AX2550 (string port)
: port_(""), serial_port_(NULL), serial_listener_(1),
  connected_(false), synced_(false)
{
  this->port_ = port;
  // Set default callbacks
  this->info = defaultInfo;
  this->watch_dog_callback = NULL;
  // Set the custom tokenizer
  this->serial_listener_.setTokenizer(tokenizer);
  // If the port is set, connect
  if (!this->port_.empty()) {
    this->connect();
  }
}

AX2550::~AX2550 () {
  this->disconnect();
}

void
AX2550::connect (string port) {
  // Make sure we aren't already connected
  if (this->connected_) {
    AX2550_THROW(ConnectionException, "already connected");
  }
  // If a port was passed in, set the interal one
  if (!port.empty()) {
    this->port_ = port;
  }
  // Check to see if the port is set to something
  if (this->port_.empty()) {
    AX2550_THROW(ConnectionException, "serial port name is empty");
  }
  // Call disconnect to ensure we are in a clean state
  this->disconnect();
  // Setup the filters
  this->setupFilters_();
  // Setup the serial port
  this->serial_port_ = new Serial();
  this->serial_port_->setPort(this->port_);
  this->serial_port_->setBaudrate(9600);
  this->serial_port_->setParity(serial::parity_even);
  this->serial_port_->setStopbits(serial::stopbits_one);
  this->serial_port_->setBytesize(serial::sevenbits);
  serial::Timeout to = serial::Timeout::simpleTimeout(10);
  this->serial_port_->setTimeout(to);
  // Open the serial port
  this->serial_port_->open();
  // Setup the serial listener
  this->serial_listener_.setChunkSize(2);
  this->serial_listener_.startListening((*this->serial_port_));
  // Synchronize with the motor controller
  this->sync_();
  this->connected_ = true;
}

void
AX2550::disconnect () {
  this->connected_ = false;
  if (this->serial_listener_.isListening()) {
    this->serial_listener_.stopListening();
  }
  if (this->serial_port_ != NULL) {
    delete this->serial_port_;
    this->serial_port_ = NULL;
  }
}

bool
AX2550::isConnected () {
  return this->connected_;
}

bool
AX2550::issueCommand (const string &command, string &fail_why) {
  // Setup an echo filter
  BufferedFilterPtr echo_filt = this->serial_listener_.createBufferedFilter(
    SerialListener::exactly(command));
  this->serial_port_->write(command+"\r");
  // Listen for the echo
  if (echo_filt->wait(50).empty()) {
    fail_why = "failed to receive an echo";
    return false;
  }
  return true;
}

inline string
string_format(const string &fmt, ...) {
  int size = 100;
  string str;
  va_list ap;
  while (1) {
    str.resize(size);
    va_start(ap, fmt);
    int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
    va_end(ap);
    if (n > -1 && n < size) {
      str.resize(n);
      return str;
    }
    if (n > -1) {
      size = n + 1;
    } else {
      size *= 2;
    }
  }
  return str;
}

void
AX2550::move (double right_speed, double left_speed) 
{
  if(!this->connected_) 
  {
    AX2550_THROW(CommandFailedException, "must be connected to move");
  }
  // Grab the lock
  boost::mutex::scoped_lock lock(this->mc_mutex);
  string serial_buffer;
  unsigned char right_speed_hex, left_speed_hex;
  string fail_why;
  // Create the speed command for the right motor
  right_speed_hex = (unsigned char) (fabs(right_speed));
  if(right_speed < 0) 
  {
    serial_buffer = string_format("!a%.2X", right_speed_hex);
  } 
  else 
  {
    serial_buffer = string_format("!A%.2X", right_speed_hex);
  }
  // Issue the speed command
  if (!this->issueCommand(serial_buffer, fail_why)) 
  {
    AX2550_THROW(CommandFailedException, fail_why.c_str());
  }
  // Listen for an ack or nak
  this->ack_nak_filt_->clear();
  string result = this->ack_nak_filt_->wait(100);
  if (result != "+") 
  {
    if (result == "-") 
    {
      AX2550_THROW(CommandFailedException, "nak received, command failed");
    }
    AX2550_THROW(CommandFailedException, "did not receive an ack or nak");
  }
  // Create the speed command for the left motor
  left_speed_hex = (unsigned char) (fabs(left_speed));
  if(left_speed < 0) 
  {
    serial_buffer = string_format("!b%.2X", left_speed_hex);
  } 
  else 
  {
    serial_buffer = string_format("!B%.2X", left_speed_hex);
  }
  // Issue the speed command
  if (!this->issueCommand(string(serial_buffer), fail_why)) 
  {
    AX2550_THROW(CommandFailedException, fail_why.c_str());
  }
  // Listen for an ack or nak
  this->ack_nak_filt_->clear();
  result = this->ack_nak_filt_->wait(100);
  if (result != "+") 
  {
    if (result == "-") 
    {
      AX2550_THROW(CommandFailedException, "nak received, command failed");
    }
    AX2550_THROW(CommandFailedException, "did not receive an ack or nak");
  }
}

void
AX2550::queryEncoders (long &encoder1, long &encoder2, bool relative) {
  if(!this->connected_) 
  {
    AX2550_THROW(CommandFailedException, "must be connected to query the encoders");
  }
  // Check the count in the encoder filter, should be 0
  if (this->encoders_filt_->count()) 
  {
    stringstream ss;
    ss << "There were " << this->encoders_filt_->count()
       << " orphaned encoder messages in the filter...";
    this->warn(ss.str());
  }
  // Clear the encoder queue
  this->encoders_filt_->clear();
  // Grab the lock
  boost::mutex::scoped_lock lock(this->mc_mutex);
  // Query the first encoder
  string cmd1, cmd2, fail_why;
  if (relative) {
    cmd1 = "?q4";
  } else {
    cmd1 = "?q0";
  }
  this->serial_port_->write(cmd1+"\r");
  // Query the second encoder
  if (relative) {
    cmd2 = "?q5";
  } else {
    cmd2 = "?q1";
  }
  this->serial_port_->write(cmd2+"\r");
  // Listen for Query 1
  string response = this->encoders_filt_->wait(100);
  if (response.empty()) {
    string msg = string("failed to receive a response from ")+cmd1;
    AX2550_THROW(CommandFailedException, msg.c_str());
  }
  // Parse the response
  char fillbyte;
  // Determine sign
  if(response.substr(0,1).find_first_of("01234567") != std::string::npos) {
    // Then positive
    fillbyte = '0';
  } else {
    // Then negative
    fillbyte = 'F';
  }
  // Add filler bytes
  size_t difference = 8 - response.length();
  string filler(difference, fillbyte);
  response.insert(0, filler);
  // Convert to integer
  signed int encoder = 0;
  sscanf(response.c_str(), "%X", &encoder);
  encoder1 = encoder;
  // reset stuff
  fillbyte = '0';
  difference = 0;
  filler = "";
  encoder = 0;
  // Listen for Query 2
  response = this->encoders_filt_->wait(100);
  if (response.empty()) {
    string msg = "failed to receive a response from "+cmd2;
    AX2550_THROW(CommandFailedException, msg.c_str());
  }
  // Parse the response
  // Determine sign
  if(response.substr(0,1).find_first_of("01234567") != std::string::npos) {
    // Then positive
    fillbyte = '0';
  } else {
    // Then negative
    fillbyte = 'F';
  }
  // Add filler bytes
  difference = 8 - response.length();
  filler = string(difference, fillbyte);
  response.insert(0, filler);
  // Convert to integer
  encoder = 0;
  sscanf(response.c_str(), "%X", &encoder);
  encoder2 = encoder;
}

void
AX2550::sync_ () {
  if (this->synced_)
    return;
  boost::mutex::scoped_lock lock(this->mc_mutex);
  // Reset the motor controller
  this->serial_port_->write("%rrrrrr\r");
  // Wait for an R/C Message
  {
    BufferedFilterPtr rc_msg_filt =
      this->serial_listener_.createBufferedFilter(
        SerialListener::startsWith(":"));
    rc_msg_filt->clear();
    if (rc_msg_filt->wait(2000).empty()) {
      AX2550_THROW(SynchronizationException,
        "did not receive an R/C message after reset");
    }
  }
  // Write \r to the port until in serial mode
  BufferedFilterPtr ok_filt =
    this->serial_listener_.createBufferedFilter(
      SerialListener::contains("OK"));
  bool got_ok = false;
  for (int i = 0; i < 20; ++i) {
    this->serial_port_->write("\r");
    if (!ok_filt->wait(50).empty()) {
      got_ok = true;
      break;
    }
  }
  // Check to see if we ever got an OK
  if (!got_ok) {
    AX2550_THROW(SynchronizationException, "failed to get into serial mode");
  }
  this->synced_ = true;
  this->info("Synchronized with the ax2550");
}

inline bool
isAnEncoderMsg (const string &token) {
  string test = "0123456789abcdefABCDEF";
#if 0
  std::cout << "isAnEncoderMsg: Is `" << token
            << "` an encoder message?: ";
  if (token.substr(0,1).find_first_of(test) != string::npos) {
    std::cout << "True";
  } else {
    std::cout << "False";
  }
  std::cout << std::endl;
#endif
  // If token[0] is any of 0123456789abcdefABCDEF (hex)
  if (token.substr(0,1).find_first_of(test) != string::npos) {
    return true;
  }
  return false;
}

void
AX2550::watchDogCallback_ (const string &token) {
  if (this->watch_dog_callback != NULL) {
    this->watch_dog_callback();
  }
}

inline bool
isAckOrNak (const string &token) {
#if 0
  std::cout << "isAckOrNak: Is `" << token
            << "` an ack or nak?: ";
  if (token.find_first_of("+-") != string::npos) {
    std::cout << "True";
  } else {
    std::cout << "False";
  }
  std::cout << std::endl;
#endif
  if (token.find_first_of("+-") != string::npos) {
    return true;
  }
  return false;
}

void
AX2550::setupFilters_ () {
  // Setup the encoder filter
  this->encoders_filt_ =
    this->serial_listener_.createBufferedFilter(isAnEncoderMsg);
  // Setup the watchdog filter
  this->watch_dog_filt_ = this->serial_listener_.createFilter(
    SerialListener::exactly("W"),
    boost::bind(&AX2550::watchDogCallback_, this, _1));
  // Setup ack/nak filter
  this->ack_nak_filt_ =
    this->serial_listener_.createBufferedFilter(isAckOrNak);
  // // Setup R/C message filter
  // this->rc_msg_filt_ = this->serial_listener_.createBufferedFilter(
  //   SerialListener::startsWith(":"));
}





