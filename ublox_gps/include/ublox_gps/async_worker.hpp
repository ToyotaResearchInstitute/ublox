//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_ASYNC_WORKER_HPP
#define UBLOX_GPS_ASYNC_WORKER_HPP

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <thread>
#include <vector>

#include <asio/buffer.hpp>
#include <asio/error_code.hpp>
#include <asio/io_service.hpp>
#include <asio/placeholders.hpp>
#include <asio/write.hpp>

#include "worker.hpp"

namespace ublox_gps {

int debug; //!< Used to determine which debug messages to display

/**
 * @brief Handles Asynchronous I/O reading and writing.
 */
template <typename StreamT>
class AsyncWorker final : public Worker {
 public:
  /**
   * @brief Construct an Asynchronous I/O worker.
   * @param stream the stream for th I/O service
   * @param io_service the I/O service
   * @param buffer_size the size of the input and output buffers
   */
  AsyncWorker(std::shared_ptr<StreamT> stream,
              std::shared_ptr<asio::io_service> io_service,
              std::size_t buffer_size = 8192);
  virtual ~AsyncWorker();

  /**
   * @brief Set the callback function which handles input messages.
   * @param callback the read callback which handles received messages
   */
  void setCallback(const Callback& callback) { read_callback_ = callback; }

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  void setRawDataCallback(const Callback& callback) { write_callback_ = callback; }

  /**
   * @brief Send the data bytes via the I/O stream.
   * @param data the buffer of data bytes to send
   * @param size the size of the buffer
   */
  bool send(const unsigned char* data, const unsigned int size);
  /**
   * @brief Wait for incoming messages.
   * @param timeout the maximum time to wait
   */
  void wait(const std::chrono::milliseconds& timeout);

  bool isOpen() const { return stream_->is_open(); }

 protected:
  /**
   * @brief Read the input stream.
   */
  void doRead(const asio::error_code& error, std::size_t bytes);

  /**
   * @brief Process messages read from the input stream.
   * @param error_code an error code for read failures
   * @param the number of bytes received
   */
  void readEnd(const asio::error_code&, std::size_t);

  /**
   * @brief Send all the data in the output buffer.
   */
  void doWrite();

  /**
   * @brief Close the I/O stream.
   */
  void doClose();

  std::shared_ptr<StreamT> stream_; //!< The I/O stream
  std::shared_ptr<asio::io_service> io_service_; //!< The I/O service

  std::mutex read_mutex_; //!< Lock for the input buffer
  std::condition_variable read_condition_;
  std::vector<unsigned char> in_; //!< The input buffer
  std::size_t in_buffer_size_; //!< number of bytes currently in the input
                               //!< buffer

  std::mutex write_mutex_; //!< Lock for the output buffer
  std::condition_variable write_condition_;
  std::vector<unsigned char> out_; //!< The output buffer

  std::unique_ptr<std::thread> background_thread_; //!< thread for the I/O
                                                       //!< service
  Callback read_callback_; //!< Callback function to handle received messages
  Callback write_callback_; //!< Callback function to handle raw data

  bool stopping_; //!< Whether or not the I/O service is closed
};

template <typename StreamT>
AsyncWorker<StreamT>::AsyncWorker(std::shared_ptr<StreamT> stream,
        std::shared_ptr<asio::io_service> io_service,
        std::size_t buffer_size)
    : stopping_(false) {
  stream_ = stream;
  io_service_ = io_service;
  in_.resize(buffer_size);
  in_buffer_size_ = 0;

  out_.reserve(buffer_size);

  //io_service_->post(std::bind(&AsyncWorker<StreamT>::doRead, this));
  background_thread_ = std::make_unique<std::thread>([this]{ io_service_->run(); });
}

template <typename StreamT>
AsyncWorker<StreamT>::~AsyncWorker() {
  io_service_->post(std::bind(&AsyncWorker<StreamT>::doClose, this));
  background_thread_->join();
  //io_service_->reset();
}

template <typename StreamT>
bool AsyncWorker<StreamT>::send(const unsigned char* data,
                                const unsigned int size) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (size == 0) {
    // ROS_ERROR("Ublox AsyncWorker::send: Size of message to send is 0");
    return true;
  }

  if (out_.capacity() - out_.size() < size) {
    // ROS_ERROR("Ublox AsyncWorker::send: Output buffer too full to send message");
    return false;
  }
  out_.insert(out_.end(), data, data + size);

  io_service_->post(std::bind(&AsyncWorker<StreamT>::doWrite, this));
  return true;
}

template <typename StreamT>
void AsyncWorker<StreamT>::doWrite() {
  std::lock_guard<std::mutex> lock(write_mutex_);
  // Do nothing if out buffer is empty
  if (out_.size() == 0) {
    return;
  }
  // Write all the data in the out buffer
  asio::write(*stream_, asio::buffer(out_.data(), out_.size()));

  if (debug >= 2) {
    // Print the data that was sent
    std::ostringstream oss;
    for (std::vector<unsigned char>::iterator it = out_.begin();
         it != out_.end(); ++it) {
      oss << static_cast<unsigned int>(*it) << " ";
    }
    // ROS_DEBUG("U-Blox sent %li bytes: \n%s", out_.size(), oss.str().c_str());
  }
  // Clear the buffer & unlock
  out_.clear();
  write_condition_.notify_all();
}

template <typename StreamT>
void AsyncWorker<StreamT>::doRead(const asio::error_code & error,
                                  std::size_t bytes) {
  std::lock_guard<std::mutex> lock(read_mutex_);
  stream_->async_read_some(
      asio::buffer(in_.data() + in_buffer_size_,
                          in_.size() - in_buffer_size_),
                          std::bind(&AsyncWorker<StreamT>::readEnd, this,
                              asio::placeholders::error,
                              asio::placeholders::bytes_transferred));
}

template <typename StreamT>
void AsyncWorker<StreamT>::readEnd(const asio::error_code& error,
                                   std::size_t bytes_transfered) {
  std::lock_guard<std::mutex> lock(read_mutex_);
  if (error) {
    // ROS_ERROR("U-Blox ASIO input buffer read error: %s, %li",
    //           error.message().c_str(),
    //           bytes_transfered);
  } else if (bytes_transfered > 0) {
    in_buffer_size_ += bytes_transfered;

    unsigned char *pRawDataStart = &(*(in_.begin() + (in_buffer_size_ - bytes_transfered)));
    std::size_t raw_data_stream_size = bytes_transfered;

    if (write_callback_) {
      write_callback_(pRawDataStart, raw_data_stream_size);
    }

    if (debug >= 4) {
      std::ostringstream oss;
      for (std::vector<unsigned char>::iterator it =
               in_.begin() + in_buffer_size_ - bytes_transfered;
           it != in_.begin() + in_buffer_size_; ++it) {
        oss << static_cast<unsigned int>(*it) << " ";
      }
      // ROS_DEBUG("U-Blox received %li bytes \n%s", bytes_transfered,
      //          oss.str().c_str());
    }

    if (read_callback_) {
      read_callback_(in_.data(), in_buffer_size_);
    }

    read_condition_.notify_all();
  }

  if (!stopping_) {
    //io_service_->post(std::bind(&AsyncWorker<StreamT>::doRead, this));
  }
}

template <typename StreamT>
void AsyncWorker<StreamT>::doClose() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  stopping_ = true;
  asio::error_code error;
  stream_->close(error);
  if (error) {
    // ROS_ERROR_STREAM(
    //     "Error while closing the AsyncWorker stream: " << error.message());
  }
}

template <typename StreamT>
void AsyncWorker<StreamT>::wait(
  const std::chrono::milliseconds& timeout) {
  std::unique_lock<std::mutex> lock(read_mutex_);
  read_condition_.wait_for(lock, timeout);
}

}  // namespace ublox_gps

#endif  // UBLOX_GPS_ASYNC_WORKER_HPP
