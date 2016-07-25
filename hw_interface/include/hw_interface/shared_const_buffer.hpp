#ifndef SHARED_CONST_BUFFER_HPP__
#define SHARED_CONST_BUFFER_HPP__

//
// reference_counted.hpp
// ~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio.hpp>
#include <vector>

// A reference-counted non-modifiable buffer class.

namespace hw_interface_support_types
{
    class shared_const_buffer
    {
    public:
      // Construct from a std::string.
      explicit shared_const_buffer(const std::string& data)
        : data_(new std::vector<char>(data.begin(), data.end())),
          buffer_(boost::asio::buffer(*data_))
      {
      }

      // Implement the ConstBufferSequence requirements.
      typedef boost::asio::const_buffer value_type;
      typedef const boost::asio::const_buffer* const_iterator;
      const boost::asio::const_buffer* begin() const { return &buffer_; }
      const boost::asio::const_buffer* end() const { return &buffer_ + 1; }

    private:
      std::shared_ptr<std::vector<char> > data_;
      boost::asio::const_buffer buffer_;
    };
}

#endif //SHARED_CONST_BUFFER_HPP__
