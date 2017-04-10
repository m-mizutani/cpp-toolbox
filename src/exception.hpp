#ifndef __CPPTB_EXCEPTION_HPP
#define __CPPTB_EXCEPTION_HPP

#include <exception>
#include <sstream>
#include <string>

namespace tb {
namespace Exception {

// Exception::Error is base exception of cpptb.
// Exception classes of netdec should inherit Exception::Error
// if there is no special reason.

class Error : public std::exception {
 private:
  std::string errmsg_;
 public:
  explicit Error(const std::string &errmsg) : errmsg_(errmsg) {}
  virtual ~Error() throw() {}
  virtual const char* what() const throw() {
    return this->errmsg_.c_str();
  }
};

// ConfigError for invalid preparation.

class NoDataError : public Error {
 public:
  explicit NoDataError(const std::string &errmsg) : Error(errmsg) {}
};


}   // namespace Exception
}   // namespace tb

#endif    // __CPPTB_EXCEPTION_HPP
