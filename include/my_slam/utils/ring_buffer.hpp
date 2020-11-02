// copyright

#ifndef RING_BUFFER_HPP_
#define RING_BUFFER_HPP_

#include <vector>
// continuous memory, grows via doubling in size and copying
#include <mutex>
#include <exception>
#include <string>

namespace utils {

class RingBufferException : public std::exception {
 private:
  std::string ex_;

 public:
  explicit RingBufferException(std::string ex) : ex_(ex) {}

  virtual const char* what() const throw() {
    return ex_.c_str();
  }
};

template <typename T>
class RingBuffer {
 protected:
  volatile int back_ptr_;
  // volatile: tells compiler to not optimize out things
  // cannot assume only written code touches this variable
  volatile int front_ptr_;

  std::vector<T> data_;

  // for now 1-producer / 1-consumer so no lock_guards
  // unique_ptrs on mutexes / or mutexes are used
  // std::mutex front_mutex_;
  // std::mutex back_mutex_;

  int mem_size;
  // ints for sizes, no mod weirdness

 public:
  RingBuffer() :
    back_ptr_(0),
    front_ptr_(0),
    mem_size(1) {}

  // trampoline constructor
  // essentially bounces to the default constructor
  explicit RingBuffer(int size) : RingBuffer(static_cast<size_t>(size)) {
    if (size < 0) {
      throw RingBufferException("invalid <0 size");
    }
  }

  // templated, explicit constructor must be
  // defined in hpp otherwise
  // downstream include hpp call in child class
  // finds undefined parent constructor call
  //
  // compiler needs to know the complete type
  // at the point of instantiation
  explicit RingBuffer(size_t size) :
    back_ptr_(0),
    front_ptr_(0),
    mem_size(size + 1) {
    if (size >= INT32_MAX) {
      throw RingBufferException("invalid >= INT32_MAX size");
    }
    // at least mem_size == 1 guarantee

    data_.resize(mem_size);
    // implicit default ctor on all elements in data_

    for (unsigned int i = 0; i < data_.size(); ++i) {
      do_preallocate(&get_next_slot());
      advance_front();
      // at the end of iterations, front_ptr_ will advance
      // back to 0
    }
  }

  virtual ~RingBuffer() {}

  T& get_next_slot() {
    // const std::lock_guard<std::mutex> lock(front_mutex_);
    return data_[front_ptr_];
  }

  void advance_front() {
    // const std::lock_guard<std::mutex> lock(front_mutex_);
    front_ptr_ = (front_ptr_ + 1) % mem_size;
  }

  void commit_next_slot() {
    advance_front();
  }

  T& back() {
    // const std::lock_guard<std::mutex> lock(back_mutex_);
    return data_[back_ptr_];
  }

  void advance_back() {
    // const std::lock_guard<std::mutex> lock(back_mutex_);
    if (back_ptr_ == front_ptr_) {
      throw RingBufferException("size would go negative");
    }
    back_ptr_ = (back_ptr_ + 1) % mem_size;
  }

  void pop_back() {
    advance_back();
  }

  size_t max_size() {
    return mem_size - 1;
  }

  size_t size() {
    return (front_ptr_ - back_ptr_ + mem_size) % mem_size;
  }

  bool full() {
    return size() == max_size();
  }

  bool empty() {
    return static_cast<int>(size()) == 0;
  }

  T& operator[](size_t index) {
    if (index > INT32_MAX || index < 0 || index > max_size()) {
      throw RingBufferException("invalid index");
    }

    return data_[(front_ptr_ + static_cast<int>(index)) % mem_size];
  }

  // not pure, subclasses do not NEED to implement this
  // will use default ctor if not implemented which is fine
  // ommitting argument 'data', no 'used parameter' warnings
  virtual void do_preallocate(T* /* data */) {}
};

}  // namespace utils

#endif  // RING_BUFFER_HPP_