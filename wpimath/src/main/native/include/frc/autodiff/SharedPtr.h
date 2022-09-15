// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <cstddef>
#include <utility>

#if __GNUC__ == 12 && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuse-after-free"
#endif  // __GNUC__ == 12 && !defined(__clang__)

namespace frc::autodiff {

/**
 * A custom shared_ptr implementation without thread synchronization overhead.
 */
template <typename T>
class SharedPtr {
 public:
  constexpr SharedPtr() noexcept = default;

  constexpr SharedPtr(std::nullptr_t) noexcept {}  // NOLINT

  explicit SharedPtr(T* ptr) {
    if (ptr != nullptr) {
      m_ptr = ptr;
      m_refCount = new uint16_t{1};
    }
  }

  ~SharedPtr() { Release(); }

  SharedPtr(const SharedPtr<T>& rhs) noexcept {  // NOLINT
    m_ptr = rhs.m_ptr;
    m_refCount = rhs.m_refCount;

    Retain();
  }

  SharedPtr<T>& operator=(const SharedPtr<T>& rhs) noexcept {  // NOLINT
    if (m_ptr != rhs.m_ptr) {
      Release();

      m_ptr = rhs.m_ptr;
      m_refCount = rhs.m_refCount;

      Retain();
    }

    return *this;
  }

  SharedPtr(SharedPtr<T>&& rhs) noexcept {  // NOLINT
    std::swap(m_ptr, rhs.m_ptr);
    std::swap(m_refCount, rhs.m_refCount);
  }

  SharedPtr<T>& operator=(SharedPtr<T>&& rhs) noexcept {
    std::swap(m_ptr, rhs.m_ptr);
    std::swap(m_refCount, rhs.m_refCount);

    return *this;
  }

  T& operator*() const noexcept { return *m_ptr; }

  T* operator->() const noexcept { return m_ptr; }

  friend bool operator==(const SharedPtr<T>& lhs,
                         const SharedPtr<T>& rhs) noexcept {
    return lhs.m_ptr == rhs.m_ptr;
  }

  friend bool operator!=(const SharedPtr<T>& lhs,
                         const SharedPtr<T>& rhs) noexcept {
    return lhs.m_ptr != rhs.m_ptr;
  }

  friend bool operator==(const SharedPtr<T>& lhs, std::nullptr_t) noexcept {
    return lhs.m_ptr == nullptr;
  }

  friend bool operator==(std::nullptr_t, const SharedPtr<T>& rhs) noexcept {
    return nullptr == rhs.m_ptr;
  }

  friend bool operator!=(const SharedPtr<T>& lhs, std::nullptr_t) noexcept {
    return lhs.m_ptr != nullptr;
  }

  friend bool operator!=(std::nullptr_t, const SharedPtr<T>& rhs) noexcept {
    return nullptr != rhs.m_ptr;
  }

 private:
  T* m_ptr = nullptr;
  uint16_t* m_refCount = nullptr;

  void Retain() {
    if (m_refCount != nullptr) {
      ++*m_refCount;
    }
  }

  void Release() {
    if (m_refCount != nullptr) {
      --*m_refCount;

      if (*m_refCount == 0) {
        delete m_ptr;
        m_ptr = nullptr;

        delete m_refCount;
        m_refCount = nullptr;
      }
    }
  }
};

template <typename T, typename... Args>
SharedPtr<T> MakeShared(Args&&... args) {
  return SharedPtr<T>{new T(std::forward<Args>(args)...)};
}

}  // namespace frc::autodiff

#if __GNUC__ == 12 && !defined(__clang__)
#pragma GCC diagnostic pop
#endif  // __GNUC__ == 12 && !defined(__clang__)
