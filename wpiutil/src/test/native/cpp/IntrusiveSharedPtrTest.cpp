// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <stdint.h>

#include <type_traits>
#include <utility>

#include "gtest/gtest.h"
#include "wpi/IntrusiveSharedPtr.h"

namespace {

struct Mock {
  uint32_t refCount = 0;
};

inline void IntrusiveSharedPtrIncRefCount(Mock* obj) {
  ++obj->refCount;
}

inline void IntrusiveSharedPtrDecRefCount(Mock* obj) {
  if (--obj->refCount == 0) {
    delete obj;
  }
}

}  // namespace

TEST(IntrusiveSharedPtrTest, Traits) {
  using Ptr = wpi::IntrusiveSharedPtr<Mock>;

  EXPECT_EQ(sizeof(Ptr), sizeof(wpi::IntrusiveSharedPtr<Mock>*));
  EXPECT_EQ(std::alignment_of_v<Ptr>,
            std::alignment_of_v<wpi::IntrusiveSharedPtr<Mock>*>);

  EXPECT_TRUE(std::is_default_constructible_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_default_constructible_v<Ptr>);
  EXPECT_TRUE(!std::is_trivially_default_constructible_v<Ptr>);

  EXPECT_TRUE(std::is_copy_constructible_v<Ptr>);
  EXPECT_TRUE(!std::is_trivially_copy_constructible_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_copy_constructible_v<Ptr>);

  EXPECT_TRUE(std::is_move_constructible_v<Ptr>);
  EXPECT_TRUE(!std::is_trivially_move_constructible_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_move_constructible_v<Ptr>);

  EXPECT_TRUE(std::is_copy_assignable_v<Ptr>);
  EXPECT_TRUE(!std::is_trivially_copy_assignable_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_copy_assignable_v<Ptr>);

  EXPECT_TRUE(std::is_move_assignable_v<Ptr>);
  EXPECT_TRUE(!std::is_trivially_move_assignable_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_move_assignable_v<Ptr>);

  EXPECT_TRUE(std::is_swappable_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_swappable_v<Ptr>);

  EXPECT_TRUE(std::is_destructible_v<Ptr>);
  EXPECT_TRUE(!std::is_trivially_destructible_v<Ptr>);
  EXPECT_TRUE(std::is_nothrow_destructible_v<Ptr>);

  EXPECT_TRUE((std::is_constructible_v<Ptr, std::nullptr_t>));
  EXPECT_TRUE((std::is_nothrow_constructible_v<Ptr, std::nullptr_t>));
  EXPECT_TRUE((!std::is_trivially_constructible_v<Ptr, std::nullptr_t>));
}

TEST(IntrusiveSharedPtrTest, DefaultConstruction) {
  wpi::IntrusiveSharedPtr<Mock> empty;

  EXPECT_EQ(empty.Get(), nullptr);
  EXPECT_EQ(empty.operator->(), nullptr);
}

TEST(IntrusiveSharedPtrTest, ConstuctedFromNullptr) {
  wpi::IntrusiveSharedPtr<Mock> empty{nullptr};

  EXPECT_EQ(empty.Get(), nullptr);
  EXPECT_EQ(empty.operator->(), nullptr);
}

TEST(IntrusiveSharedPtrTest, CompareToEmptySharedPtr) {
  wpi::IntrusiveSharedPtr<Mock> empty1;
  wpi::IntrusiveSharedPtr<Mock> empty2;

  EXPECT_EQ(empty1, empty2);
  EXPECT_FALSE(empty1 != empty2);
}

TEST(IntrusiveSharedPtrTest, CompareToSharedPtrCreatedFromNullptr) {
  wpi::IntrusiveSharedPtr<Mock> empty1;
  wpi::IntrusiveSharedPtr<Mock> empty2(nullptr);

  EXPECT_EQ(empty1, empty2);
  EXPECT_FALSE(empty1 != empty2);

  EXPECT_EQ(empty2, empty1);
  EXPECT_FALSE(empty2 != empty1);
}

TEST(IntrusiveSharedPtrTest, Counting) {
  {
    auto object = new Mock{};

    // Attach
    wpi::IntrusiveSharedPtr<Mock> ptr1{object};
    EXPECT_EQ(object, ptr1.Get());
    EXPECT_EQ(object->refCount, 1);
    EXPECT_TRUE(static_cast<bool>(ptr1));
    EXPECT_EQ(ptr1.operator->(), object);

    // Ref
    wpi::IntrusiveSharedPtr<Mock> ptr2{object};
    EXPECT_EQ(object, ptr2.Get());
    EXPECT_EQ(object->refCount, 2);
    EXPECT_TRUE(static_cast<bool>(ptr2));
    EXPECT_EQ(ptr2.operator->(), object);
  }

  // Copy and assignment
  {
    auto object = new Mock{};
    wpi::IntrusiveSharedPtr<Mock> ptr1{object};
    EXPECT_EQ(object->refCount, 1);

    wpi::IntrusiveSharedPtr<Mock> ptr2{ptr1};
    EXPECT_EQ(object->refCount, 2);

    wpi::IntrusiveSharedPtr<Mock> ptr3{object};
    EXPECT_EQ(object->refCount, 3);
  }

  // Move
  {
    auto object = new Mock{};
    wpi::IntrusiveSharedPtr<Mock> ptr1{object};
    EXPECT_EQ(object->refCount, 1);

    auto ptr2 = std::move(ptr1);
    EXPECT_EQ(object->refCount, 1);
  }

  // Self-assignment
  {
    auto object = new Mock{};
    wpi::IntrusiveSharedPtr<Mock> ptr{object};

    ptr = ptr;
    EXPECT_EQ(object->refCount, 1);

    ptr = std::move(ptr);
    EXPECT_EQ(ptr.Get(), object);
    EXPECT_EQ(object->refCount, 1);
  }
}
