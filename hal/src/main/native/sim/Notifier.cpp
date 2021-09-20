// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Notifier.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include <wpi/StringExtras.h>
#include <wpi/condition_variable.h>
#include <wpi/mutex.h>

#include "HALInitializer.h"
#include "NotifierInternal.h"
#include "hal/Errors.h"
#include "hal/HALBase.h"
#include "hal/cpp/fpga_clock.h"
#include "hal/handles/UnlimitedHandleResource.h"
#include "hal/simulation/NotifierData.h"

namespace {
struct Notifier {
  wpi::mutex nameMutex;
  std::string name;
  std::atomic<uint64_t> waitTime{UINT64_MAX};
  // True if in HAL_WaitForNotifierAlarm()
  std::atomic<bool> waitingForAlarm{false};
  wpi::mutex condMutex;
  wpi::condition_variable cond;
};
}  // namespace

using namespace hal;

static wpi::mutex* notifierMutex;
static wpi::condition_variable* notifierCond;
static std::atomic<bool> notifiersPaused{false};

class NotifierHandleContainer
    : public UnlimitedHandleResource<HAL_NotifierHandle, Notifier,
                                     HAL_HandleEnum::Notifier> {
 public:
  ~NotifierHandleContainer() {
    ForEach([](HAL_NotifierHandle handle, Notifier* notifier) {
      notifier->waitTime = UINT64_MAX;
      notifier->cond.notify_all();  // wake up any waiting threads
    });
    std::unique_lock ulock{*notifierMutex};
    notifierCond->notify_all();
  }
};

static NotifierHandleContainer* notifierHandles;

namespace hal {
namespace init {
void InitializeNotifier() {
  static wpi::mutex nMutex;
  notifierMutex = &nMutex;

  static wpi::condition_variable nCond;
  notifierCond = &nCond;

  static NotifierHandleContainer nHandles;
  notifierHandles = &nHandles;
}
}  // namespace init

void PauseNotifiers() {
  notifiersPaused = true;
}

void ResumeNotifiers() {
  notifiersPaused = false;
  WakeupNotifiers();
}

void WakeupNotifiers() {
  notifierHandles->ForEach([](HAL_NotifierHandle handle, Notifier* notifier) {
    notifier->cond.notify_all();
  });
}

void WaitNotifiers() {
  std::unique_lock ulock{*notifierMutex};
  // Wait for all Notifiers to block in HAL_WaitForNotifierAlarm()
  while (1) {
    int notWaiting = 0;
    notifierHandles->ForEach(
        [&](HAL_NotifierHandle handle, Notifier* notifier) {
          if (notifier->waitTime != UINT64_MAX && !notifier->waitingForAlarm) {
            ++notWaiting;
          }
        });
    if (notWaiting == 0) {
      return;
    }
    notifierCond->wait(ulock);
  }
}

void WakeupWaitNotifiers() {
  int32_t status = 0;
  uint64_t curTime = HAL_GetFPGATime(&status);

  // Wake up Notifiers that have expired timeouts
  notifierHandles->ForEach([&](HAL_NotifierHandle handle, Notifier* notifier) {
    // Only wait for the Notifier if it has a valid timeout that's expired
    uint64_t waitTime = notifier->waitTime;
    if (waitTime != UINT64_MAX && curTime >= waitTime) {
      notifier->waitingForAlarm = false;
      notifier->cond.notify_all();
    }
  });

  // Wait for all Notifiers to block in HAL_WaitForNotifierAlarm()
  std::unique_lock ulock{*notifierMutex};
  while (1) {
    int notWaiting = 0;
    notifierHandles->ForEach(
        [&](HAL_NotifierHandle handle, Notifier* notifier) {
          if (notifier->waitTime != UINT64_MAX && !notifier->waitingForAlarm) {
            ++notWaiting;
          }
        });
    if (notWaiting == 0) {
      return;
    }
    notifierCond->wait(ulock);
  }
}
}  // namespace hal

extern "C" {

HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status) {
  hal::init::CheckInit();
  std::shared_ptr<Notifier> notifier = std::make_shared<Notifier>();
  HAL_NotifierHandle handle = notifierHandles->Allocate(notifier);
  if (handle == HAL_kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return HAL_kInvalidHandle;
  }
  return handle;
}

HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime, int32_t priority,
                                       int32_t* status) {
  return true;
}

void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle, const char* name,
                         int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) {
    return;
  }
  std::scoped_lock lock{notifier->nameMutex};
  notifier->name = name;
}

void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) {
    return;
  }

  notifier->waitTime = UINT64_MAX;
  notifier->cond.notify_all();
  std::unique_lock ulock{*notifierMutex};
  notifierCond->notify_all();
}

void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
  auto notifier = notifierHandles->Free(notifierHandle);
  if (!notifier) {
    return;
  }

  // Just in case HAL_StopNotifier() wasn't called...
  notifier->waitTime = UINT64_MAX;
  notifier->cond.notify_all();
  std::unique_lock ulock{*notifierMutex};
  notifierCond->notify_all();
}

void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime, int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) {
    return;
  }

  notifier->waitTime = triggerTime;

  // We wake up any waiters to change how long they're sleeping for
  notifier->cond.notify_all();
  std::unique_lock ulock{*notifierMutex};
  notifierCond->notify_all();
}

void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) {
    return;
  }

  notifier->waitTime = UINT64_MAX;

  // We wake up any waiters since the alarm was cancelled
  notifier->cond.notify_all();
  std::unique_lock ulock{*notifierMutex};
  notifierCond->notify_all();
}

uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) {
    return 0;
  }

  uint64_t waitTime = notifier->waitTime;
  while (waitTime != UINT64_MAX) {
    uint64_t curTime = HAL_GetFPGATime(status);
    if (waitTime != UINT64_MAX && curTime >= waitTime) {
      notifier->waitTime = UINT64_MAX;
      notifier->waitingForAlarm = false;
      return curTime;
    }

    double waitDuration;
    if (waitTime == UINT64_MAX || notifiersPaused) {
      // If not running, wait 1000 seconds
      waitDuration = 1000.0;
    } else {
      waitDuration = (waitTime - curTime) * 1e-6;
    }

    notifier->waitingForAlarm = true;
    {
      std::unique_lock ulock{*notifierMutex};
      notifierCond->notify_all();
    }
    {
      std::unique_lock lock{notifier->condMutex};
      notifier->cond.wait_for(lock,
                              std::chrono::duration<double>(waitDuration));
    }
    waitTime = notifier->waitTime;
  }
  notifier->waitingForAlarm = false;
  return 0;
}

uint64_t HALSIM_GetNextNotifierTimeout(void) {
  uint64_t timeout = UINT64_MAX;
  notifierHandles->ForEach([&](HAL_NotifierHandle, Notifier* notifier) {
    uint64_t waitTime = notifier->waitTime;
    if (waitTime != UINT64_MAX && timeout > waitTime) {
      timeout = waitTime;
    }
  });
  return timeout;
}

int32_t HALSIM_GetNumNotifiers(void) {
  int32_t count = 0;
  notifierHandles->ForEach([&](HAL_NotifierHandle, Notifier* notifier) {
    if (notifier->waitTime != UINT64_MAX) {
      ++count;
    }
  });
  return count;
}

int32_t HALSIM_GetNotifierInfo(struct HALSIM_NotifierInfo* arr, int32_t size) {
  int32_t num = 0;
  notifierHandles->ForEach([&](HAL_NotifierHandle handle, Notifier* notifier) {
    if (notifier->waitTime == UINT64_MAX) {
      return;
    }
    if (num < size) {
      arr[num].handle = handle;
      if (notifier->name.empty()) {
        wpi::format_to_n_c_str(arr[num].name, sizeof(arr[num].name),
                               "Notifier{}",
                               static_cast<int>(getHandleIndex(handle)));
      } else {
        std::strncpy(arr[num].name, notifier->name.c_str(),
                     sizeof(arr[num].name) - 1);
        arr[num].name[sizeof(arr[num].name) - 1] = '\0';
      }
      arr[num].timeout = notifier->waitTime;
    }
    ++num;
  });
  return num;
}

}  // extern "C"
