// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/DataLogWriter.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace wpi::log;

DataLogWriter::DataLogWriter(std::string_view filename,
                             std::string_view extraHeader)
    : DataLogWriter{s_defaultMessageLog, filename, extraHeader} {}

DataLogWriter::DataLogWriter(wpi::Logger& msglog, std::string_view filename,
                             std::string_view extraHeader)
    : DataLogWriter{msglog,
                    std::make_unique<std::ofstream>(std::string{filename}),
                    extraHeader} {
  if (!m_os->is_open()) {
    Stop();
  }
}

DataLogWriter::DataLogWriter(std::unique_ptr<std::ofstream> os,
                             std::string_view extraHeader)
    : DataLogWriter{s_defaultMessageLog, std::move(os), extraHeader} {}

DataLogWriter::DataLogWriter(wpi::Logger& msglog,
                             std::unique_ptr<std::ofstream> os,
                             std::string_view extraHeader)
    : DataLog{msglog, extraHeader}, m_os{std::move(os)} {
  StartFile();
}

DataLogWriter::~DataLogWriter() {
  if (m_os) {
    Flush();
  }
}

void DataLogWriter::Flush() {
  if (!m_os) {
    return;
  }
  std::vector<Buffer> writeBufs;
  FlushBufs(&writeBufs);
  for (auto&& buf : writeBufs) {
    (*m_os) << buf.GetData();
  }
  ReleaseBufs(&writeBufs);
}

void DataLogWriter::Stop() {
  DataLog::Stop();
  Flush();
  m_os.reset();
}

bool DataLogWriter::BufferFull() {
  Flush();
  return false;
}

extern "C" {

struct WPI_DataLog* WPI_DataLog_CreateWriter(
    const struct WPI_String* filename, const struct WPI_String* extraHeader) {
  auto rv = reinterpret_cast<WPI_DataLog*>(new DataLogWriter{
      wpi::to_string_view(filename), wpi::to_string_view(extraHeader)});
  return rv;
}

}  // extern "C"
