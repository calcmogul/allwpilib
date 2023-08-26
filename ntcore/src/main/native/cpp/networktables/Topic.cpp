// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "networktables/Topic.h"

#include <glaze/json.hpp>

#include "networktables/GenericEntry.h"

using namespace nt;

glz::json_t Topic::GetProperty(std::string_view name) const {
  return ::nt::GetTopicProperty(m_handle, name);
}

void Topic::SetProperty(std::string_view name, const glz::json_t& value) {
  ::nt::SetTopicProperty(m_handle, name, value);
}

glz::json_t Topic::GetProperties() const {
  return ::nt::GetTopicProperties(m_handle);
}

GenericSubscriber Topic::GenericSubscribe(const PubSubOptions& options) {
  return GenericSubscribe("", options);
}

GenericSubscriber Topic::GenericSubscribe(std::string_view typeString,
                                          const PubSubOptions& options) {
  return GenericSubscriber{::nt::Subscribe(
      m_handle, ::nt::GetTypeFromString(typeString), typeString, options)};
}

GenericPublisher Topic::GenericPublish(std::string_view typeString,
                                       const PubSubOptions& options) {
  return GenericPublisher{::nt::Publish(
      m_handle, ::nt::GetTypeFromString(typeString), typeString, options)};
}

GenericPublisher Topic::GenericPublishEx(std::string_view typeString,
                                         const glz::json_t& properties,
                                         const PubSubOptions& options) {
  return GenericPublisher{::nt::PublishEx(m_handle,
                                          ::nt::GetTypeFromString(typeString),
                                          typeString, properties, options)};
}

GenericEntry Topic::GetGenericEntry(const PubSubOptions& options) {
  return GetGenericEntry("", options);
}

GenericEntry Topic::GetGenericEntry(std::string_view typeString,
                                    const PubSubOptions& options) {
  return GenericEntry{::nt::GetEntry(
      m_handle, ::nt::GetTypeFromString(typeString), typeString, options)};
}
