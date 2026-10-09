// Copyright 2026 Spyrosoft Limited.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @file      service_mock_fork_extensions_test.cpp
// @date      2026-10-10
//
// @brief     Covers the mock API extensions carried in the cocorobotics fork: the *_mocked
//            client methods, wait_for_service() falling back to service_is_ready(), and
//            rclcpp::Service::send_response() forwarding to ServiceMock.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rtest/static_registry.hpp>

#include <chrono>
#include <future>
#include <memory>

namespace
{
using SetBool = std_srvs::srv::SetBool;
using Types = rclcpp::ClientTypes<SetBool>;
using ::testing::Return;

class ForkExtensionsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.start_parameter_event_publisher(false);
    options.start_parameter_services(false);
    options.enable_rosout(false);
    node_ = std::make_shared<rclcpp::Node>("fork_extensions_node", options);
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(ForkExtensionsTest, PublishIsForwardedToPublishMsg)
{
  auto publisher = node_->create_publisher<std_msgs::msg::String>("topic", 10);
  auto mock = rtest::findPublisher<std_msgs::msg::String>(node_, "/topic");
  ASSERT_TRUE(mock);

  std_msgs::msg::String msg;
  msg.data = "hello";
  EXPECT_CALL(*mock, publish_msg(msg)).Times(1);
  publisher->publish(msg);
}

TEST_F(ForkExtensionsTest, AsyncSendRequestReturnsResultOfMockedCall)
{
  auto client = node_->create_client<SetBool>("service");
  auto mock = rtest::findServiceClient<SetBool>(node_, "service");
  ASSERT_TRUE(mock);

  auto response = std::make_shared<SetBool::Response>();
  response->success = true;
  EXPECT_CALL(*mock, async_send_request_mocked(::testing::_)).WillOnce([response](auto) {
    std::promise<Types::SharedResponse> promise;
    promise.set_value(response);
    return Types::FutureResponseAndId(promise.get_future(), 7);
  });

  auto future = client->async_send_request(std::make_shared<SetBool::Request>());
  EXPECT_EQ(future.request_id, 7);
  EXPECT_EQ(future.get(), response);
}

TEST_F(ForkExtensionsTest, WaitForServiceFallsBackToServiceIsReady)
{
  auto client = node_->create_client<SetBool>("service");
  auto mock = rtest::findServiceClient<SetBool>(node_, "service");
  ASSERT_TRUE(mock);

  EXPECT_CALL(*mock, service_is_ready_mocked()).WillOnce(Return(true)).WillOnce(Return(false));
  EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  EXPECT_FALSE(client->wait_for_service());
}

TEST_F(ForkExtensionsTest, WaitForServiceExpectationOverridesFallback)
{
  auto client = node_->create_client<SetBool>("service");
  auto mock = rtest::findServiceClient<SetBool>(node_, "service");
  ASSERT_TRUE(mock);

  EXPECT_CALL(*mock, service_is_ready_mocked()).Times(0);
  EXPECT_CALL(*mock, wait_for_service(std::chrono::milliseconds(1500))).WillOnce(Return(true));
  EXPECT_TRUE(client->wait_for_service(std::chrono::nanoseconds(1'500'000'000)));
}

TEST_F(ForkExtensionsTest, ServiceSendResponseIsForwardedToMock)
{
  auto service = node_->create_service<SetBool>(
    "service", [](const std::shared_ptr<SetBool::Request>, std::shared_ptr<SetBool::Response>) {});
  auto mock = rtest::findService<SetBool>(node_, "service");
  ASSERT_TRUE(mock);

  rmw_request_id_t header{};
  header.sequence_number = 5;
  SetBool::Response response;
  response.success = true;
  EXPECT_CALL(*mock, send_response(header, response)).Times(1);
  service->send_response(header, response);
}

TEST_F(ForkExtensionsTest, MockHandleRequestReturnsResponseWithoutSending)
{
  auto service = node_->create_service<SetBool>(
    "service",
    [](
      const std::shared_ptr<SetBool::Request> request,
      std::shared_ptr<SetBool::Response> response) { response->success = request->data; });
  auto mock = rtest::findService<SetBool>(node_, "service");
  ASSERT_TRUE(mock);

  auto request = std::make_shared<SetBool::Request>();
  request->data = true;
  EXPECT_CALL(*mock, send_response(::testing::_, ::testing::_)).Times(0);
  auto response = mock->handle_request(std::make_shared<rmw_request_id_t>(), request);
  ASSERT_TRUE(response);
  EXPECT_TRUE(response->success);
}

}  // namespace
