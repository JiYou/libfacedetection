#ifndef _PROTOCAL_H
#define _PROTOCAL_H

#include "util.h"
#include "serial.h"
#include "irq.h"
#include <string>
#include <list>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <string.h>
#include <vector>
#include <unordered_map>

namespace RM {
// begin of namespace RM

// 通信息协议抽象类
typedef int (*protocal_cb_t)(void *data);


class Protocal {
  public:
    class Request {
      public:
        virtual std::string GetBody() const = 0;
        virtual void SetBody(const std::string &body) = 0;
        virtual ~Request() {}
    };
    virtual Request *getRequest() = 0;
    class Response {
      public:
        virtual std::string GetBody() const = 0;
        virtual void SetBody(const std::string &body) = 0;
        virtual ~Response() {}
    };
    virtual Response *getResponse() = 0;
  public:
    // 同步发送协议
    virtual void SendMessage(
        const Protocal::Request *m,
        Protocal::Response *ret) = 0;

    // 异步发送协议
    virtual std::future<void> AsyncSendMessage(
        const Protocal::Request *m,
        Protocal::Response *ret) = 0;

    virtual void BatchSendMessage(
        std::vector<Protocal::Request*> &input,
        std::vector<Protocal::Response*> &output
    ) = 0;

    virtual std::future<void> AsyncBatchSendMessage(
        std::vector<Protocal::Request*> &input,
        std::vector<Protocal::Response*> &output
    ) = 0;
  public:
    virtual ~Protocal() {}
};

// 串口通信协议
// 串口接收方在接收到底层的中断的时候，直接去调用相应的中断处理函数。
class SerialProtocal : public Protocal {
    WITH_NO_COPY_CLASS(SerialProtocal);
  public:
    virtual void SendMessage(
        const Protocal::Request *m,
        Protocal::Response *ret) override;

    virtual std::future<void> AsyncSendMessage(
        const Protocal::Request *m,
        Protocal::Response *ret) override;

    virtual void BatchSendMessage(
        std::vector<Protocal::Request*> &input,
        std::vector<Protocal::Response*> &output
    ) override;

    virtual std::future<void> AsyncBatchSendMessage(
        std::vector<Protocal::Request*> &input,
        std::vector<Protocal::Response*> &output
    ) override;
  private:
    SerialPortController *serial_port_;
  public:
    explicit SerialProtocal(const char *comName) {
        serial_port_ = SerialPortController::GetInstance(comName);
    }
    virtual ~SerialProtocal() {
        /*DO NOT Call delete/free serial_port_*/
    }
  public:
    // 请求消息
    class SerialRequest : public Protocal::Request {
      public:
        std::string body_;
        virtual std::string GetBody() const override {
            return body_;
        }
        virtual void SetBody(const std::string &body) override {
            body_ = body;
        }
    };
    virtual Protocal::Request *getRequest() override;
    // 返回消息
    class SerialResponse : public Protocal::Response {
      private:
        mutable std::mutex lock_;
      public:
        std::string body_;

        virtual std::string GetBody() const override {
            std::unique_lock<std::mutex> l(lock_);
            return body_;
        }
        virtual void SetBody(const std::string &body) override {
            std::unique_lock<std::mutex> l(lock_);
            body_ = body;
        }
    };
    virtual Protocal::Response *getResponse() override;
};

} // end of namespace RM
#endif
