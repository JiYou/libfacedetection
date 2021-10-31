#ifndef SERIAL_H
#define SERIAL_H

#ifdef _WIN32
#include <Windows.h>
#include <WinBase.h>
#include <process.h>
#endif

#include "nce/util.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <string>
#include <future>
#include <list>
#include <vector>
#include <unordered_map>

namespace RM {
// begin namespace RM

// SerialPortController 串口控制类
class SerialPortController {
    WITH_NO_COPY_CLASS(SerialPortController);
  public:
    const static char* COM1;
    const static char *GET_VERSION;
    const static char *VERSION_RET;
    const static char *GET_POWER;
    const static char *POWER_RET;
    const static char *GET_UNKNOWN;
    const static char *UNKNOWN_RET;
  public:
    friend class StaticMapperUtil<std::string, SerialPortController>;
    static SerialPortController *GetInstance(const char *comName);
    std::string Write(const std::string &str);
    std::future<std::string> AsyncWrite(const std::string &str);
    std::vector<std::string> WriteBatch(const std::vector<std::string> &vs);
    std::future<std::vector<std::string> > AsyncWriteBatch(const std::vector<std::string> &vs);
  public:
    int GetSleepTime() {
        return serial_recv_thd_->GetSleepTime();
    }
    void SetSleepTime(int n) {
        serial_recv_thd_->SetSleepTime(n);
    }
  private:
    friend std::shared_ptr<SerialPortController>;
    std::string write(const std::string &str);
    explicit SerialPortController(const char *comName);
    explicit SerialPortController(const std::string &comName);
    virtual ~SerialPortController();
  private:
    // 锁住整个controller
    // 以免多个线程同时发送消息。
    // 也就是说，这个controller只支持单线程使用。
    std::mutex mutex_controller_;

  private:
    class SerialPort {
        WITH_NO_COPY_CLASS(SerialPort);
        // 这里并不去真正打开串，只是记录写入的数据。然后原样返回
        std::string data_;
        // 记录寄存器的值
        std::unordered_map<int, int> regs_;
        int chosed_addr_;
        std::mutex lock_;
      public:
        bool Open(const char *comName);
        void Close();
        size_t Read(char *buf, size_t len);
        size_t Write(const char *data, size_t len);
        void Flush();
      public:
        SerialPort() {}
        ~SerialPort() {}
    };

  private:
    // 读串口的线程
    class SerialReceiveThread {
        WITH_NO_COPY_CLASS(SerialReceiveThread);
      private:
        void entry(SerialPort *p);
      public:
        void Run(void);
        void Stop(void);
        std::string GetMessage(void);
        std::future<std::string> AsyncGetMessage(void);
      public:
        int GetSleepTime() {
            std::unique_lock<std::mutex> l(sleep_time_lock_);
            return sleep_time_ms_;
        }
        void SetSleepTime(int n) {
            std::unique_lock<std::mutex> l(sleep_time_lock_);
            sleep_time_ms_ = n;
        }
      private:
        std::thread *thd_;
        SerialPort *serial_port_;
        std::list<std::string> message_queue_;
        std::mutex qlock_;
        std::condition_variable cond_;
        std::atomic<bool> stop_;

        // 可以根据需要设置串口接收线程刷新的速度
        // 比如某些时候需要快速响应中断的时候，最好是全力刷新
        std::mutex sleep_time_lock_;
        // 初始值，每10ms去读串口
        int sleep_time_ms_ = 10;
      public:
        explicit SerialReceiveThread(SerialPort *p);
        ~SerialReceiveThread();
    };

    SerialPort *serial_port_;
    SerialReceiveThread *serial_recv_thd_;
};


} // end namespace RM.

#endif // SERIAL_H
