#ifndef NCE_BOARD_H
#define NCE_BOARD_H

#include "nce/irq.h"
#include "nce/protocal.h"
#include "nce/util.h"

#include <stdio.h>
#include <unordered_map>
#include <memory>
#include <vector>
#include <mutex>
#include <functional>

namespace RM {
// begin of namespace RM

// 控制板上有各种各样的硬件外设。
// 所以这里需要三个抽象:
// a. 控制板
// b. 外设
// c. 外设寄存器
// 外设的种类
// 1. Power   电源控制器
// 2. Motor   马达
// 3. Speaker 蜂鸣器
// 4. LED     灯
// other.

/*
 * 设备寄存器
 * 由于有各种宽度，比如有的寄存器是8位宽，有的是16位宽。
 * 为了通用性，这里使用了模板，利用T来指定数据的宽度。
 * 这里主要是处理电机上的各种寄存器。
 * 这个类并没有要求使用什么具体的通信协议
 *
 * 实际上也是一个抽象类，只不过实现了Register的方法
 * 把buildMessage的部分，留给子类实现。
 */
template<typename T>
class DeviceRegister {
    WITH_NO_COPY_CLASS(DeviceRegister);
  public:
    // 可以使用的公共接口
    // 比如遇到中断的时候，如果对中断信号进行了响应
    // 理想情况一下硬件板子上应该是
    // 返回<return 0x00>
    // 但是返回的是<return 0x16>
    // 这个时候就会出现问题。
    // 这里加个回调函数，在board.h对中断函数的响应这里
    // 一旦发现想要的值并不是0
    // 那就就再次处理一下中断，直接成功为止。
    // 如果硬件可靠，按理说不需要这样的处理!
    // bool check是说，如果发现返回值不是
    // <return 0x00>
    // 那么是否还会继发送清IRQ指令
    // force_clear_irq就是说是否连续发送指令
    virtual bool Write(T content, bool force_clear_irq=false);

    virtual std::future<bool> AsyncWrite(T content);
    virtual T Read();
    virtual std::future<T> AsyncRead();
    virtual T Addr() const {
        return addr_;
    }

  protected:
    // 子类可以调用
    virtual void SetProtocal(Protocal *p) {
        RM_ASSERT(p);
        // 这是为了正确的初始化。
        // 多个入口来设置寄存器的进候，能够保证是正确的。
        if (protocal_) RM_ASSERT(protocal_ == p);
        if (protocal_) return;
        protocal_ = p;
        req_ = protocal_->getRequest();
        resp_ = protocal_->getResponse();
    }
    virtual Protocal* GetProtocal() const {
        return protocal_;
    }
  protected:
    // 必须需要子类实现
    virtual std::vector<std::string> buildWriteMessage(T content) = 0;
    virtual std::vector<std::string> buildReadMessage() = 0;
    virtual T DecodeResponse(Protocal::Response *resp) = 0;

    explicit DeviceRegister(T t) : addr_(t),
        protocal_(NULL), req_(NULL), resp_(NULL) {}
    virtual ~DeviceRegister() {
        // DO NOT free protocal_ here.
        protocal_ = NULL;
        if (req_) delete req_;
        if (resp_) delete resp_;
    }
  private:
    T addr_;
    std::mutex lock_;
    Protocal *protocal_;
    Protocal::Request *req_;
    Protocal::Response *resp_;
};

template<typename T>
bool DeviceRegister<T>::Write(T content, bool force_clear_irq) {
    std::unique_lock<std::mutex> l(lock_);
    // 在写数据之前，应该是设置好了通信协议
    RM_ASSERT(protocal_);
    std::vector<std::string> vs = buildWriteMessage(content);
    for (auto &item : vs) {
        req_->SetBody(item);
        protocal_->SendMessage(req_, resp_);
        // 加force_clear_irq
        // 主要是因为有时候发送了清中断指令不起作用
        // 所以需要重复发送多次
        if (force_clear_irq) {
            constexpr int kMaxSendTimes = 10;
            const char *kIRQString = "0x00";
            for (int i = 0; i < kMaxSendTimes &&
                    resp_->GetBody().find(kIRQString) == std::string::npos;
                    i++) {
                protocal_->SendMessage(req_, resp_);
                LOG(INFO) << "遇到中断发送之后没有正确返回0x00, 这里是" << i << "次发送" << std::endl;
            }
        }
    }
    // TODO 检查返回的消息来决定返回true/false.
    return true;
}

template<typename T>
std::future<bool> DeviceRegister<T>::AsyncWrite(T content) {
    std::future<bool> f = std::async(std::launch::async, [=]() {
        return Write(content);
    });
    return f;
}

template<typename T>
T DeviceRegister<T>::Read() {
    std::unique_lock<std::mutex> l(lock_);
    std::vector<std::string> vs = buildReadMessage();
    // 这里如果有多个消息要发送的时候，最好是采用batch send 模式
    // Protocal那边会有一把锁，直接所有的请求响应完毕才会释放
    // 1. 如果只是发送一个消息，为了效率考虑，这里还是利用sendMessage来进行发送
    // 2. 如果是发送多个消息，才使用batch模式
    if (vs.size() == 1) {
        auto &msg = vs[0];
        req_->SetBody(msg);
        protocal_->SendMessage(req_, resp_);
    } else {
        std::vector<Protocal::Request*> reqs;
        std::vector<Protocal::Response*> resps;
        for (auto &msg : vs) {
            auto q = protocal_->getRequest();
            reqs.push_back(q);
            q->SetBody(msg);
            resps.push_back(protocal_->getResponse());
        }
        protocal_->BatchSendMessage(reqs, resps);
        resp_->SetBody(resps.back()->GetBody());
    }
    // 利用最后一次返回的结果来进行解析
    return DecodeResponse(resp_);
}

template<typename T>
std::future<T> DeviceRegister<T>::AsyncRead() {
    std::future<T> f =  std::async(std::launch::async, [&]() {
        return Read();
    });
    return f;
}

/*
 * 设备寄存器－基于串口通信的寄存器
 *  目前这里串口通信是基本8位的unsigned char.
 *  已经是非常特殊的实现了，不应该有子类了
 */
class DeviceRegisterSerialProtocal : public DeviceRegister<unsigned char> {
  public:
    friend class StaticMapperUtil<unsigned char, DeviceRegisterSerialProtocal>;
    static DeviceRegisterSerialProtocal* GetInstance(unsigned char addr, const char *comName);
  protected:
    virtual std::vector<std::string> buildWriteMessage(unsigned char content) override;
    virtual std::vector<std::string> buildReadMessage() override;
    virtual unsigned char DecodeResponse(Protocal::Response *resp) override;
  protected:
    explicit DeviceRegisterSerialProtocal(unsigned char addr_) :
        DeviceRegister<unsigned char>(addr_) {}
    virtual ~DeviceRegisterSerialProtocal() {}
};

/**
 * 主板上寄存器的地址
 */
#define CONTROL_REG_ADDR 0x00
#define SWITCH1_REG_ADDR 0x01
#define SWITCH2_REG_ADDR 0x02
#define GRATING_COVER_REG_ADDR 0x03
#define SWITCH3_REG_ADDR 0x03
// 步进电机控制寄存器
#define LIGHT_MOTOR_REG_ADDR_HIGH 0x0c  //JM1
#define LIGHT_MOTOR_REG_ADDR_LOW 0x0d
#define FRONT_BACK_MOTOR_REG_ADDR_HIGH 0x0e  //JM2
#define FRONT_BACK_MOTOR_REG_ADDR_LOW 0x0f
#define LEFT_RIGHT_MOTOR_REG_ADDR_HIGH 0x10  //JM3
#define LEFT_RIGHT_MOTOR_REG_ADDR_LOW 0x11
#define UP_DOWN_MOTOR_REG_ADDR_HIGH 0x12 //JM4
#define UP_DOWN_MOTOR_REG_ADDR_LOW 0x13
// 直流电机
#define DIRECT_CURRENT_MOTOR_REG_ADDR_FULL 0x14 //JM5
#define LEFT_RIGHT_MOTOR_DELAY_ADDR 0x15
#define FRONT_BACK_MOTOR_DELAY_ADDR 0x16
#define LIGHT_MOTOR_DELAY_ADDR 0x17
#define UP_DOWN_MOTOR_DELAY_ADDR 0x18
typedef int board_addr_t;

/**
 * 统一使用COM1的寄存器，可读可写带缓存
 */
class RegCOM1 : public DeviceRegisterSerialProtocal {
  public:
    // 这里需要用二进制的方式来指定哪一些需要被清0
    // 比如，如果要清理第0位，那就是0x01
    // 如果要清理第3位。那就是0x04
    virtual void SetBit(int mask);
    virtual void ClearBit(int mask);
    virtual unsigned char CurrentValue();
  protected:
    RegCOM1(board_addr_t addr) :
        DeviceRegisterSerialProtocal((unsigned char)addr),
        value_(0), has_init_(false) {
        serial_port_ = new SerialProtocal("COM1");
        SetProtocal(serial_port_);
        read_times_ = write_times_ = 0;
    }
    virtual ~RegCOM1() {
        delete serial_port_;
    }
  protected:
    virtual unsigned char read_no_lock() {
        read_times_++;
        return Read();
    }
    virtual void write_no_lock(unsigned char val) {
        write_times_++;
        Write(val);
    }
  private:
    unsigned char value_;
    SerialProtocal *serial_port_;
    // 是否和底层硬件打过交道?
    bool has_init_;
    // 上锁，保证多线程正确性
    std::mutex lock_;
    // 硬件上的读写次数
    int write_times_;
    int read_times_;
};

// 电源状态寄存器
// 这里需要查看 主板串口通信协议规范.pdf
// Bit0 :电机控制板的 D_VDD_EN 输出，0 : 关闭，1:打开;
// Bit1 :LED_5V 输出控制，0:关闭，1:打开;
// Bit2 :LED_12V 输出控制，0:关闭，1:打开;
class PowerReg : public RegCOM1 {
  public:
    static PowerReg *GetInstance() {
        static PowerReg p;
        return &p;
    }
    enum {
        POINTS_8 = 0x4 /*12V Bit2*/,
        CIRCLE = 0x02 /*5V Bit1*/,
        MOTOR_CONTROL_POWER = 0x01 /*Bit0*/,
    };
    void OpenLED(int light_id) {
        SetBit(light_id);
    }
    void CloseLED(int light_id) {
        ClearBit(light_id);
    }
  protected:
    virtual std::vector<std::string> buildWriteMessage(unsigned char content) override;
    virtual std::vector<std::string> buildReadMessage() override;
    virtual unsigned char DecodeResponse(Protocal::Response *resp) override;
  private:
    explicit PowerReg() : RegCOM1(0) { }
    virtual ~PowerReg() {}
};

/**
 * 控制寄存器:地址 0x00
 * 控制寄存器：
 * b4: 仪器上下移动步进电机使能
 * b3: 仪器光路移动步进电机使能
 * b2: 仪器前后移动步进电机使能
 * b1: 仪器左右移动步进电机使能
 * b0: EXINT
 */
class ControlReg final : public RegCOM1 {
  public:
    enum {
        LIGHT_MOTOR = 0x02,      //JM1
        FRONT_BACK_MOTOR = 0x04, //JM2
        LEFT_RIGHT_MOTOR = 0x08, //JM3
        UP_DOWN_MOTOR = 0x10,    //JM4
        EXINT = 0x01,
    };
  public:
    static ControlReg *GetInstance();
    // 当发生中断的时候，强制把最低位设置0.
    void ClearIRQ() {
        auto val = CurrentValue();
        // 如果缓存中最低位为1.那么调用ClearBit.
        val &= ~EXINT;
        // 由于硬件的原因，如果中断返回值不是0x00
        // 那么就一直发送清中断命令
        Write(val, false/*force_clear_irq*/);
    }
  private:
    explicit ControlReg() : RegCOM1(CONTROL_REG_ADDR) {}
    virtual ~ControlReg() {}
};

/**
 * 只读寄存器
 * 缓存由底层硬件来更新，每次读的都是旧值
 * 当发生中断的时候，主板的中断函数会自动
 * 把缓存值更新掉。
 */
class ReadOnlyRegCOM1 : public DeviceRegisterSerialProtocal {
  public:
    // 看某一位是否为1
    // 比如，如果要看最后一bit，那么就是
    // CurrentValue & 0x01
    // mask就是0x01
    bool IsCover(int mask) {
        return CurrentValue() & mask;
    }
    // 给上层用的时候，直接返回缓存值
    virtual unsigned char CurrentValue() {
        std::unique_lock<std::mutex> l(lock_);
        if (read_times_ == 0) {
            read_times_++;
            cache_ = DeviceRegisterSerialProtocal::Read();
        }
        return cache_;
    }
    virtual unsigned char Read() override {
        return CurrentValue();
    }
  private:
    // 只读寄存器，把写的接口不用
    virtual bool Write(unsigned char content, bool force_clear_irq=false) override {
        RM_ASSERT(0);
        return false;
    }
    // 只读寄存器，把写的接口不用
    virtual std::future<bool> AsyncWrite(unsigned char content) override {
        RM_ASSERT(0);
        std::future<bool> f =  std::async(std::launch::async, [=]() {
            return Write(content);
        });
        return f;
    }
  protected:
    ReadOnlyRegCOM1(board_addr_t addr) :
        DeviceRegisterSerialProtocal((unsigned char)addr) {
        serial_port_ = new SerialProtocal("COM1");
        SetProtocal(serial_port_);
        read_times_ = 0;
    }
    virtual ~ReadOnlyRegCOM1() {
        delete serial_port_;
    }
  protected:
    virtual unsigned char read_no_lock() {
        std::unique_lock<std::mutex> l(lock_);
        read_times_++;
        cache_ = DeviceRegisterSerialProtocal::Read();
        return cache_;
    }
  private:
    // 底层中断会调用这个函数来进行更新
    friend class SerialBoard;
    void refresh_cache() {
        read_no_lock();
    }
  private:
    SerialProtocal *serial_port_;
    // 上锁，保证多线程正确性
    std::mutex lock_;
    // 硬件上的读写次数
    int read_times_;
    // 缓存值,只能由底层硬件来更新
    volatile unsigned char cache_ = 0;
};

// 这个寄存器只有8位
class SwitchReg final : public ReadOnlyRegCOM1 {
  public:
    // 这里是给马达光栅用的几个bit位，实际上是和下面的B0 ~ B7是可以对应上的
    // 只是这里为了更加方便使用，所以进行了重新命名
    enum {
      FRONT_BACK_MOTOR_BIT_POS = 0x04, // B2
      UP_DOWN_MOTOR_BIT_POS = 0x40, // B6
      LEFT_RIGHT_MOTOR_BIT_POS = 0x10, // B4
      LIGHT_MOTOR_BIT_POS = 0x01, // B0
    };
  public:
    enum {
        B7 = 0x80,
        B6 = 0x40,
        B5 = 0x20,
        B4 = 0x10,
        B3 = 0x08,
        B2 = 0x04,
        B1 = 0x02,
        B0 = 0x01,
    };
  public:
    friend class StaticMapperUtil<board_addr_t, SwitchReg>;
    static SwitchReg *GetInstance(board_addr_t addr);
    std::string to_string();
  private:
    explicit SwitchReg(board_addr_t addr) : ReadOnlyRegCOM1(addr) {
        // 只能用于这三个地址
        RM_ASSERT(
            addr == SWITCH1_REG_ADDR ||
            addr == SWITCH2_REG_ADDR ||
            addr == SWITCH3_REG_ADDR
        );
    }
    virtual ~SwitchReg() {}
};

/**
 * 统一使用COM1的寄存器， 步进电机控制寄存器
 * 不能有缓存
 */
class StepReg : public DeviceRegisterSerialProtocal {
  public:
    friend class StaticMapperUtil<board_addr_t, StepReg>;
    static StepReg *GetInstance(board_addr_t addr);
    virtual unsigned char CurrentValue();
  protected:
    StepReg(board_addr_t addr) :
        DeviceRegisterSerialProtocal((unsigned char)addr) {
        // 由于硬件板的设置，这里只接收到以下地址
        int addrs[] = {
            LEFT_RIGHT_MOTOR_REG_ADDR_HIGH,
            LEFT_RIGHT_MOTOR_REG_ADDR_LOW,
            LEFT_RIGHT_MOTOR_DELAY_ADDR,
            FRONT_BACK_MOTOR_REG_ADDR_HIGH,
            FRONT_BACK_MOTOR_REG_ADDR_LOW,
            FRONT_BACK_MOTOR_DELAY_ADDR,
            LIGHT_MOTOR_REG_ADDR_HIGH,
            LIGHT_MOTOR_REG_ADDR_LOW,
            LIGHT_MOTOR_DELAY_ADDR,
            UP_DOWN_MOTOR_REG_ADDR_HIGH,
            UP_DOWN_MOTOR_REG_ADDR_LOW,
            UP_DOWN_MOTOR_DELAY_ADDR,
            DIRECT_CURRENT_MOTOR_REG_ADDR_FULL,
        };
        // 一定要在给定的列表中
        bool find = false;
        for (auto &item: addrs) {
            if (item == addr) {
                find = true;
                break;
            }
        }
        RM_ASSERT(find);

        serial_port_ = new SerialProtocal("COM1");
        SetProtocal(serial_port_);
    }
    virtual ~StepReg() {
        delete serial_port_;
    }
  protected:
    virtual unsigned char read_no_lock() {
        return Read();
    }
    virtual void write_no_lock(unsigned char val) {
        Write(val);
    }
  private:
    SerialProtocal *serial_port_;
    // 上锁，保证多线程正确性
    std::mutex lock_;
};

// 这个类里面包含了目前主板上定义的所有的寄存器。
class SerialBoard {
  public:
    static SerialBoard *GetInstance() {
        static SerialBoard board;
        return &board;
    }
  public:
    ControlReg *ctrl_;
    SwitchReg *switch1_;
    SwitchReg *switch2_;
    SwitchReg *switch3_;
    SwitchReg *grating_cover_; // same to swtich3_;
    StepReg *lr_motor_high_;
    StepReg *lr_motor_low_;
    StepReg *lr_motor_delay_;
    StepReg *fb_motor_high_;
    StepReg *fb_motor_low_;
    StepReg *fb_motor_delay_;
    StepReg *light_motor_high_;
    StepReg *light_motor_low_;
    StepReg *light_motor_delay_;
    StepReg *ud_motor_high_;
    StepReg *ud_motor_low_;
    StepReg *ud_motor_delay_;
    PowerReg *power_;
    StepReg *direct_current_;
  public:
    // 中断响应函数。主板级别的中断响应函数就是
    // 只复位EXINT位。
      static int handleIRQ(void* data);
  private:
    SerialBoard() {
        ctrl_ = ControlReg::GetInstance();
        // 刚进来的时候，一定要记得初始化。
        ctrl_->Write(0);
        ctrl_->Write(0);
        switch1_ = SwitchReg::GetInstance(SWITCH1_REG_ADDR);
        switch1_->refresh_cache();
        switch2_ = SwitchReg::GetInstance(SWITCH2_REG_ADDR);
        switch2_->refresh_cache();
        switch3_ = SwitchReg::GetInstance(SWITCH3_REG_ADDR);
        switch3_->refresh_cache();
        grating_cover_ = SwitchReg::GetInstance(SWITCH3_REG_ADDR); // same to swtich3_;

        lr_motor_high_ = StepReg::GetInstance(LEFT_RIGHT_MOTOR_REG_ADDR_HIGH);
        lr_motor_low_ = StepReg::GetInstance(LEFT_RIGHT_MOTOR_REG_ADDR_LOW);
        lr_motor_delay_ = StepReg::GetInstance(LEFT_RIGHT_MOTOR_DELAY_ADDR);

        fb_motor_high_ = StepReg::GetInstance(FRONT_BACK_MOTOR_REG_ADDR_HIGH);
        fb_motor_low_ = StepReg::GetInstance(FRONT_BACK_MOTOR_REG_ADDR_LOW);
        fb_motor_delay_ = StepReg::GetInstance(FRONT_BACK_MOTOR_DELAY_ADDR);

        light_motor_high_ = StepReg::GetInstance(LIGHT_MOTOR_REG_ADDR_HIGH);
        light_motor_low_ = StepReg::GetInstance(LIGHT_MOTOR_REG_ADDR_LOW);
        light_motor_delay_ = StepReg::GetInstance(LIGHT_MOTOR_DELAY_ADDR);

        ud_motor_high_ = StepReg::GetInstance(UP_DOWN_MOTOR_REG_ADDR_HIGH);
        ud_motor_low_ = StepReg::GetInstance(UP_DOWN_MOTOR_REG_ADDR_LOW);
        ud_motor_delay_ = StepReg::GetInstance(UP_DOWN_MOTOR_DELAY_ADDR);

        // 直接电机寄存器
        direct_current_ = StepReg::GetInstance(DIRECT_CURRENT_MOTOR_REG_ADDR_FULL);
        power_ = PowerReg::GetInstance();

        // 注册主板的中断。
        // 主板的中断是挂在了永久中断函数上。
        // 一担接收到了光栅信号就会被触发
        IRQVector *irqs = IRQVector::GetInstance();
        irqs->AppendIRQFunc(GRATING_COVER, irq_vector_item_t::irq_handler_t{
            SerialBoard::handleIRQ, nullptr
        });
    }
};

} // end of namespace RM.
#endif  // end NCE_BOARD_H
