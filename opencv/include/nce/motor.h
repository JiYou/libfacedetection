#ifndef _MOTOR_H
#define _MOTOR_H

#include "nce/board.h"
#include "nce/util.h"

#include <stdio.h>
#include <map>
#include <mutex>


namespace RM {
// begin of namespace RM

class Motor {
  public:
    // 清除马达内部的数据，注意与reset的区别
    virtual void Clear(void) {
        clear();
    }
    // 读取马达内部的数据。比如余下要走的步数
    // d表示去取正转还是反转
    virtual int Read(bool *d = nullptr) {
        return read(d);
    }
    // 最后一次转动的方向
    virtual bool Direction() {
        return direction_;
    }

  public:
    // 移动马达，返回移动的步数
    // d表示方向，只有正转/反转两种
    // 这里要一直移动，直接走到0
    virtual void Move(bool d, int steps);
    virtual std::future<void> AsyncMove(bool d, int steps) {
        return std::async(std::launch::async, [ = ]() {
            return Move(d, steps);
        });
    }
    // move with delay setting.
    virtual void Move(bool d, int steps, int time);
    virtual std::future<void> AsyncMove(bool d, int steps, int time) {
        return std::async(std::launch::async, [ = ]() {
            return Move(d, steps, time);
        });
    }
    // 马达复位, 返回走的步数
    // 复位并不是说把内部数据清0，而是说要走到光偶开关那里
    virtual int Reset(void);
    virtual std::future<int> AsyncReset() {
        return std::async(std::launch::async, [&]() {
            return Reset();
        });
    }
    // clear all counters
    virtual void ClearCounter(void) {
        std::unique_lock<std::mutex> l(lock_);
        total_steps_ = 0;
        pos_ = 0;
        // true表示高位被置1，false表示没有置1.
        direction_ = false;
    }

  public:
    // 异步操作，可以在一个线程里面支持异步操作
    // 但是需要在外部上锁
    // Usage:
    // std::lock_guard l(motor.AquireLock());
    // motor.SendMoveCommand()
    // // can do some thing.
    // if (motor.IsCommandRunningOver()) {
    //     // the motor still running.
    // } else {
    //     // the motor is not running over.
    // }
    std::mutex &AquireLock() {
        return lock_;
    }
    // 记录命令发送过来的时候的指令里面记走了多少步
    int triggered_steps_ = 0;
    // 注意使用std::lock_guard l(motor.AquireLock());上锁
    void SendMoveCommand(bool d, int steps);
    int IsCommandRunningOver();

  public:
    // 获取当前位置,注意有正负
    virtual int CurrentPosition() const {
        return pos_;
    }
    // 总共移动的步数，不关心正负
    virtual size_t TotalMovedSteps(void) const {
        return total_steps_;
    }

  public:
    virtual ~Motor() {}
    virtual void PowerOff();
    virtual bool PowerStatus() {
        return power_status();
    }

  public:
    // 这个留给子类实现
    virtual int HardReset() = 0;

  protected:
    // 这个留给子类实现
    virtual void power_on() = 0;
    virtual void power_off() = 0;
    virtual bool power_status() = 0;
    virtual void clear() = 0;
    virtual int read(bool *d) = 0;
    virtual void delay(int time) = 0;
    virtual void move(bool d, int steps) = 0;
    virtual void move(bool d, int steps, int time) = 0;

  public:
    // 0 is hardware default value,
    // is the smallest value, should not be smaller than it.
    const int motor_default_delay_time = 0x07;
  protected:
    volatile int total_steps_ = 0;
    std::atomic<int> pos_{0};
    // true表示高位被置1，false表示没有置1.
    bool direction_ = false;
    std::mutex lock_;
};

// 每种马达目前都是只有一个的。
// 并且这些马达都会被控制寄存器管理起来。
class SerialMotor : public Motor {
  public:
    friend class StaticMapperUtil<int, SerialMotor>;
    static SerialMotor *GetInstance(int motor_type);

  public:
    virtual int HardReset() override;
    // 第一位为二进制最右边的一位
    // 注意：这里使用掩码，比如第3位用0b100来表示
    virtual void SetSwitchRegister(RM::SwitchReg *sw, int mask);
    virtual bool IsSwitchCover();

  protected:
    virtual void power_on() override {
        ctrl_->SetBit(bit_pos_);
    }
    virtual void power_off() override {
        ctrl_->ClearBit(bit_pos_);
    }
    virtual bool power_status() override {
        return (ctrl_->Read() & bit_pos_) > 0;
    }
    virtual void clear() override {
        high_->Write(0);
        low_->Write(0);
    }
    virtual int read(bool *d) override;
    virtual void delay(int time) override;
    virtual void move(bool d, int steps) override;
    virtual void move(bool d, int steps, int time) override;

  private:
    SerialMotor(int motor_type);
    virtual ~SerialMotor() {}

  private:
    SerialBoard *board_;
    StepReg *high_;
    StepReg *low_;
    StepReg *delay_;
    ControlReg *ctrl_;
    // 控制寄存器上的控制位
    int bit_pos_;

  private:
    typedef enum {
        // TODO: 采用档片的另外一边来做判断，但后面档片还需要加长点
        LEFT_RIGHT_MOTOR = false,  // true->left, false->right
        FRONT_BACK_MOTOR = false,  // true->front, false->back
        LIGHT_MOTOR = true,
        UP_DOWN_MOTOR = false  // false->up, true->down
    } Motor_Direction_t;

    //  四种马达各自的复位函数
    void motor_reset_step(int motor_type, IRQ_TYPE_t irq_type, bool direction,
                          int step, int sleepTime, bool once);
    void light_motor_reset_step(int step, int sleepTime, bool once);
    int light_motor_reset();
    int left_right_motor_reset();
    int up_down_motor_reset();
    int front_back_motor_reset();

    std::mutex switch_lock_;
    // 查看是否被档住
    SwitchReg *switch_reg_ = nullptr;

    // 控制位
    int switch_bit_pos_ = 0;
    // 光栅被挡住时，马达要走的方向
    std::atomic<bool> has_set_switch_{false};
};

class DirectCurrentMotor : public Motor {
  public:
    friend class StaticMapperUtil<int, DirectCurrentMotor>;
    static DirectCurrentMotor *GetInstance(int motor_type);

  public:
    // 控制直流电机转动，dir 表示转向，is_btn_up 表示是否需要停止
    // dir=1 -> 正转, 0->反转
    // is_btn_up=1->放开, 0 -> 按下
    // 短按走马达
    virtual void Move(bool dir);
    // 长按走马达
    virtual void Move(bool dir, bool is_btn_up);

  protected:
    virtual int HardReset() override;
    virtual void power_on() override;
    virtual void power_off() override;
    virtual bool power_status() override;
    virtual void clear() override;
    virtual int read(bool *d) override;
    virtual void delay(int time) override;
    virtual void move(bool d, int steps) override;
    virtual void move(bool d, int steps, int time) override;

  private:
    SerialBoard *board_ = nullptr;
    StepReg *reg_ = nullptr;

  private:
    DirectCurrentMotor(int motor_type);
    virtual ~DirectCurrentMotor() {}
};

}  // namespace RM

#endif  // _MOTOR_H
