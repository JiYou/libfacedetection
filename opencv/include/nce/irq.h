#ifndef _NCE_IRQ_H
#define _NCE_IRQ_H

#include "nce/util.h"
#include <string>
#include <list>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <string.h>
#include <unordered_map>
#include <assert.h>

namespace RM {
// begin of namesapce RM

// 这里设置中断向量的原因是由于底层硬件的中断是通过同一个串口上来的
// 中断向量的种类
typedef enum {
    // 光栅中断

    // 这个是通用的光栅中断，只要有信号过来，挂在这个中断链上的
    // 中断函数都会被执行
    GRATING_COVER = 0x00,
    // 左右光栅中断，只处理这个中断链上的中断信息
    LEFT_RIGHT_GRATING_COVER = 0x01,
    UP_DOWN_GRATING_COVER = 0x02,
    FRONT_BACK_GRATING_COVER = 0x03,
    LIGHT_GRATING_COVER = 0x04,
    MAX_IRQ_NUMBER,
} IRQ_TYPE_t;

// 中断向量表中的一项
struct irq_vector_item_t {
    typedef int (*callback_t)(void *data);
    IRQ_TYPE_t irq_number_;

    struct irq_handler_t {
        callback_t f_;
        void *data_;
    };
    std::list<irq_handler_t> handlers_;

    int operator()(void);
    const int operator()(void) const;
    bool append(irq_handler_t t);
};

// 中断向量表
class IRQVector {
    WITH_NO_COPY_CLASS(IRQVector);
  public:
    // "<event #1>"
    const static std::string GRATING_COVER_STR;
  public:
    static IRQVector *GetInstance();
    const IRQ_TYPE_t SearchIRQ(const std::string &str) const;
    const irq_vector_item_t&GetIRQ(const IRQ_TYPE_t t) const;

    void SetIRQ(IRQ_TYPE_t t, const irq_vector_item_t &item);
    void SetIRQ(const std::string &str, const irq_vector_item_t &item);

    void AppendIRQFunc(IRQ_TYPE_t t, irq_vector_item_t::irq_handler_t handler);
    void AppendIRQFunc(const std::string &str, irq_vector_item_t::irq_handler_t handler);

    const irq_vector_item_t &RunIRQ(IRQ_TYPE_t t);
    const irq_vector_item_t &RunIRQ(std::string &str);

    void AddTempIRQFunc(IRQ_TYPE_t t, irq_vector_item_t::irq_handler_t handler);
    void AddTempIRQFunc(const std::string &str, irq_vector_item_t::irq_handler_t handler);
  private:
    // 一些临时的，只运行一次的中断响应函数
    // 运行完成之后，就把要相应的item remove掉
    // 如果注册的中断处理函数里面包含了局部变量栈中变量，那么一定要小心!!
    // 安全起见，一定要等中断响应做完才可以退出函数或者线程。
    irq_vector_item_t temp_irq_[MAX_IRQ_NUMBER];
    // 中断向量表: 中断向号->函数
    // 这里注册的函数不能是局部变量。必须是全局静态函数
    irq_vector_item_t irq_table_[MAX_IRQ_NUMBER];
    // 如果需要修改中断向量表，需要先上锁
    std::mutex *irq_lock_map_[MAX_IRQ_NUMBER];

    // 串口中断过来的时候，不是一个数值，而一个字符串
    // 所以设置这么一个hash供查询中断号
    // 正常的系统里面可以不用这个
    // 串口接收到的中断消息
    // 这个中断消息是由硬件协议固定的
    // 所以一经初始化，就不可更改
    const std::unordered_map<std::string, IRQ_TYPE_t> irq_hash_map_ = {
        {GRATING_COVER_STR, GRATING_COVER},
    };
  private:
    IRQVector();
    ~IRQVector();
};

} // end of namesapce RM
#endif
