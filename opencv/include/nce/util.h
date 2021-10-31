#ifndef RM_UTIL_H
#define RM_UTIL_H

#undef NDEBUG
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef _MSC_VER
#include <locale.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#ifdef _WIN32
#include <windows.h>

#include <atomic>
#endif

#include <chrono>
#include <iostream>

#ifndef LOG
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#define ERROR 0
#define WARNING 1
#define INFO 2

inline std::string getTimeStampString(void) {
    using std::chrono::system_clock;
    std::time_t tt = system_clock::to_time_t(system_clock::now());
    struct std::tm valtm;
#ifdef _WIN32
    localtime_s(&valtm, &tt);
#else
    localtime_r(&tt, &valtm);
#endif
    std::stringstream ss;
    ss << std::put_time(&valtm, "%F %X");
    return ss.str();
}

class logger {
private:
    std::fstream out;
    logger(void) {
        out.open("rm.log.txt", std::fstream::out | std::fstream::binary);
    }

    ~logger() {
        out.close();
    }
public:
    static std::fstream& GetInstance(int x) {
        static logger log;

        log.out << getTimeStampString() << " ";

        if (x == ERROR) {
            log.out << "[ERROR] ";
        } else if (x == WARNING) {
            log.out << "[WARNING] ";
        } else if (x == INFO) {
            log.out << "[INFO] ";
        }

        log.out.flush();
        return log.out;
    }
};

#define LOG(x) logger::GetInstance(x) << __FILE__                   \
                                      << ":"                        \
                                      << __LINE__                   \
                                      << " "                        \
                                      << __FUNCTION__ << "() "
#endif

#define BEGIN_POINT(x) auto x = std::chrono::system_clock::now()
#define END_POINT(y, x, msg)                                             \
    do {                                                                 \
        auto y = std::chrono::system_clock::now();                       \
        auto diff =                                                      \
            std::chrono::duration_cast<std::chrono::nanoseconds>(y - x); \
        LOG(INFO) << msg << " " << diff.count() / 1000000 << " (ms)"     \
                  << std::endl;                                          \
    } while (0)

namespace RM {
// begin of namesapce RM
extern int left_eye_pos_g;
extern int right_eye_pos_g;
inline void Sleep(int ms) {
    if (ms <= 0) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

#define WITH_NO_COPY_CLASS(C)          \
   private:                            \
    C(const C &&p) = delete;           \
    C(const C &p) = delete;            \
    C &operator=(const C &p) = delete; \
    C &operator=(C &&p) = delete

#define WITH_DEFAULT_CLASS(C)           \
   public:                              \
    C(C &&p) = default;                 \
    C(const C &p) = default;            \
    C &operator=(const C &p) = default; \
    C &operator=(C &&p) = default

#define UNUSED(expr)  \
    do {              \
        (void)(expr); \
    } while (0)

template <typename T>
class defer {
   public:
};
// 这个类的主要作用：
// 单件设计模式里面：GetInstance(int index)
// 不同的index会生成不同的值，这个mapper类会记录
// 相应的index对象，负责相应的create/destroy.
template <typename index, typename Object>
class StaticMapperUtil {
    std::unordered_map<index, Object *> *pmap_;
    std::mutex lock_;

   public:
    StaticMapperUtil() { pmap_ = new std::unordered_map<index, Object *>(); }
    ~StaticMapperUtil() {
        RM::Sleep(400);
        std::cout << "delete StaticMapperUtil()" << std::endl;
        // Here would exit the system. No need to free the resource.
        // let OS free it.
        /*
        for (auto & item : *pmap_) {
            if (item.second)
                delete item.second;
        }
        delete pmap_;
        */
    }
    Object *get(index name) {
        std::lock_guard<std::mutex> l(lock_);
        auto iter = pmap_->find(name);
        if (iter == pmap_->end()) {
            Object *p = new Object(name);
            (*pmap_)[name] = p;
            return p;
        }
        return iter->second;
    }
};

// 删除字符串中的所有的空白符以及回车等等
void delete_space(std::string &str);

// 把字符串切分成为多个token.
// <abc><abc> => ["<abc>", "<abc>"]
std::vector<std::string> split_token(std::string &s, const char left = '<',
                                     const char right = '>');

// 添加空白符
// <return0xff> => <return 0xff>
void add_space(std::string &str);

/**
 * @brief 提取中断信息
 *
 * 由于中断函数的随机性。为了仿止出现
 * @par 比如：
 * @code
 *      std::string str = "<return<replace>0xff>"
 * @endcode
 *
 * 这种不容易解析的字符串。这里需要将replace提取出来。
 *
 * @param[in] str     需要处理的字符串
 * @param[in] replace 需要提取的字符串
 * @note 输入字符串已经删除了空白符
 *
 * @return 返回所有的replace string vector.
 */
std::vector<std::string> fetch_string(std::string &str,
                                      const std::string &replace);

// 把一个数输出为二进制数，比如0x01 == 0b0000,0000,0000,0001
template <typename T>
std::string to_string_0b(T v) {
    const int BITS_IN_BYTE = 8;
    const int n = sizeof(v) * BITS_IN_BYTE;
    const int BUFSIZE = 100;
    char buf[BUFSIZE];
    int i = 0;
    int cnt = 0;
    memset(buf, 0, BUFSIZE);
    for (i = 0; i < (n + n / 4 - 1); i++, cnt++) {
        // 为了更好阅读，每4个bit输出一个逗号
        if (cnt > 0 && (cnt % 4 == 0)) {
            buf[i++] = ',';
        }
        buf[i] = '0';
        if (v & 0x01) {
            buf[i] = '1';
        }
        v >>= 1;
    }
    buf[i++] = 'B';
    buf[i++] = '0';
    std::reverse(buf, buf + i);
    return std::string(buf);
}

#ifdef _MSC_VER
wchar_t *c2w(const char *pc);
char *w2c(const wchar_t *pw);
int KillProcess(const char *procName);
#endif


#define RM_ASSERT(x)                                                         \
    do {                                                                     \
        if (!(x)) {                                                          \
            LOG(ERROR) << __FILE__ << ":" << __LINE__ << ":" << __FUNCTION__ \
                       << "(ASSERT_ERROR)" << std::endl;                     \
            assert(x);                                                       \
        }                                                                    \
    } while (0)

// interface to get camera number.
int CameraNumber(void);

// test COM1 is exist or not.
// if not find, the thread.dll & export.cpp
// would not try to init the system's motor.
bool isCOMOnTheSystem(std::string comName = "COM1");

// list all the video files under the dir.
std::vector<std::string> scan_dir(std::string &path);


#ifdef _WIN32
std::string getTempFileName(std::string prefix);
#endif

}  // end of namespace RM

#endif
