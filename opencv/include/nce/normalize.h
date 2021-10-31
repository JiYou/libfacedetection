#ifndef _RM_NORMALIZE_H
#define _RM_NORMALIZE_H

// pre declare;
struct Config;  // config.h
struct RefData; // button.h

/*
 * 功能：重新规一化测量的结果
 * 主要是根据机器的配置来重新归一化测量的结果
 *
 * 参数：
 * - config: 保存的配置
 * - measure_result: 最初的测量结果
 *
 * NOTE: 输入结果扔然保存在measure_result里面
 *
 */
void normalize_measure_result(
    const struct Config *confg,
    struct RefData *measure_result /*OUT*/
);

#endif /*!_RM_NORMALIZE_H*/