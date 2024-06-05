#ifndef __CWM_TEST_H__
#define __CWM_TEST_H__

#ifdef __cplusplus
extern "C"{
#endif

#define ALGO_TEST_EN        1

#if defined(ALGO_TEST_EN) && (1 == ALGO_TEST_EN)
void algo_test(void);
#endif

#ifdef __cplusplus
}
#endif

#endif




