/**
 * @file simpletest.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Super simple unit testing framework for C
 * @version 0.1
 * @date 2022-02-03
 *
 * @copyright Copyright (c) 2022
 *
 * ## Overview
 * This extremely simple library aims to be simple and functional, allowing you
 * to write unit tests quickly, without getting your way.
 * This package incorporates seamlessly into CTest and is extremely lightweight.
 *
 * ## Usage
 * To use, just wrap any boolean expressing in the `TEST` macro to check if it's
 * true or not. If true, it counts as a passed test, otherwise it fails.
 * Then, after calling all your test code, return the output of TestResult()
 * from your main function. Optionally, you can use PrintTestStatus() before you
 * return to print the results.
 *
 * For convenience, this package also provides the TESTAPPROX(a,b,tol) macro
 * which calls
 * ~~~
 * TEST(fabs(a - b) < tol)
 * ~~~
 * which is useful for comparing floating point numbers.
 *
 * Any failed tests will show the expression that failed, along with the file
 * and line number of test.
 *
 * ## Example
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #include "simpletest.h""
 *
 * void MyTestFunction() {
 *   TEST(2 == 1 + 1);
 *   TEST(4 == 2 * 2);
 *   TESTAPPROX(1.2, 2.4 / 2, 1e-8);
 * }
 *
 * int main() {
 *   PrintTestResult();
 *   return TestResult();
 * }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 */
#pragma once
#include <math.h>

#define TEST(test)                          \
  do {                                      \
    if (!(test)) {                          \
      TestFail();                           \
      PrintFail(#test, __FILE__, __LINE__); \
    } else {                                \
      TestPass();                           \
    }                                       \
  } while (0)

#define TESTAPPROX(a, b, tol) TEST(fabs((a) - (b)) < tol)

void TestPass();
void TestFail();
int TestResult();
void PrintTestResult();
void PrintFail(const char* expr, const char* files, int line);
