#include "simpletest.h"

#include <stdio.h>

int tests_total = 0;
int tests_passed = 0;

void ResetTests() {
  tests_total = 0;
  tests_passed = 0;
}

void TestFail() { ++tests_total; }

void TestPass() {
  ++tests_total;
  ++tests_passed;
}

int TestResult() { return !(tests_passed == tests_total); }

void PrintTestResult() {
  printf("Passed %d / %d tests.\n", tests_passed, tests_total);
  if (tests_passed == tests_total) {
    printf("ALL TESTS PASSED!\n");
  }
}

void PrintFail(const char* expr, const char* file, int line) {
  printf("TEST FAILED! %s\n", expr);
  printf("%s%s:%d\n", "             ", file, line);
}
