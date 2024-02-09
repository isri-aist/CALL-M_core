#ifndef func_h
#define func_h

float constrain(float value, float min, float MAX);

unsigned long millis(void);

int nscanf(const char *str, const char *fmt, ...)
  __attribute__ ((format (scanf, 2, 3)));

#endif
