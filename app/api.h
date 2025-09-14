#ifndef GPIO_API_H
#define GPIO_API_H

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

int GPIOExport(int pin);
int GPIOUnexport(int pin);
int GPIODirection(int pin, const char *dir);
int GPIORead(int pin);
int GPIOWrite(int pin, int value);

#endif