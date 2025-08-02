#pragma once


class Attacker{
public:
    // 激光单次发射（高电平持续一段时间再拉低）
    void singleShot(int duration_ms = 100);
    // 激光多次发射，默认发射三次，每次发射持续100ms，间隔100ms
    void multipleShots(int times = 3, int duration_ms = 100, int interval_ms = 100);

private:
    // Pin ID in the GPIO port of jetson machine.
    const int pinNumber = 32;   // physical pin ID.

    // runtime singleton
public:
    static Attacker* createInstance();
    static Attacker* get();
private:
    Attacker();
    ~Attacker();

private:
    static Attacker* singleton;
};
