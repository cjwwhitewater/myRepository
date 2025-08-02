#pragma once


class Attacker{
public:
    void singleShot();
    void multipleShots();

private:
    // Pin ID of the GPIO port of the nano machine
    const int pinNumber = 29;

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
