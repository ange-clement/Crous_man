#ifndef BITMAP_HPP
#define BITMAP_HPP

#include "SystemIDs.hpp"

class Bitmap {
public:
    unsigned int bitmap;

public:
    Bitmap();

    Bitmap(unsigned short bitmap);

    Bitmap(std::initializer_list<SystemIDs> systems);

    void loadFromSystemIDS(std::initializer_list<SystemIDs> systems);
    void addId(SystemIDs other);

    Bitmap* combine(const Bitmap* other);

    bool equals(const Bitmap* other);
};

#endif