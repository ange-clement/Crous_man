#ifndef BITMAP_HPP
#define BITMAP_HPP

#include "SystemIDs.hpp"

class Bitmap {
public:
    unsigned int bitmap;

public:
    Bitmap();
    ~Bitmap();

    Bitmap(unsigned int bitmap);

    Bitmap(std::initializer_list<SystemIDs> systems);

    void loadFromSystemIDS(std::initializer_list<SystemIDs> systems);
    void addId(SystemIDs other);
    void removeId(SystemIDs other);
    void addBitmap(const Bitmap* other);
    void removeBitmap(const Bitmap* other);

    Bitmap* combine(const Bitmap* other);

    bool equals(const Bitmap* other);
};

#endif